import asyncio
import time
import uuid
from typing import Generic, TypeVar

import asyncio_for_robotics.zenoh as afor
import numpy as np
import zenoh
from nptyping import NDArray, Shape, UInt8
from ros2_pyterfaces.cydr.idl import types

from .builtin_msgs import Attachment
from .pub import publisher_keyexpr
from .qos import QosProfile
from .service_common import ServiceType, qualify_service_name, token_keyexpr
from .utils import TopicInfo, resolve_liveliness_context, rmw_zenoh_gid, ros_type_to_dds_type

_ReqT = TypeVar("_ReqT")
_ResT = TypeVar("_ResT")


class Client(Generic[_ReqT, _ResT]):
    """RMW_ZENOH-compatible ROS service client."""

    def __init__(
        self,
        msg_type: type[ServiceType[_ReqT, _ResT]],
        topic: str,
        qos_profile: QosProfile | None = None,
        session: zenoh.Session | None = None,
        domain_id: int | str | None = None,
        namespace: str = "%",
        node_name: str | None = None,
        defer: bool = False,
        timeout: float = 10.0,
        _service_hash: str | None = None,
        _enclave: str = "%",
        _node_id: str | int | None = None,
        _zenoh_id: str | None = None,
        _entity_id: int | str | None = None,
    ):
        ctx = resolve_liveliness_context(
            session=session,
            domain_id=domain_id,
            namespace=namespace,
            _enclave=_enclave,
            _node_id=_node_id,
            _zenoh_id=_zenoh_id,
            _entity_id=_entity_id,
        )
        self.session = ctx.session
        self.namespace = ctx.namespace
        self._enclave = ctx.enclave
        self.domain_id = ctx.domain_id
        self._zenoh_id = ctx.zenoh_id
        self.node_name = (
            f"ros_ez_{uuid.uuid4().hex[:8]}" if node_name is None else node_name
        )
        self._node_id = ctx.node_id
        self._entity_id = ctx.entity_id
        self.timeout = timeout
        self._event_loop = asyncio.get_event_loop()

        qos_profile = QosProfile.services() if qos_profile is None else qos_profile.normalized()
        self.service_type = msg_type
        self.service_name = qualify_service_name(topic, self.namespace, self.node_name)
        self.dds_type = ros_type_to_dds_type(msg_type.get_type_name())
        self.hash = _service_hash if _service_hash is not None else msg_type.hash_rihs01()
        self.topic_info: TopicInfo[type[ServiceType[_ReqT, _ResT]]] = TopicInfo(
            topic=self.service_name, msg_type=msg_type, qos=qos_profile
        )

        self.token: zenoh.LivelinessToken | None = None
        self.zenoh_cli: zenoh.Querier | None = None
        self._pending_requests: set[asyncio.Future[_ResT]] = set()
        self.count = types.int64(0)
        self.gid: NDArray[Shape["16"], UInt8] = np.frombuffer(
            rmw_zenoh_gid(self.token_keyexpr), dtype=np.uint8, count=16
        )
        if not defer:
            self.declare()

    async def async_bind(self) -> None:
        """Bind this client lifetime to the current task."""
        try:
            if self.token is None:
                self.declare()
            await asyncio.Future()
        finally:
            self.close()

    def declare(self) -> None:
        """Declare the service client on Zenoh and the ROS graph."""
        ses = afor.auto_session(self.session)
        self.token = ses.liveliness().declare_token(self.token_keyexpr)
        self.zenoh_cli = ses.declare_querier(
            self.service_keyexpr,
            target=zenoh.QueryTarget.ALL_COMPLETE,
            consolidation=zenoh.ConsolidationMode.NONE,
            timeout=self.timeout,
        )

    def undeclare(self) -> None:
        """Undeclare the service client and cancel pending requests."""
        for future in tuple(self._pending_requests):
            if not future.done():
                future.cancel()
        self._pending_requests.clear()
        if self.token is not None:
            self.token.undeclare()
            self.token = None
        if self.zenoh_cli is not None:
            self.zenoh_cli.undeclare()
            self.zenoh_cli = None

    def close(self) -> None:
        self.undeclare()

    async def wait_for_service(self, polling_rate: float = 0.25) -> None:
        """Wait until at least one matching service server is visible."""
        if self.zenoh_cli is None:
            raise ValueError("Client not declared.")
        while True:
            if self.zenoh_cli.matching_status.matching:
                return
            await asyncio.sleep(polling_rate)

    def call(self, req: _ReqT) -> asyncio.Future[_ResT]:
        """Send a service request and return a future for the response."""
        if self.zenoh_cli is None:
            raise ValueError("Client not declared.")

        request_future: asyncio.Future[_ResT] = self._event_loop.create_future()
        self._pending_requests.add(request_future)
        cancellation_token = zenoh.CancellationToken()

        def on_timeout() -> None:
            if request_future.done():
                return
            cancellation_token.cancel()
            request_future.set_exception(
                TimeoutError(f"Timed out waiting for response on {self.service_name}")
            )

        timeout_handle = self._event_loop.call_later(self.timeout, on_timeout)

        def handle_reply(reply: zenoh.Reply) -> None:
            self._event_loop.call_soon_threadsafe(
                self._resolve_reply,
                request_future,
                timeout_handle,
                cancellation_token,
                reply,
            )

        def on_done(_future: asyncio.Future[_ResT]) -> None:
            cancellation_token.cancel()
            timeout_handle.cancel()
            self._pending_requests.discard(request_future)

        request_future.add_done_callback(on_done)

        attachment = Attachment(
            sequence_number=self.count,
            source_timestamp=types.int64(time.time_ns()),
            source_gid=self.gid,
        ).serialize()[4:]
        self.count += 1
        self.zenoh_cli.get(
            handle_reply,
            payload=req.serialize(),
            encoding=zenoh.Encoding.APPLICATION_CDR,
            attachment=attachment,
            cancellation_token=cancellation_token,
        )
        return request_future

    def _resolve_reply(
        self,
        request_future: asyncio.Future[_ResT],
        timeout_handle: asyncio.Handle,
        cancellation_token: zenoh.CancellationToken,
        reply: zenoh.Reply,
    ) -> None:
        if request_future.done():
            return
        timeout_handle.cancel()
        cancellation_token.cancel()
        if reply.err is not None:
            request_future.set_exception(
                RuntimeError(reply.err.payload.to_bytes().decode("utf-8", errors="replace"))
            )
            return
        if reply.ok is None:
            request_future.set_exception(RuntimeError("Service reply had no payload."))
            return
        try:
            response = self.service_type.Response.deserialize(reply.ok.payload.to_bytes())
        except Exception as exc:
            request_future.set_exception(exc)
            return
        request_future.set_result(response)

    @property
    def token_keyexpr(self) -> str:
        ses = afor.auto_session(self.session)
        return token_keyexpr(
            "SC",
            name=self.service_name,
            dds_type=self.dds_type,
            hash=self.hash,
            qos_profile=self.topic_info.qos,
            node_name=self.node_name,
            session=ses,
            domain_id=self.domain_id,
            namespace=self.namespace,
            _enclave=self._enclave,
            _node_id=self._node_id,
            _zenoh_id=self._zenoh_id,
            _entity_id=self._entity_id,
        )

    @property
    def service_keyexpr(self) -> str:
        return publisher_keyexpr(
            name=self.service_name,
            dds_type=self.dds_type,
            hash=self.hash,
            namespace="/",
            domain_id=self.domain_id,
        )
