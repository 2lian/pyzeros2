import asyncio
import time
import uuid
from typing import Coroutine, Generic, TypeVar

import asyncio_for_robotics.zenoh as afor
import numpy as np
import zenoh
from nptyping import NDArray, Shape, UInt8
from ros2_pyterfaces.cydr.idl import types

from .builtin_msgs import Attachment
from .pub import publisher_keyexpr
from .qos import QosProfile
from .service_common import ServiceType, qualify_service_name, token_keyexpr
from .utils import (
    TopicInfo,
    resolve_liveliness_context,
    rmw_zenoh_gid,
    ros_type_to_dds_type,
)

_ReqT = TypeVar("_ReqT")
_ResT = TypeVar("_ResT")


class Client(Generic[_ReqT, _ResT]):
    """RMW_ZENOH-compatible ROS service client.

    This client declares a ROS service client on the graph and sends requests
    as Zenoh queries. Requests are started immediately when `call_async()` is
    invoked, while awaiting and cancellation remain under caller control.
    """

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
        _service_hash: str | None = None,
        _enclave: str = "%",
        _node_id: str | int | None = None,
        _zenoh_id: str | None = None,
        _entity_id: int | str | None = None,
    ):
        """Create a service client bound to a Zenoh session and ROS node identity.

        Args:
            msg_type: Service type exposing `Request`, `Response`,
                `get_type_name()`, and `hash_rihs01()`.
            topic: Service name to call.
            qos_profile: QoS profile advertised on the ROS graph token.
            session: Zenoh session used to declare the client.
            domain_id: ROS domain id. If omitted, `ROS_DOMAIN_ID` is used.
            namespace: ROS namespace used to qualify relative service names.
            node_name: ROS node name owning this client. A random name is used
                if omitted.
            defer: If `False`, declare the client immediately. If `True`,
                declaration is deferred until `declare()` or `async_bind()`.
            _service_hash: Internal type hash override.
            _enclave: Internal enclave segment used in the liveliness token.
            _node_id: Internal node id override for RMW_ZENOH compatibility.
            _zenoh_id: Internal Zenoh id override.
            _entity_id: Internal entity id override.
        """
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

        qos_profile = (
            QosProfile.services() if qos_profile is None else qos_profile.normalized()
        )
        self.service_type = msg_type
        self.service_name = qualify_service_name(topic, self.namespace, self.node_name)
        self.dds_type = ros_type_to_dds_type(msg_type.get_type_name())
        self.hash = (
            _service_hash if _service_hash is not None else msg_type.hash_rihs01()
        )
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

    def async_bind(self) -> Coroutine[None, None, None]:
        """Bind this client lifetime to the current task.

        When this method is called, the client is declared immediately if
        needed. The returned coroutine keeps it alive until canceled, then
        closes it in `finally`.

        Returns:
            This coroutine never returns normally. It keeps the client declared
            until the surrounding task is canceled, then closes it in `finally`.
        """
        if self.token is None:
            self.declare()

        async def bind() -> None:
            try:
                await asyncio.Future()
            finally:
                self.close()

        return bind()

    def declare(self) -> None:
        """Declare the service client on Zenoh and on the ROS graph."""
        ses = afor.auto_session(self.session)
        self.token = ses.liveliness().declare_token(self.token_keyexpr)
        self.zenoh_cli = ses.declare_querier(
            self.service_keyexpr,
            target=zenoh.QueryTarget.ALL_COMPLETE,
            consolidation=zenoh.ConsolidationMode.NONE,
        )

    def undeclare(self) -> None:
        """Undeclare the service client and cancel pending requests."""
        for future in self._pending_requests:
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
        """Close this client and undeclare it from Zenoh and the ROS graph."""
        self.undeclare()

    async def wait_for_service(self) -> None:
        """Wait until at least one matching service server is visible.

        Returns:
            As soon as a matching service server is visible to the querier.
        """
        if self.zenoh_cli is None:
            raise ValueError("Client not declared.")
        if self.zenoh_cli.matching_status.matching:
            return

        event_loop = asyncio.get_running_loop()
        service_ready = asyncio.Event()

        def on_matching(status: zenoh.MatchingStatus) -> None:
            if status.matching:
                event_loop.call_soon_threadsafe(service_ready.set)

        listener = self.zenoh_cli.declare_matching_listener(on_matching)
        try:
            if self.zenoh_cli.matching_status.matching:
                return
            await service_ready.wait()
        finally:
            listener.undeclare()

    def _request(self, req: _ReqT) -> tuple[asyncio.Future[_ResT], zenoh.CancellationToken]:
        """Start a service request immediately and return its tracking objects.

        Args:
            req: Request message to serialize and send.

        Returns:
            A tuple of `(request_future, cancellation_token)` where
            `request_future` completes with the decoded response and
            `cancellation_token` can abort the underlying Zenoh query.
        """
        if self.zenoh_cli is None:
            raise ValueError("Client not declared.")

        event_loop = asyncio.get_running_loop()
        request_future: asyncio.Future[_ResT] = event_loop.create_future()
        self._pending_requests.add(request_future)
        cancellation_token = zenoh.CancellationToken()

        def handle_reply(reply: zenoh.Reply) -> None:
            event_loop.call_soon_threadsafe(
                self._resolve_reply,
                request_future,
                reply,
            )

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
        return request_future, cancellation_token

    async def _wait_for_request(
        self,
        request_future: asyncio.Future[_ResT],
        cancellation_token: zenoh.CancellationToken,
    ) -> _ResT:
        """Await a request future and ensure transport cleanup on exit.

        Args:
            request_future: Future that will be completed by the reply handler.
            cancellation_token: Token used to cancel the underlying Zenoh query.

        Returns:
            The decoded service response.
        """
        try:
            return await request_future
        finally:
            self._pending_requests.discard(request_future)
            cancellation_token.cancel()
            if not request_future.done():
                request_future.cancel()

    def call_async(self, req: _ReqT) -> Coroutine[None, None, _ResT]:
        """Start a service request and return an awaitable for its response.

        The request is registered and sent immediately when this method is
        called. The returned coroutine may be awaited later or scheduled by the
        caller in a task or task group.

        Args:
            req: Request message to send.

        Returns:
            A coroutine that resolves to the decoded service response.
        """
        request_future, cancellation_token = self._request(req)
        return self._wait_for_request(request_future, cancellation_token)

    def _resolve_reply(
        self,
        request_future: asyncio.Future[_ResT],
        reply: zenoh.Reply,
    ) -> None:
        """Decode one reply sample into the pending request future.

        Args:
            request_future: Future to resolve with the decoded response.
            reply: Raw Zenoh reply received for the request.
        """
        if request_future.done():
            return
        if reply.err is not None:
            request_future.set_exception(
                RuntimeError(
                    reply.err.payload.to_bytes().decode("utf-8", errors="replace")
                )
            )
            return
        if reply.ok is None:
            request_future.set_exception(RuntimeError("Service reply had no payload."))
            return
        try:
            response = self.service_type.Response.deserialize(
                reply.ok.payload.to_bytes()
            )
        except Exception as exc:
            request_future.set_exception(exc)
            return
        request_future.set_result(response)

    @property
    def token_keyexpr(self) -> str:
        """Return the liveliness token key expression for this service client."""
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
        """Return the Zenoh key expression used to send service requests."""
        return publisher_keyexpr(
            name=self.service_name,
            dds_type=self.dds_type,
            hash=self.hash,
            namespace="/",
            domain_id=self.domain_id,
        )
