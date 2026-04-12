from __future__ import annotations

import asyncio
import time
from typing import TYPE_CHECKING, Coroutine, Generic, TypeVar

import numpy as np
import zenoh
from asyncio_for_robotics import Scope
from nptyping import NDArray, Shape, UInt8
from ros2_pyterfaces.cydr.idl import types

from ._scope import ScopeOwned, _AUTO_SCOPE
from .builtin_msgs import Attachment
from .pub import publisher_keyexpr
from .qos import QosProfile
from .session import auto_session
from .service_common import ServiceType, qualify_service_name, token_keyexpr
from .utils import (
    TopicInfo,
    resolve_liveliness_context,
    rmw_zenoh_gid,
    ros_type_to_dds_type,
)

_ReqT = TypeVar("_ReqT")
_ResT = TypeVar("_ResT")

if TYPE_CHECKING:
    from .node import Node


class Client(ScopeOwned, Generic[_ReqT, _ResT]):
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
        session: Node | None = None,
        defer: bool = False,
        *,
        scope: Scope | None | object = _AUTO_SCOPE,
    ):
        """Create a service client bound to a session node.

        Args:
            msg_type: Service type exposing `Request`, `Response`,
                `get_type_name()`, and `hash_rihs01()`.
            topic: Service name to call.
            qos_profile: QoS profile advertised on the ROS graph token.
            session: Session node owning this client. Resolved via
                ``auto_session`` when ``None``.
            defer: If `False`, declare the client immediately.
            scope: Optional afor lexical scope owning this client.
        """
        self._init_scope(scope)
        node = auto_session(session)
        ctx = resolve_liveliness_context(
            session=node.session,
            domain_id=node.domain_id,
            namespace=node.namespace,
            _enclave=node._enclave,
            _node_id=node._node_id,
        )
        self.session = ctx.session
        self.namespace = ctx.namespace
        self._enclave = ctx.enclave
        self.domain_id = ctx.domain_id
        self._zenoh_id = ctx.zenoh_id
        self.node_name = node.name
        self._node_id = ctx.node_id
        self._entity_id = ctx.entity_id

        qos_profile = (
            QosProfile.services() if qos_profile is None else qos_profile.normalized()
        )
        self.service_type = msg_type
        self.service_name = qualify_service_name(topic, self.namespace, self.node_name)
        self.dds_type = ros_type_to_dds_type(msg_type.get_type_name())
        self.hash = msg_type.hash_rihs01()
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

    @property
    def fully_qualified_name(self) -> str:
        """Absolute service name including the node's namespace."""
        return self.service_name

    def declare(self) -> None:
        """Declare the service client on Zenoh and on the ROS graph."""
        if self.token is not None:
            return
        self.token = self.session.liveliness().declare_token(self.token_keyexpr)
        self.zenoh_cli = self.session.declare_querier(
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
        self._set_lifetime_result(True)

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

    def _request(
        self, req: _ReqT
    ) -> tuple[asyncio.Future[_ResT], zenoh.CancellationToken]:
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
        return token_keyexpr(
            "SC",
            name=self.service_name,
            dds_type=self.dds_type,
            hash=self.hash,
            qos_profile=self.topic_info.qos,
            node_name=self.node_name,
            session=self.session,
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
