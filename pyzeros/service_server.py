from __future__ import annotations

import asyncio
import time
from typing import TYPE_CHECKING, Generic, TypeVar

import numpy as np
import zenoh
from asyncio_for_robotics import BaseSub, Scope
from nptyping import NDArray, Shape, UInt8
from ros2_pyterfaces.cydr.idl import types

from ._scope import _AUTO_SCOPE
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
_EvnT = TypeVar("_EvnT")

if TYPE_CHECKING:
    from .node import Node


class Responder(Generic[_ReqT, _ResT]):
    """One pending service request waiting for a reply.

    Yielded by ``Server.listen_reliable()`` (or ``listen()``).  Read
    ``responder.request``, fill ``responder.response``, call
    ``responder.send()``.

    Example::

        async for responder in server.listen_reliable():
            result = responder.request.a + responder.request.b
            responder.response.sum = result
            responder.send()
    """

    def __init__(
        self,
        request: _ReqT,
        response: _ResT,
        query: zenoh.Query,
        service_keyexpr_value: str,
        request_attachment: Attachment,
    ):
        """Create a responder for one incoming Zenoh service query.

        Args:
            request: Decoded request message received from the client.
            response: Fresh response message instance to be filled by the user.
            query: Underlying Zenoh query to reply to.
            service_keyexpr_value: Zenoh key expression to use for the reply.
            request_attachment: Attachment received with the request.
        """
        self.request = request
        self.response = response
        self._query = query
        self._service_keyexpr = service_keyexpr_value
        self._request_attachment = request_attachment
        self._sent = False

    def send(self, response: _ResT | None = None) -> None:
        """Send the response back to the client.  Must be called exactly once.

        Args:
            response: Override response to send.  If ``None``, ``self.response``
                is used.

        Raises:
            RuntimeError: If called a second time on the same responder.
        """
        if self._sent:
            raise RuntimeError("This responder has already sent a reply.")
        if response is None:
            response = self.response
        reply_attachment = Attachment(
            sequence_number=self._request_attachment.sequence_number,
            source_timestamp=types.int64(time.time_ns()),
            source_gid=self._request_attachment.source_gid,
        ).serialize()[4:]
        self._query.reply(
            self._service_keyexpr,
            response.serialize(),
            encoding=zenoh.Encoding.APPLICATION_CDR,
            attachment=reply_attachment,
        )
        self._query.drop()
        self._sent = True


class Server(BaseSub[Responder[_ReqT, _ResT]]):
    """ROS service server backed by a Zenoh queryable.

    Extends ``BaseSub`` — incoming requests arrive as ``Responder`` objects
    through ``listen_reliable()`` or ``listen()``.  Each ``Responder``
    carries the decoded request, a mutable response, and ``send()`` to
    reply.

    Example::

        server = pyzeros.Server(AddTwoInts, "add_two_ints")
        async for r in server.listen_reliable():
            r.response.sum = r.request.a + r.request.b
            r.send()
    """

    def __init__(
        self,
        msg_type: ServiceType[_ReqT, _ResT, _EvnT],
        topic: str,
        qos_profile: QosProfile | None = None,
        session: Node | None = None,
        defer: bool = False,
        *,
        scope: Scope | None | object = _AUTO_SCOPE,
    ):
        """Create a service server bound to a session node.

        Args:
            msg_type: Service type exposing `Request`, `Response`,
                `get_type_name()`, and `hash_rihs01()`.
            topic: Service name to serve.
            qos_profile: QoS profile advertised on the ROS graph token.
            session: Session node owning this server. Resolved via
                ``auto_session`` when ``None``.
            defer: If `False`, declare the server immediately.
            scope: Optional afor lexical scope owning this server.
        """
        if scope is _AUTO_SCOPE:
            scope = Scope.current(default=None)
        super().__init__(scope=scope)
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
        self.topic_info: TopicInfo[ServiceType[_ReqT, _ResT, _EvnT]] = TopicInfo(
            topic=self.service_name, msg_type=msg_type, qos=qos_profile
        )

        self.token: zenoh.LivelinessToken | None = None
        self.zenoh_srv: zenoh.Queryable | None = None
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
        """Declare the service server on Zenoh and on the ROS graph."""
        if self.token is not None:
            return
        self.token = self.session.liveliness().declare_token(self.token_keyexpr)
        self.zenoh_srv = self.session.declare_queryable(
            self.service_keyexpr,
            self._handle_query,
            complete=True,
        )

    def undeclare(self) -> None:
        """Undeclare the service server and its graph token."""
        if self.token is not None:
            self.token.undeclare()
            self.token = None
        if self.zenoh_srv is not None:
            self.zenoh_srv.undeclare()
            self.zenoh_srv = None

    def close(self) -> None:
        """Close this server, stop listeners, and undeclare it."""
        super().close()
        self.undeclare()

    def _handle_query(self, query: zenoh.Query) -> None:
        """Convert one Zenoh query into a `Responder` and enqueue it.

        Args:
            query: Raw Zenoh query carrying the serialized request.
        """
        try:
            if query.payload is None:
                raise ValueError("Received service request without payload.")
            if query.attachment is None:
                raise ValueError("Received service request without attachment.")
            request = self.service_type.Request.deserialize(query.payload.to_bytes())
            request_attachment = Attachment.deserialize(
                b"\x00\x01\x00\x00" + query.attachment.to_bytes()
            )
            responder = Responder(
                request=request,
                response=self.service_type.Response(),
                query=query,
                service_keyexpr_value=self.service_keyexpr,
                request_attachment=request_attachment,
            )
        except Exception as exc:
            query.reply_err(
                str(exc).encode("utf-8"), encoding=zenoh.Encoding.TEXT_PLAIN
            )
            query.drop()
            return

        if not self.input_data(responder):
            query.drop()

    @property
    def token_keyexpr(self) -> str:
        """Return the liveliness token key expression for this service server."""
        return token_keyexpr(
            "SS",
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
        """Return the Zenoh key expression used to receive service requests."""
        return publisher_keyexpr(
            name=self.service_name,
            dds_type=self.dds_type,
            hash=self.hash,
            namespace="/",
            domain_id=self.domain_id,
        )
