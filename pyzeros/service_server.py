import asyncio
import time
import uuid
from typing import Coroutine, Generic, TypeVar

import asyncio_for_robotics.zenoh as afor
import numpy as np
import zenoh
from asyncio_for_robotics import BaseSub
from nptyping import NDArray, Shape, UInt8
from ros2_pyterfaces.cydr.idl import types

from .builtin_msgs import Attachment
from .pub import publisher_keyexpr
from .qos import QosProfile
from .service_common import ServiceType, qualify_service_name, token_keyexpr
from .utils import TopicInfo, resolve_liveliness_context, rmw_zenoh_gid, ros_type_to_dds_type

_ReqT = TypeVar("_ReqT")
_ResT = TypeVar("_ResT")


class Responder(Generic[_ReqT, _ResT]):
    """A pending service response bound to one incoming request.

    Instances are yielded by `Server.listen_*()` methods. The user reads
    `request`, optionally mutates `response`, and calls `send()` exactly once.
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
        """Send the response back to the waiting service client.

        Args:
            response: Optional response override. If omitted, `self.response`
                is serialized and sent.
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
    """RMW_ZENOH-compatible ROS service server.

    The server behaves like an async subscriber whose items are `Responder`
    objects. Each responder contains the request plus a mutable response object
    and exposes `send()` to publish the reply.
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
        """Create a service server bound to a Zenoh session and ROS node identity.

        Args:
            msg_type: Service type exposing `Request`, `Response`,
                `get_type_name()`, and `hash_rihs01()`.
            topic: Service name to serve.
            qos_profile: QoS profile advertised on the ROS graph token.
            session: Zenoh session used to declare the server.
            domain_id: ROS domain id. If omitted, `ROS_DOMAIN_ID` is used.
            namespace: ROS namespace used to qualify relative service names.
            node_name: ROS node name owning this server. A random name is used
                if omitted.
            defer: If `False`, declare the server immediately. If `True`,
                declaration is deferred until `declare()` or `async_bind()`.
            _service_hash: Internal type hash override.
            _enclave: Internal enclave segment used in the liveliness token.
            _node_id: Internal node id override for RMW_ZENOH compatibility.
            _zenoh_id: Internal Zenoh id override.
            _entity_id: Internal entity id override.
        """
        super().__init__()
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

        qos_profile = QosProfile.services() if qos_profile is None else qos_profile.normalized()
        self.service_type = msg_type
        self.service_name = qualify_service_name(topic, self.namespace, self.node_name)
        self.dds_type = ros_type_to_dds_type(msg_type.get_type_name())
        self.hash = _service_hash if _service_hash is not None else msg_type.hash_rihs01()
        self.topic_info: TopicInfo[type[ServiceType[_ReqT, _ResT]]] = TopicInfo(
            topic=self.service_name, msg_type=msg_type, qos=qos_profile
        )

        self.token: zenoh.LivelinessToken | None = None
        self.zenoh_srv: zenoh.Queryable | None = None
        self.gid: NDArray[Shape["16"], UInt8] = np.frombuffer(
            rmw_zenoh_gid(self.token_keyexpr), dtype=np.uint8, count=16
        )
        if not defer:
            self.declare()

    def async_bind(self) -> Coroutine[None, None, None]:
        """Bind this server lifetime to the current task.

        When this method is called, the server is declared immediately if
        needed. The returned coroutine keeps it alive until canceled, then
        closes it in `finally`.

        Returns:
            This coroutine never returns normally. It keeps the server declared
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
        """Declare the service server on Zenoh and on the ROS graph."""
        ses = afor.auto_session(self.session)
        self.token = ses.liveliness().declare_token(self.token_keyexpr)
        self.zenoh_srv = ses.declare_queryable(
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
            query.reply_err(str(exc).encode("utf-8"), encoding=zenoh.Encoding.TEXT_PLAIN)
            query.drop()
            return

        if not self.input_data(responder):
            query.drop()

    @property
    def token_keyexpr(self) -> str:
        """Return the liveliness token key expression for this service server."""
        ses = afor.auto_session(self.session)
        return token_keyexpr(
            "SS",
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
        """Return the Zenoh key expression used to receive service requests."""
        return publisher_keyexpr(
            name=self.service_name,
            dds_type=self.dds_type,
            hash=self.hash,
            namespace="/",
            domain_id=self.domain_id,
        )
