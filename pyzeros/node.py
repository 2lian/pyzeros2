import uuid
from typing import TypeVar
import warnings

import zenoh
from asyncio_for_robotics import Scope

from pyzeros.pub import Pub
from pyzeros.qos import QosProfile
from pyzeros._scope import _AUTO_SCOPE
from pyzeros.service_common import ServiceType
from pyzeros.service_client import Client
from pyzeros.service_server import Server
from pyzeros.sub import Sub
from pyzeros.utils import (
    normalize_namespace,
    resolve_liveliness_context,
    resolve_liveliness_identity,
    topic_join,
)

_MsgType = TypeVar("_MsgType")
_ReqT = TypeVar("_ReqT")
_ResT = TypeVar("_ResT")
_EvnT = TypeVar("_EvnT")


def token_keyexpr(
    name: str | None = None,
    session: zenoh.Session | None = None,
    domain_id: int | str | None = None,
    namespace: str = "%",
    _enclave: str = "%",
    _node_id: str | int | None = None,
    _zenoh_id: str | None = None,
    _entity_id: int | str | None = None,
) -> str:
    """Generates a zenoh liveliness token keyexpr associated with a ROS node.

    This declares the node on the ROS graph, making it visible to ROS graph
    introspection tools.
    """
    if name is None:
        name = f"ros_ez_{uuid.uuid4().hex[:8]}"
    namespace = normalize_namespace(namespace)
    domain_id, _zenoh_id, _node_id, _entity_id = resolve_liveliness_identity(
        session=session,
        domain_id=domain_id,
        _node_id=_node_id,
        _zenoh_id=_zenoh_id,
        _entity_id=_entity_id,
        node_id_from_entity=True,
    )
    return "/".join(
        [
            "@ros2_lv",
            str(domain_id),
            _zenoh_id,
            str(_node_id),
            str(_entity_id),
            "NN",
            _enclave,
            namespace.replace("/", "%"),
            name,
        ]
    )


class Node:
    """RMW_ZENOH-compatible ROS node identity.

    A ``Node`` owns a Zenoh liveliness token that makes it visible to
    ``ros2 node list`` and other ROS graph introspection tools.  It also
    serves as the factory for ``Pub``, ``Sub``, ``Client``, and ``Server``
    instances that share the same node identity.

    Prefer ``Session()`` + ``session_context()`` over constructing a ``Node``
    directly — the session layer handles Zenoh transport and cleanup for you.
    """

    def __init__(
        self,
        name: str | None = None,
        session: zenoh.Session | None = None,
        domain_id: int | str | None = None,
        namespace: str = "%",
        defer: bool = False,
        _enclave: str = "%",
        _node_id: str | int | None = None,
        _zenoh_id: str | None = None,
        _entity_id: int | str | None = None,
    ):
        """Creates a ROS node backed by a Zenoh liveliness token.

        Args:
            name: Node name exposed on the ROS graph. If omitted, a random
                `ros_ez_*` name is generated.
            session: Zenoh session used to declare the node.
            domain_id: ROS domain id. If omitted, `ROS_DOMAIN_ID` is used and
                defaults to `0`.
            namespace: ROS namespace advertised for the node.
            defer: If `False`, the node is declared immediately. If `True`,
                declaration is deferred until `declare()`.
            _enclave: Internal enclave segment used when building the token
                keyexpr.
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
            node_id_from_entity=True,
        )
        self.session = ctx.session
        self.namespace = ctx.namespace
        self._enclave = ctx.enclave
        self.domain_id = ctx.domain_id
        self._zenoh_id = ctx.zenoh_id
        self.name = f"ros_ez_{uuid.uuid4().hex[:8]}" if name is None else name
        self._node_id = ctx.node_id
        self._entity_id = ctx.entity_id

        self.token: zenoh.LivelinessToken | None = None
        if not defer:
            self.declare()

    def create_subscriber(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: QosProfile | None = None,
        defer: bool = False,
        *,
        scope: Scope | None | object = _AUTO_SCOPE,
    ) -> Sub[_MsgType]:
        """Creates a subscriber attached to this node.

        Args:
            msg_type: ROS message type published on the topic.
            topic: ROS topic name.
            qos_profile: QoS profile associated with the subscriber.
            defer: If `False`, the subscriber is declared immediately. If
                `True`, declaration is deferred until `declare()` is called.
            scope: Optional afor lexical scope owning the subscriber.

        Returns:
            A `Sub` instance sharing this node identity and Zenoh session.
        """
        warnings.warn("Precated, prefere passing the node at instantiation")
        return Sub(
            msg_type=msg_type,
            topic=topic,
            qos_profile=qos_profile,
            session=self,
            defer=defer,
            scope=scope,
        )

    def create_publisher(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: QosProfile | None = None,
        defer: bool = False,
        *,
        scope: Scope | None | object = _AUTO_SCOPE,
    ):
        """Creates a publisher attached to this node.

        Args:
            msg_type: ROS message type published on the topic.
            topic: ROS topic name.
            qos_profile: QoS profile associated with the publisher.
            defer: If `False`, the publisher is declared immediately. If
                `True`, declaration is deferred until `declare()` is called.
            scope: Optional afor lexical scope owning the publisher.

        Returns:
            A `Pub` instance sharing this node identity and Zenoh session.
        """
        warnings.warn("Precated, prefere passing the node at instantiation")
        return Pub(
            msg_type,
            topic,
            qos_profile,
            session=self,
            defer=defer,
            scope=scope,
        )

    def create_client(
        self,
        msg_type: ServiceType[_ReqT, _ResT, _EvnT],
        topic: str,
        qos_profile: QosProfile | None = None,
        defer: bool = False,
        *,
        scope: Scope | None | object = _AUTO_SCOPE,
    ) -> Client[_ReqT, _ResT, _EvnT]:
        """Creates a service client attached to this node.

        Args:
            msg_type: Service type (e.g. ``AddTwoInts``).
            topic: Service name.
            qos_profile: QoS profile for the client token.
            defer: If ``True``, declaration is deferred until ``declare()``.
            scope: Optional afor scope owning this client.
        """
        warnings.warn("Precated, prefere passing the node at instantiation")
        return Client(
            msg_type=msg_type,
            topic=topic,
            qos_profile=qos_profile,
            session=self,
            defer=defer,
            scope=scope,
        )

    def create_service(
        self,
        msg_type: ServiceType[_ReqT, _ResT, _EvnT],
        topic: str,
        qos_profile: QosProfile | None = None,
        defer: bool = False,
        *,
        scope: Scope | None | object = _AUTO_SCOPE,
    ) -> Server[_ReqT, _ResT]:
        """Creates a service server attached to this node.

        Args:
            msg_type: Service type (e.g. ``Trigger``).
            topic: Service name.
            qos_profile: QoS profile for the server token.
            defer: If ``True``, declaration is deferred until ``declare()``.
            scope: Optional afor scope owning this server.
        """
        warnings.warn("Precated, prefere passing the node at instantiation")
        return Server(
            msg_type=msg_type,
            topic=topic,
            qos_profile=qos_profile,
            session=self,
            defer=defer,
            scope=scope,
        )

    @property
    def fully_qualified_name(self) -> str:
        """Absolute node name including the namespace."""
        return topic_join("/", self.namespace, self.name)

    def declare(self):
        """Declare the node's liveliness token, making it visible on the ROS graph.

        No-op if already declared.  Called automatically unless ``defer=True``.
        """
        if self.token is not None:
            return
        self.token = self.session.liveliness().declare_token(self.token_keyexpr)

    def undeclare(self):
        """Remove the node's liveliness token from the ROS graph.

        Idempotent — safe to call multiple times.
        """
        if self.token is None:
            return
        self.token.undeclare()
        self.token = None

    def close(self) -> None:
        """Close this node from the ROS graph."""
        self.undeclare()

    @property
    def zenoh_session(self) -> zenoh.Session:
        """Alias for the underlying Zenoh session."""
        return self.session

    @property
    def token_keyexpr(self) -> str:
        """The token keyexpr associated to this node."""
        return token_keyexpr(
            self.name,
            self.session,
            self.domain_id,
            self.namespace,
            self._enclave,
            self._node_id,
            self._zenoh_id,
            self._entity_id,
        )
