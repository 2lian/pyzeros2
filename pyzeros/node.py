import asyncio
import uuid
from typing import TypeVar

import asyncio_for_robotics.zenoh as afor
import zenoh

from pyzeros.pub import Pub
from pyzeros.sub import Sub
from pyzeros.utils import (
    QosProfile,
    normalize_namespace,
    resolve_liveliness_context,
    resolve_liveliness_identity,
)

_MsgType = TypeVar("_MsgType")


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
    """RMW_ZENOH compatible ROS node.

    This should preferably not be instantiated manually when a higher level
    factory is available, but it can be used directly to manage the node
    liveliness token and create publishers bound to the same node identity.
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
                declaration is deferred until `declare()` or `async_bind()`.
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

    async def async_bind(self):
        """Binds the node lifetime to an asyncio task.

        When this coroutine is canceled the node is undeclared. This is useful
        for cleanup when using a `TaskGroup`, ensuring the node only stays
        declared for the lifetime of the task.

        If `defer=True`, calling this coroutine also declares the node on
        Zenoh.

        Example:
            async with asyncio.TaskGroup() as tg:
                node = Node(...)
                tg.create_task(node.async_bind())
        """
        try:
            if self.token is None:
                self.declare()
            await asyncio.Future()
        finally:
            self.undeclare()

    def create_subscriber(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: QosProfile = None,
        defer: bool = False,
    ) -> Sub[_MsgType]:
        """Creates a subscriber attached to this node.

        Args:
            msg_type: ROS message type published on the topic.
            topic: ROS topic name.
            qos_profile: QoS profile associated with the subscriber.
            defer: If `False`, the subscriber is declared immediately. If
                `True`, declaration is deferred until `declare()` or
                `async_bind()` is called on the subscriber.

        Returns:
            A `Sub` instance sharing this node identity and Zenoh session.
        """
        return Sub(
            msg_type=msg_type,
            topic=topic,
            qos_profile=qos_profile,
            session=self.session,
            domain_id=self.domain_id,
            namespace=self.namespace,
            node_name=self.name,
            defer=defer,
            _enclave=self._enclave,
            _node_id=self._node_id,
        )

    def create_publisher(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: QosProfile = None,
        defer: bool = False,
    ):
        """Creates a publisher attached to this node.

        Args:
            msg_type: ROS message type published on the topic.
            topic: ROS topic name.
            qos_profile: QoS profile associated with the publisher.
            defer: If `False`, the publisher is declared immediately. If
                `True`, declaration is deferred until `declare()` or
                `async_bind()` is called on the publisher.

        Returns:
            A `Pub` instance sharing this node identity and Zenoh session.
        """
        return Pub(
            msg_type,
            topic,
            qos_profile,
            session=self.session,
            domain_id=self.domain_id,
            namespace=self.namespace,
            node_name=self.name,
            defer=defer,
            _enclave=self._enclave,
            _node_id=self._node_id,
        )

    def declare(self):
        """Declares the node on Zenoh and ROS."""
        ses = afor.auto_session(self.session)
        self.token = ses.liveliness().declare_token(self.token_keyexpr)

    def undeclare(self):
        """Undeclares the node on Zenoh and ROS."""
        if self.token is None:
            return
        self.token.undeclare()
        self.token = None

    @property
    def token_keyexpr(self) -> str:
        """The token keyexpr associated to this node."""
        ses = afor.auto_session(self.session)
        return token_keyexpr(
            self.name,
            ses,
            self.domain_id,
            self.namespace,
            self._enclave,
            self._node_id,
            self._zenoh_id,
            self._entity_id,
        )
