import asyncio
import time
import uuid
from typing import Any, Generic, Literal, Optional, TypeVar

import asyncio_for_robotics.zenoh as afor
import numpy as np
import zenoh
from nptyping import NDArray, Shape, UInt8
from ros2_pyterfaces.cydr.idl import types

from .builtin_msgs import Attachment
from .utils import (
    TopicInfo,
    mangle_liveliness_topic,
    resolve_liveliness_context,
    resolve_liveliness_identity,
    rmw_zenoh_gid,
    ros_type_to_dds_type,
)

_MsgType = TypeVar("_MsgType")


def publisher_keyexpr(
    name: str,
    dds_type: str,
    hash: str,
    namespace: str = "/",
    domain_id: int | str | None = None,
) -> str:
    """Generates the Zenoh key expression used to publish a ROS topic.

    The key expression encodes the ROS domain, effective topic path, DDS type,
    and topic hash so that `rmw_zenoh`-compatible subscribers can resolve the
    publisher.
    """
    if name[0] == "/" or namespace=="%":
        namespace="/"
    name = name.removeprefix("/").removesuffix("/")
    namespace = namespace.removeprefix("/").removesuffix("/")
    name = f"{namespace}/{name}"
    name = name.removeprefix("/").removesuffix("/")
    if domain_id is None:
        domain_id = os.environ.get("ROS_DOMAIN_ID", 0)
    return "/".join([str(domain_id), name, dds_type, hash])


def token_keyexpr(
    name: str,
    dds_type: str,
    hash: str,
    node_name: str | None = None,
    session: zenoh.Session | None = None,
    domain_id: int | str | None = None,
    namespace: str = "%",
    _enclave: str = "%",
    _node_id: str | int | None = None,
    _zenoh_id: str | None = None,
    _entity_id: int | str | None = None,
) -> str:
    """Generates a zenoh liveliness token keyexpr associated with a ROS publisher.

    This declares the publisher on the ROS graph, making it visible to ROS
    graph introspection tools.
    """
    if node_name is None:
        node_name = f"naked_pub_{uuid.uuid4().hex[:8]}"
    domain_id, _zenoh_id, _node_id, _entity_id = resolve_liveliness_identity(
        session=session,
        domain_id=domain_id,
        _node_id=_node_id,
        _zenoh_id=_zenoh_id,
        _entity_id=_entity_id,
    )
    encoded_namespace, encoded_name = mangle_liveliness_topic(name, namespace)
    return "/".join(
        [
            "@ros2_lv",
            str(domain_id),
            _zenoh_id,
            str(_node_id),
            str(_entity_id),
            "MP",
            _enclave,
            encoded_namespace,
            node_name,
            encoded_name,
            dds_type,
            hash,
            "::,10:,:,:,,",  # placeholder
        ]
    )


class Pub(Generic[_MsgType]):
    """RMW_ZENOH compatible ROS publisher.

    A `Pub` manages both the Zenoh publisher used to send serialized payloads
    and the liveliness token that advertises the publisher on the ROS graph.
    """

    def __init__(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: None = None,
        session: zenoh.Session | None = None,
        domain_id: int | str | None = None,
        namespace: str = "%",
        node_name: str | None = None,
        defer: bool = False,
        _topic_hash: str | None = None,
        _enclave: str = "%",
        _node_id: str | int | None = None,
        _zenoh_id: str | None = None,
        _entity_id: int | str | None = None,
    ):
        """Creates a ROS publisher backed by Zenoh.

        This should preferably not be instantiated manually, but rather using
        `node.create_publisher`.

        Args:
            msg_type: Message type implementing `get_type_name()`,
                `hash_rihs01()`, and `serialize()`.
            topic: ROS topic name.
            qos_profile: QoS profile associated with the publisher.
            session: Zenoh session used to declare the publisher.
            domain_id: ROS domain id. If omitted, `ROS_DOMAIN_ID` is used and
                defaults to `0`.
            namespace: ROS namespace advertised for the publisher.
            node_name: Name of the node the publisher belongs to.
            defer: If `False`, the publisher is declared immediately. If
                `True`, declaration is deferred until `declare()` or
                `async_bind()` is called.
            _topic_hash: Internal topic hash override. If omitted, it is
                derived from `msg_type`.
            _enclave: Internal enclave segment used when building the token
                keyexpr.
            _node_id: Internal id of the node the publisher belongs to.
            _zenoh_id: Internal Zenoh session id override.
            _entity_id: Internal entity id override for this publisher.
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
        self.dds_type = ros_type_to_dds_type(msg_type.get_type_name())
        self.hash = _topic_hash if _topic_hash is not None else msg_type.hash_rihs01()
        self.topic_info: TopicInfo[type[_MsgType]] = TopicInfo(
            topic=topic, msg_type=msg_type, qos=qos_profile
        )
        self.token: zenoh.LivelinessToken | None = None
        self.zenoh_pub: zenoh.Publisher | None = None
        if defer == False:
            self.declare()

        self.count = types.int64(0)
        self.gid: NDArray[Shape["16"], UInt8] = np.frombuffer(
            rmw_zenoh_gid(self.token_keyexpr), dtype=np.uint8, count=16
        )

    def publish(self, msg: _MsgType | bytes | memoryview | bytearray):
        """Publishes a message.

        If `msg` is a byte buffer, it is sent as-is without serialization.
        Otherwise `msg.serialize()` is used and an `rmw_zenoh` compatible
        attachment is added with sequence number, timestamp, and source GID.

        Args:
            msg: Message instance or already serialized payload to publish.
        """
        if self.zenoh_pub is None:
            raise ValueError("Publisher not declared.")
        self.zenoh_pub.put(
            msg if isinstance(msg, (bytes, memoryview, bytearray)) else msg.serialize(),
            attachment=Attachment(
                sequence_number=self.count,
                source_timestamp=types.int64(time.time_ns()),
                source_gid=self.gid,
            ).serialize()[4:],
        )
        self.count += 1

    async def async_bind(self):
        """Binds the publisher lifetime to an asyncio task.

        When this coroutine is canceled the publisher is undeclared. This is
        very useful for cleanup: using a `TaskGroup` the publisher can be
        restricted to live only inside the TaskGroup lifetime. In the event of
        a crash of the task group, the publisher will automatically be
        undeclared.

        If `defer=True`, then calling this coroutine also declares the
        publisher on Zenoh.

        Example:
            async with asyncio.TaskGroup() as tg:
                pub = Pub(...)
                tg.create_task(pub.async_bind())
        """
        try:
            if self.token is None:
                self.declare()
            await asyncio.Future()
        finally:
            self.undeclare()

    @property
    def token_keyexpr(self):
        """The token keyexpr associated to this publisher."""
        ses = afor.auto_session(self.session)
        return token_keyexpr(
            name=self.topic_info.topic,
            dds_type=self.dds_type,
            hash=self.hash,
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
    def publisher_keyexpr(self):
        """The publisher keyexpr associated to this publisher."""
        return publisher_keyexpr(
            name=self.topic_info.topic,
            dds_type=self.dds_type,
            hash=self.hash,
            namespace=self.namespace,
            domain_id=self.domain_id,
        )

    def declare(self):
        """Declares the publisher on Zenoh and ROS."""
        ses = afor.auto_session(self.session)
        self.token = ses.liveliness().declare_token(self.token_keyexpr)
        self.zenoh_pub = ses.declare_publisher(self.publisher_keyexpr)

    def undeclare(self):
        """Undeclares the publisher on Zenoh and ROS."""
        if self.token is not None:
            self.token.undeclare()
            self.token = None
        if self.zenoh_pub is not None:
            self.zenoh_pub.undeclare()
            self.zenoh_pub = None
