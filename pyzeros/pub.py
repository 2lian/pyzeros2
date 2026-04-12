from __future__ import annotations

import os
import time
import uuid
from typing import TYPE_CHECKING, Generic, TypeVar

import numpy as np
import zenoh
from asyncio_for_robotics import Scope
from nptyping import NDArray, Shape, UInt8
from ros2_pyterfaces.cydr.idl import types

from ._scope import ScopeOwned, _AUTO_SCOPE
from .builtin_msgs import Attachment
from .qos import QosProfile
from .session import auto_session
from .utils import (
    TopicInfo,
    mangle_liveliness_topic,
    resolve_liveliness_context,
    resolve_liveliness_identity,
    rmw_zenoh_gid,
    ros_type_to_dds_type,
    topic_join,
)

_MsgType = TypeVar("_MsgType")

if TYPE_CHECKING:
    from .node import Node


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
    if name[0] == "/" or namespace == "%":
        namespace = "/"
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
    qos_profile: QosProfile | None = None,
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
    qos_profile = (
        QosProfile.default() if qos_profile is None else qos_profile.normalized()
    )
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
            qos_profile.encode(),
        ]
    )


class Pub(ScopeOwned, Generic[_MsgType]):
    """RMW_ZENOH compatible ROS publisher.

    A `Pub` manages both the Zenoh publisher used to send serialized payloads
    and the liveliness token that advertises the publisher on the ROS graph.
    """

    def __init__(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: QosProfile | None = None,
        session: Node | None = None,
        defer: bool = False,
        *,
        scope: Scope | None | object = _AUTO_SCOPE,
    ):
        """Creates a ROS publisher backed by Zenoh.

        Args:
            msg_type: Message type implementing `get_type_name()`,
                `hash_rihs01()`, and `serialize()`.
            topic: ROS topic name.
            qos_profile: QoS profile associated with the publisher.
            session: Session node owning this publisher. Resolved via
                ``auto_session`` when ``None``.
            defer: If `False`, the publisher is declared immediately.
            scope: Optional afor lexical scope owning this publisher.
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
        self.dds_type = ros_type_to_dds_type(msg_type.get_type_name())
        self.hash = msg_type.hash_rihs01()
        qos_profile = (
            QosProfile.default() if qos_profile is None else qos_profile.normalized()
        )
        self.topic_info: TopicInfo[type[_MsgType]] = TopicInfo(
            topic=topic, msg_type=msg_type, qos=qos_profile
        )
        self.token: zenoh.LivelinessToken | None = None
        self.zenoh_pub: zenoh.Publisher | None = None
        if defer is False:
            self.declare()

        self.count = types.int64(0)
        self.gid: NDArray[Shape["16"], UInt8] = np.frombuffer(
            rmw_zenoh_gid(self.token_keyexpr), dtype=np.uint8, count=16
        )

    @property
    def fully_qualified_name(self) -> str:
        """Absolute topic name including the node's namespace."""
        return topic_join("/", self.namespace, self.topic_info.topic)

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

    @property
    def token_keyexpr(self):
        """The token keyexpr associated to this publisher."""
        return token_keyexpr(
            name=self.topic_info.topic,
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
        if self.token is not None:
            return
        self.token = self.session.liveliness().declare_token(self.token_keyexpr)
        self.zenoh_pub = self.session.declare_publisher(
            self.publisher_keyexpr,
            encoding=zenoh.Encoding.APPLICATION_CDR,
            **self.topic_info.qos.publisher_options(),
        )

    def undeclare(self):
        """Undeclares the publisher on Zenoh and ROS."""
        if self.token is not None:
            self.token.undeclare()
            self.token = None
        if self.zenoh_pub is not None:
            self.zenoh_pub.undeclare()
            self.zenoh_pub = None
        self._set_lifetime_result(True)

    def close(self) -> None:
        """Close this publisher and undeclare it from Zenoh and the ROS graph."""
        self.undeclare()
