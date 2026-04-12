from __future__ import annotations

import uuid
from typing import TYPE_CHECKING, Any, Callable, Generic, TypeVar

import numpy as np
import zenoh
from asyncio_for_robotics import BaseSub, Scope
from nptyping import NDArray, Shape, UInt8
from ros2_pyterfaces.cydr.idl import types

from pyzeros.pub import publisher_keyexpr

from ._scope import ScopeOwned, _AUTO_SCOPE
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


def subscriber_keyexpr(
    name: str,
    dds_type: str,
    hash: str,
    namespace: str = "/",
    domain_id: int | str | None = None,
) -> str:
    """Generates a zenoh subscriber keyexpr associated with a ROS subscriber."""
    return publisher_keyexpr(
        name=name,
        dds_type=dds_type,
        hash=hash,
        namespace=namespace,
        domain_id=domain_id,
    )


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
    """Generates a zenoh liveliness token keyexpr associated with a ROS subscriber.

    This declares the subscriber on the ros graph, `ros2 topic list`.
    """
    qos_profile = (
        QosProfile.default() if qos_profile is None else qos_profile.normalized()
    )
    if node_name is None:
        node_name = f"naked_sub_{uuid.uuid4().hex[:8]}"
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
            "MS",
            _enclave,
            encoded_namespace,
            node_name,
            encoded_name,
            dds_type,
            hash,
            qos_profile.encode(),
        ]
    )


class RawSub(ScopeOwned, Generic[_MsgType]):
    """Low level RMW_ZENOH compatible subscriber.

    This class manages the ROS graph liveliness token and exposes raw
    `zenoh.Sample` values to the provided callback.
    """

    def __init__(
        self,
        msg_type: type[_MsgType] | None,
        topic: str,
        callback: Callable[[zenoh.Sample], Any],
        qos_profile: QosProfile | None = None,
        session: Node | None = None,
        defer: bool = False,
        *,
        scope: Scope | None | object = _AUTO_SCOPE,
    ):
        """RMW_ZENOH compatible subscriber.

        Args:
            msg_type: Message type implementing `get_type_name()` and
                `hash_rihs01()`.
            topic: ROS topic name.
            callback: Callback invoked with each raw `zenoh.Sample`.
            qos_profile: QoS profile associated with the subscription.
            session: Session node owning this subscriber. Resolved via
                ``auto_session`` when ``None``.
            defer: If `True`, declaration is deferred until `declare()`.
            scope: Optional afor lexical scope owning this subscriber.
        """
        self._init_scope(scope)
        node = auto_session(session)
        self.callback = callback
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
        self.zenoh_sub: zenoh.Subscriber | None = None
        if defer is False:
            self.declare()

        self.gid: NDArray[Shape["16"], UInt8] = np.frombuffer(
            rmw_zenoh_gid(self.token_keyexpr), dtype=np.uint8, count=16
        )

    @property
    def token_keyexpr(self):
        """The token keyexpr associated to this subscriber."""
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
    def subscriber_keyexpr(self):
        """The subscriber keyexpr associated to this subscriber."""
        return subscriber_keyexpr(
            name=self.topic_info.topic,
            dds_type=self.dds_type,
            hash=self.hash,
            namespace=self.namespace,
            domain_id=self.domain_id,
        )

    @property
    def fully_qualified_name(self) -> str:
        """Absolute topic name including the node's namespace."""
        return topic_join("/", self.namespace, self.topic_info.topic)

    def declare(self):
        """Declares the subscriber on Zenoh and ROS."""
        if self.token is not None:
            return
        self.token = self.session.liveliness().declare_token(self.token_keyexpr)
        self.zenoh_sub = self.session.declare_subscriber(
            self.subscriber_keyexpr,
            self.callback,
            **self.topic_info.qos.subscriber_options(),
        )

    def undeclare(self):
        """Undeclares the subscriber on Zenoh and ROS."""
        if self.token is not None:
            self.token.undeclare()
            self.token = None
        if self.zenoh_sub is not None:
            self.zenoh_sub.undeclare()
            self.zenoh_sub = None
        self._set_lifetime_result(True)

    def close(self) -> None:
        """Close this subscriber and undeclare it from Zenoh and the ROS graph."""
        self.undeclare()


class Sub(BaseSub[_MsgType]):
    """High level typed subscriber built on top of `RawSub`.

    This adapter consumes raw `zenoh.Sample` values from `RawSub`, deserializes
    them into `msg_type`, and exposes them through the `BaseSub` async
    interface.
    """

    def __init__(
        self,
        msg_type: type[_MsgType] | None,
        topic: str,
        qos_profile: QosProfile | None = None,
        session: Node | None = None,
        defer: bool = False,
        *,
        scope: Scope | None | object = _AUTO_SCOPE,
    ):
        """Creates a typed topic subscription.

        Args:
            msg_type: Message type used to deserialize incoming samples.
            topic: ROS topic name.
            qos_profile: QoS profile associated with the subscription.
            session: Session node owning this subscriber. Resolved via
                ``auto_session`` when ``None``.
            defer: If `False`, background consumption starts immediately.
            scope: Optional afor lexical scope owning this subscriber.
        """
        if scope is _AUTO_SCOPE:
            scope = Scope.current(default=None)
        super().__init__(scope=scope)
        self.sample_sub: BaseSub[zenoh.Sample] = BaseSub(scope=scope)
        self.raw_sub = RawSub(
            msg_type=msg_type,
            topic=topic,
            callback=self.sample_sub.input_data,
            qos_profile=qos_profile,
            session=session,
            defer=True,
            scope=scope,
        )
        self.topic_info = TopicInfo(topic, msg_type, self.raw_sub.topic_info.qos)
        func = lambda sample: self._input_data_asyncio(
            self.topic_info.msg_type.deserialize(sample.payload.to_bytes())
        )
        self.sample_sub.asap_callback.append(func)
        if not defer:
            self.raw_sub.declare()

    @property
    def fully_qualified_name(self) -> str:
        """Absolute topic name including the node's namespace."""
        return self.raw_sub.fully_qualified_name

    def close(self):
        """Stops background consumption and closes the subscriber."""
        self.raw_sub.close()
        self.sample_sub.close()
        return super().close()
