import asyncio
import time
import uuid
from typing import Any, Callable, Coroutine, Generic, TypeVar

import asyncio_for_robotics.zenoh as afor
import numpy as np
import zenoh
from asyncio_for_robotics import BaseSub
from nptyping import NDArray, Shape, UInt8
from ros2_pyterfaces.cydr.idl import types

from pyzeros.pub import publisher_keyexpr

from .builtin_msgs import Attachment
from .qos import QosProfile
from .utils import (
    TopicInfo,
    mangle_liveliness_topic,
    resolve_liveliness_context,
    resolve_liveliness_identity,
    rmw_zenoh_gid,
    ros_type_to_dds_type,
)

_MsgType = TypeVar("_MsgType")


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
    qos_profile = QosProfile.default() if qos_profile is None else qos_profile.normalized()
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


class RawSub(Generic[_MsgType]):
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
        session: zenoh.Session | None = None,
        domain_id: int | str | None = None,
        namespace: str = "%",
        node_name: str | None = None,
        defer: bool = False,
        _hash: str | None = None,
        _dds_type: str | None = None,
        _enclave: str = "%",
        _node_id: str | int | None = None,
        _zenoh_id: str | None = None,
        _entity_id: int | str | None = None,
    ):
        """RMW_ZENOH compatible subscriber.

        This should preferably not be instantiated manually, but rather using
        node.create_subscriber.

        Args:
            msg_type: Message type implementing `get_type_name()` and
                `hash_rihs01()`.
            topic: ROS topic name.
            callback: Callback invoked with each raw `zenoh.Sample`.
            qos_profile: QoS profile associated with the subscription.
            session: Zenoh session used to declare the subscriber.
            domain_id: ROS domain id. If omitted, `ROS_DOMAIN_ID` is used and
                defaults to `0`.
            namespace: ROS namespace advertised for the subscriber.
            node_name: Name of the node the subscriber belongs to.
            defer: If `True`, declaration is deferred until `declare()` or
                `async_bind()` is called.
            _hash: Internal topic hash override. If omitted, it is derived from
                `msg_type`.
            _dds_type: Internal DDS type override. If omitted, it is derived
                from `msg_type`.
            _enclave: Internal enclave segment used when building the token
                keyexpr.
            _node_id: Internal id of the node the subscriber belongs to.
            _zenoh_id: Internal Zenoh session id override.
            _entity_id: Internal entity id override for this subscriber.
        """
        self.callback = callback
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
        self.dds_type = (
            _dds_type
            if _dds_type is not None
            else ros_type_to_dds_type(msg_type.get_type_name())
        )
        self.hash = _hash if _hash is not None else msg_type.hash_rihs01()
        qos_profile = QosProfile.default() if qos_profile is None else qos_profile.normalized()
        self.topic_info: TopicInfo[type[_MsgType]] = TopicInfo(
            topic=topic, msg_type=msg_type, qos=qos_profile
        )
        self.token: zenoh.LivelinessToken | None = None
        self.zenoh_sub: zenoh.Subscriber | None = None
        if defer == False:
            self.declare()

        self.gid: NDArray[Shape["16"], UInt8] = np.frombuffer(
            rmw_zenoh_gid(self.token_keyexpr), dtype=np.uint8, count=16
        )

    def async_bind(self) -> Coroutine[None, None, None]:
        """Binds the subscriber lifetime to an asyncio task.

        When this method is called, the subscriber is declared immediately if
        needed. The returned coroutine keeps the subscriber alive until it is
        canceled, then undeclares it in `finally`.

        Returns:
            A coroutine that never returns normally and undeclares the
            subscriber when canceled.

        Example:
            async with asyncio.TaskGroup() as tg:
                sub = RawSub(...)
                tg.create_task(sub.async_bind())
        """
        if self.token is None:
            self.declare()

        async def bind() -> None:
            try:
                await asyncio.Future()
            finally:
                self.undeclare()

        return bind()

    @property
    def token_keyexpr(self):
        """The token keyexpr associated to this subscriber."""
        ses = afor.auto_session(self.session)
        return token_keyexpr(
            name=self.topic_info.topic,
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
    def subscriber_keyexpr(self):
        """The subscriber keyexpr associated to this subscriber."""
        return subscriber_keyexpr(
            name=self.topic_info.topic,
            dds_type=self.dds_type,
            hash=self.hash,
            namespace=self.namespace,
            domain_id=self.domain_id,
        )

    def declare(self):
        """Declares the subscriber on Zenoh and ROS."""
        ses = afor.auto_session(self.session)
        self.token = ses.liveliness().declare_token(self.token_keyexpr)
        self.zenoh_sub = ses.declare_subscriber(
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
        session: zenoh.Session | None = None,
        domain_id: int | str | None = None,
        namespace: str = "%",
        node_name: str | None = None,
        defer: bool = False,
        _hash: str | None = None,
        _dds_type: str | None = None,
        _enclave: str = "%",
        _node_id: str | int | None = None,
        _zenoh_id: str | None = None,
        _entity_id: int | str | None = None,
    ):
        """Creates a typed topic subscription.

        Args:
            msg_type: Message type used to deserialize incoming samples.
            topic: ROS topic name.
            qos_profile: QoS profile associated with the subscription.
            session: Zenoh session used to declare the underlying subscriber.
            domain_id: ROS domain id. If omitted, `ROS_DOMAIN_ID` is used and
                defaults to `0`.
            namespace: ROS namespace advertised for the subscriber.
            node_name: Name of the node the subscriber belongs to.
            defer: If `False`, background consumption starts immediately. If
                `True`, startup is deferred until `async_bind()` is called.
            _hash: Internal topic hash override.
            _dds_type: Internal DDS type override.
            _enclave: Internal enclave segment used when building the token
                keyexpr.
            _node_id: Internal id of the node the subscriber belongs to.
            _zenoh_id: Internal Zenoh session id override.
            _entity_id: Internal entity id override for this subscriber.
        """
        super().__init__()
        self.sample_sub: BaseSub[zenoh.Sample] = BaseSub()
        self.raw_sub = RawSub(
            msg_type=msg_type,
            topic=topic,
            qos_profile=qos_profile,
            callback=self.sample_sub.input_data,
            session=session,
            domain_id=domain_id,
            namespace=namespace,
            node_name=node_name,
            defer=True,
            _hash=_hash,
            _dds_type=_dds_type,
            _enclave=_enclave,
            _node_id=_node_id,
            _zenoh_id=_zenoh_id,
            _entity_id=_entity_id,
        )
        self.topic_info = TopicInfo(topic, msg_type, self.raw_sub.topic_info.qos)
        self._consumer_task: asyncio.Task | None = None
        if not defer:
            self.raw_sub.declare()
            self._consumer_task = asyncio.create_task(self.async_bind(True))

    async def _run(self):
        """Consume raw samples and forward deserialized messages downstream."""
        async for sample in self.sample_sub.listen_reliable(queue_size=0):
            parsed = self.topic_info.msg_type.deserialize(sample.payload.to_bytes())
            self._input_data_asyncio(parsed)

    def close(self):
        """Stops background consumption and closes the subscriber."""
        if self._consumer_task is not None:
            self._consumer_task.cancel()
        return super().close()

    def async_bind(self, __iam_init: bool = False) -> Coroutine[None, None, None]:
        """Runs the raw subscriber and decode loop inside the current task.

        Args:
            __iam_init: Internal flag used to avoid awaiting the task that is
                currently being created from `__init__`.

        Returns:
            A coroutine that keeps the subscription active until canceled or
            closed.
        """
        if self._consumer_task is not None:
            async def wait_for_task() -> None:
                if not __iam_init:
                    await self._consumer_task

            return wait_for_task()

        if self.raw_sub.token is None:
            self.raw_sub.declare()

        async def bind() -> None:
            try:
                async with asyncio.TaskGroup() as tg:
                    tg.create_task(self.raw_sub.async_bind())
                    tg.create_task(self._run())
            finally:
                self.close()

        return bind()
