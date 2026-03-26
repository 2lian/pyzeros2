import logging
from typing import (
    Any,
    Callable,
    Dict,
    Final,
    Generic,
    Literal,
    NamedTuple,
    Optional,
    Tuple,
    TypeVar,
    overload,
)

import ros_z_py
from asyncio_for_robotics.core.sub import BaseSub

from .session import ZNode, auto_session
from .utils import QOS_DEFAULT, CdrModes, TopicInfo, deduce_cdr_mode, get_type_shim

logger = logging.getLogger(__name__)


_MsgType = TypeVar("_MsgType")
_SubOutputType = TypeVar("_SubOutputType")


class Sub(BaseSub[_SubOutputType]):
    """`BaseSub` adapter backed by a `ros_z_py` subscription.

    Use this class when you want a ROS topic to behave like an
    `asyncio_for_robotics` subscriber. The yielded message type depends on the
    constructor arguments:

    - with `raw=False`, values yielded by `listen()` / `listen_reliable()` are
      decoded into `msg_type`,
    - with `raw=True`, yielded values are `ros_z_py.ZPayloadView`.

    `msg_type` can be a native `ros_z_py` message class or a
    `ros2_pyterfaces.idl.IdlStruct` subclass. In `AUTO` mode, `IdlStruct`
    classes use the Python serialization path and native `ros_z_py` classes use
    the native path.
    """

    @overload
    def __new__(
        cls,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: ros_z_py.QosProfile = QOS_DEFAULT,
        session: Optional[ZNode] = None,
        raw: Literal[False] = False,
        cdr_mode: CdrModes = CdrModes.AUTO,
    ) -> "Sub[_MsgType]": ...

    @overload
    def __new__(
        cls,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: ros_z_py.QosProfile = QOS_DEFAULT,
        session: Optional[ZNode] = None,
        raw: Literal[True] = True,
        cdr_mode: CdrModes = CdrModes.AUTO,
    ) -> "Sub[ros_z_py.ZPayloadView]": ...

    @overload
    def __new__(
        cls,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: ros_z_py.QosProfile = QOS_DEFAULT,
        session: Optional[ZNode] = None,
        raw: bool = False,
        cdr_mode: CdrModes = CdrModes.AUTO,
    ) -> "Sub[_MsgType | ros_z_py.ZPayloadView]": ...

    def __new__(cls, *args: Any, **kwargs: Any) -> "Sub[Any]":
        return super().__new__(cls)

    @overload
    def __init__(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: ros_z_py.QosProfile = QOS_DEFAULT,
        session: Optional[ZNode] = None,
        raw: Literal[False] = False,
        cdr_mode: CdrModes = CdrModes.AUTO,
    ) -> None: ...

    @overload
    def __init__(
        self,
        msg_type: type[Any],
        topic: str,
        qos_profile: ros_z_py.QosProfile = QOS_DEFAULT,
        session: Optional[ZNode] = None,
        cdr_mode: CdrModes = CdrModes.AUTO,
        raw: Literal[True] = True,
    ) -> None: ...

    @overload
    def __init__(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: ros_z_py.QosProfile = QOS_DEFAULT,
        session: Optional[ZNode] = None,
        raw: bool = False,
        cdr_mode: CdrModes = CdrModes.AUTO,
    ) -> None: ...

    def __init__(
        self,
        msg_type: type[Any],
        topic: str,
        qos_profile: ros_z_py.QosProfile = QOS_DEFAULT,
        session: Optional[ZNode] = None,
        raw: bool = False,
        cdr_mode: CdrModes = CdrModes.AUTO,
    ) -> None:
        """Create a topic subscription.

        Args:
            msg_type: Message class associated with the topic. This can be a
                native `ros_z_py` type or a
                `ros2_pyterfaces.idl.IdlStruct` subclass.
            topic: ROS topic name.
            qos_profile: QoS profile applied to the underlying subscription.
            session: Existing `ZNode` to use. If omitted, `auto_session()`
                creates or reuses a global node.
            raw: If `False`, incoming samples are decoded into `msg_type`. If
                `True`, incoming samples are exposed as zero copy
                `ros_z_py.ZPayloadView`.
            cdr_mode: Serialization strategy override. `AUTO` selects the
                `IdlStruct` path for Python-defined or `ros2_pyterfaces`
                messages and the native path otherwise.
        """
        self.session: ZNode = self._resolve_session(session)
        self.topic_info: TopicInfo[type[Any]] = TopicInfo(
            topic=topic, msg_type=msg_type, qos=qos_profile
        )
        self.cdr_mode: CdrModes = self._deduce_cdr_mode(msg_type, cdr_mode)
        self.sub: ros_z_py.ZSubscriber = self._resolve_sub(
            self.topic_info, cdr_mode, raw
        )
        super().__init__()

    @staticmethod
    def _deduce_cdr_mode(
        msg_type: type[_MsgType], cdr_mode: CdrModes
    ) -> Literal[CdrModes.ROS_Z, CdrModes.PYTERFACE]:
        return deduce_cdr_mode(msg_type, cdr_mode)

    @property
    def name(self) -> str:
        return f"Sub Py0s-{self.topic_info.topic}"

    def _resolve_session(self, session: Optional[ZNode]) -> ZNode:
        return auto_session(session)

    def _resolve_sub(self, topic_info: TopicInfo, cdr_mode: CdrModes, raw: bool):
        if cdr_mode == CdrModes.AUTO:
            cdr_mode = self._deduce_cdr_mode(topic_info.msg_type, cdr_mode)

        if cdr_mode == CdrModes.AUTO:
            raise ValueError()
        elif cdr_mode == CdrModes.ROS_Z:
            cbk = self.callback_for_sub
            raw = raw
            print("oh no")
        elif cdr_mode == CdrModes.PYTERFACE:
            if raw == True:
                cbk = self.callback_for_sub
            else:
                cbk = lambda msg: self.callback_for_sub(
                    topic_info.msg_type.deserialize(memoryview(msg))
                )
            raw = True
        else:
            raise ValueError()

        type_dummy = get_type_shim(topic_info.msg_type, cdr_mode)  # type: ignore
        return self.session.create_subscriber(
            topic=topic_info.topic,
            msg_type=type_dummy,
            qos=topic_info.qos,
            callback=cbk,
            raw=raw,
        )

    def callback_for_sub(self, sample: _SubOutputType):
        if self._closed.is_set():
            return
        try:
            healty = self.input_data(sample)
            if not healty:
                self.close()
        except Exception as e:
            logger.error(e)

    def close(self):
        # not fully implemented missing in rust
        super().close()
