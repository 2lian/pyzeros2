from typing import Any, Generic, Literal, Optional, TypeVar

import ros_z_py
from ros2_pyterfaces.cyclone.idl import IdlStruct

from .session import ZNode, auto_session
from .sub import Sub, TopicInfo
from .utils import QOS_DEFAULT, CdrModes, get_type_shim

_MsgType = TypeVar("_MsgType")


class ZPublisher(Generic[_MsgType]):
    """Small wrapper around a `ros_z_py` publisher.

    `msg_type` can be a native `ros_z_py` message class or a
    `ros2_pyterfaces.idl.IdlStruct` subclass. In the latter case,
    `publish()` serializes the Python object before forwarding it to `ros-z`.
    """

    def __init__(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: ros_z_py.QosProfile = QOS_DEFAULT,
        session: Optional[ZNode] = None,
        cdr_mode: CdrModes = CdrModes.AUTO,
    ):
        self.session: ZNode = self._resolve_session(session)
        self.topic_info: TopicInfo[type[_MsgType]] = TopicInfo(
            topic=topic, msg_type=msg_type, qos=qos_profile
        )
        self.cdr_mode: CdrModes = self._deduce_cdr_mode(
            self.topic_info.msg_type, cdr_mode
        )
        self.zpub: ros_z_py.ZPublisher = self._resolve_publisher(
            self.topic_info, cdr_mode
        )

    def _resolve_session(self, session: Optional[ZNode]) -> ZNode:
        return auto_session(session)

    @staticmethod
    def _deduce_cdr_mode(
        msg_type: type[_MsgType], cdr_mode: CdrModes
    ) -> Literal[CdrModes.ROS_Z, CdrModes.PYTERFACE]:
        return Sub._deduce_cdr_mode(msg_type, cdr_mode)

    def _resolve_publisher(
        self, topic_info: TopicInfo, cdr_mode: CdrModes
    ) -> ros_z_py.ZPublisher:
        type_dummy = get_type_shim(topic_info.msg_type, cdr_mode)  # type: ignore
        return self.session.create_publisher(
            topic=topic_info.topic,
            msg_type=type_dummy,
            qos=topic_info.qos,
        )

    def publish(self, data: _MsgType | bytes | memoryview) -> None:
        """Publish one message.

        Args:
            data: Either a typed message instance or a pre-serialized payload.
                `bytes` are forwarded with `publish_raw()`. `IdlStruct`
                instances are serialized in Python before publishing.
        """
        if isinstance(data, (bytes, bytearray, memoryview)):
            self.zpub.publish_raw(data)
            return
        cdr = self._deduce_cdr_mode(type[data], self.cdr_mode)
        if cdr == CdrModes.PYTERFACE:
            d: IdlStruct = data  # type: ignore
            self.zpub.publish_raw(d.serialize())
        elif cdr == CdrModes.ROS_Z:
            self.zpub.publish(data)
        else:
            raise ValueError("TODO")

    def publish_raw(self, data: bytes) -> None:
        """Publish pre-serialized payload bytes unchanged."""
        self.zpub.publish_raw(data)

    def get_type_name(self) -> str:
        """Return the ROS type name advertised by the publisher."""
        return self.zpub.get_type_name()
