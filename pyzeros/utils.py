import logging
from dataclasses import dataclass, field
from typing import Any, Dict, Final, Generic, NamedTuple, Optional, Tuple, TypeVar

import ros_z_py
from asyncio_for_robotics.core.sub import BaseSub, _MsgType

logger = logging.getLogger(__name__)
QOS_DEFAULT = ros_z_py.QOS_DEFAULT


@dataclass(frozen=True, slots=True)
class TopicInfo(Generic[_MsgType]):
    """Precisely describes a ROS2 topic

    Attributes:
        name:
        msg_type:
        qos:
    """

    topic: str
    msg_type: _MsgType
    qos: ros_z_py.QosProfile = field(default_factory=lambda *_, **__: QOS_DEFAULT)

    def as_arg(self) -> Tuple[_MsgType, str, ros_z_py.QosProfile]:
        return (self.msg_type, self.topic, self.qos)

    def as_kwarg(self) -> Dict[str, Any]:
        return {"msg_type": self.msg_type, "topic": self.topic, "qos_profile": self.qos}


