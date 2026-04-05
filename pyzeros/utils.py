import logging
from dataclasses import dataclass, field
import msgspec
from enum import StrEnum
from typing import (
    Any,
    Dict,
    Final,
    Generic,
    Literal,
    NamedTuple,
    Optional,
    Tuple,
    TypeVar,
)

import numpy as np
import ros_z_py
from asyncio_for_robotics.core.sub import BaseSub, _MsgType
from nptyping import NDArray, Shape, UInt8, Int8
from ros2_pyterfaces.cydr.idl import IdlStruct, types

logger = logging.getLogger(__name__)


class Attachment(IdlStruct):
    sequence_number: types.int64 = types.int64(0)
    source_timestamp: types.int64 = types.int64(0)
    gid_length: types.uint8 = types.uint8(16)
    source_gid: NDArray[Shape["16"], UInt8] = msgspec.field(
        default_factory=lambda *_: np.zeros(16, dtype=np.uint8)
    )


def ros_type_to_dds_type(ros_type: str) -> str:
    parts = ros_type.strip("/").split("/")
    if len(parts) != 3:
        raise ValueError(
            f"Expected 'package/msg/Type' or 'package/srv/Type', got: {ros_type!r}"
        )

    package, interface_kind, type_name = parts
    if interface_kind not in {"msg", "srv", "action"}:
        raise ValueError(
            f"Unsupported interface kind {interface_kind!r} in {ros_type!r}"
        )

    return f"{package}::{interface_kind}::dds_::{type_name}_"


class CdrModes(StrEnum):
    """Select how message bytes are encoded or decoded."""

    AUTO = "auto"
    ROS_Z = "ros_z"
    PYTERFACE = "pyterface"


def is_ros2pyterfaces(msg_type: type) -> bool:
    return (
        getattr(msg_type, "serialize", None) is not None
        and getattr(msg_type, "deserialize", None) is not None
        and getattr(msg_type, "get_type_name", None) is not None
        and getattr(msg_type, "hash_rihs01", None) is not None
    )


def deduce_cdr_mode(
    msg_type: type[_MsgType], cdr_mode: CdrModes
) -> Literal[CdrModes.ROS_Z, CdrModes.PYTERFACE]:
    """Resolve `AUTO` into the concrete serialization mode for `msg_type`."""
    if cdr_mode != CdrModes.AUTO:
        return cdr_mode
    if is_ros2pyterfaces(msg_type):
        return CdrModes.PYTERFACE
    else:
        return CdrModes.ROS_Z


def get_type_shim(msg_type: Any, cdr_mode: CdrModes = CdrModes.AUTO):
    """Return the class object that should be passed to `ros_z_py`.

    Native `ros_z_py` message classes are returned unchanged. `IdlStruct`
    classes are converted into a shim exposing the ROS type metadata expected by
    the binding.
    """
    cdr_mode = deduce_cdr_mode(msg_type, cdr_mode)

    if cdr_mode == CdrModes.AUTO:
        raise ValueError()
    elif cdr_mode == CdrModes.ROS_Z:
        type_dummy = msg_type
    elif cdr_mode == CdrModes.PYTERFACE:
        type_dummy = make_ros_z_shim_type(msg_type)
    else:
        raise ValueError()

    return type_dummy


def make_ros_z_shim_type(msg_type: Any) -> type[Any]:
    """Build a lightweight shim class exposing `__msgtype__` and `__hash__`."""
    return type(
        f"{msg_type.get_type_name().replace("/", "__")}_RosZShim",
        (),
        {
            "__msgtype__": msg_type.get_type_name(),
            "__hash__": msg_type.hash_rihs01(),
        },
    )


@dataclass(frozen=True, slots=True)
class TopicInfo(Generic[_MsgType]):
    """Container for a topic name, its message class, and its QoS profile."""

    topic: str
    msg_type: _MsgType
    qos: ros_z_py.QosProfile = field(default_factory=lambda *_, **__: QOS_DEFAULT)

    def as_arg(self) -> Tuple[_MsgType, str, ros_z_py.QosProfile]:
        return (self.msg_type, self.topic, self.qos)

    def as_kwarg(self) -> Dict[str, Any]:
        return {"msg_type": self.msg_type, "topic": self.topic, "qos_profile": self.qos}
