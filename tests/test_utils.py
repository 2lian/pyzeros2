import hashlib
import inspect
import random
import string
from collections.abc import Sequence
from typing import Any, List, Type, get_args, get_origin

import pytest
from ros2_pyterfaces.cyclone import all_msgs, idl
from ros2_pyterfaces.cyclone.idl import message_to_plain_data
from ros2_pyterfaces.utils.random import random_message

MSG_TYPES: List[Type[idl.IdlStruct]] = [
    obj
    for obj in vars(all_msgs).values()
    if inspect.isclass(obj)
    and issubclass(obj, idl.IdlStruct)
    and obj is not idl.IdlStruct
]
NOT_IN_ROS = [
    # not in ROS yet
    "nav_msgs/msg/Trajectory",
    "nav_msgs/msg/TrajectoryPoint",
]
MSG_TYPES = [t for t in MSG_TYPES if t.get_type_name() not in NOT_IN_ROS]

MSG_TYPES_ids = [msg_type.get_type_name() for msg_type in MSG_TYPES]

MAX_SEQUENCE_LEN = 3
MAX_STRING_LEN = 12
STRING_ALPHABET = string.ascii_letters + string.digits + "_-/"
INTEGER_BOUNDS = {
    "byte": (0, 255),
    "int8": (-128, 127),
    "uint8": (0, 255),
    "int16": (-32768, 32767),
    "uint16": (0, 65535),
    "int32": (-(2**31), 2**31 - 1),
    "uint32": (0, 2**32 - 1),
    "int64": (-(2**63), 2**63 - 1),
    "uint64": (0, 2**64 - 1),
}

MSG_VALUES = [random_message(t) for t in MSG_TYPES*5]
MSG_VALUES_ids = [f"rand_value-{t.get_type_name()}" for t in MSG_VALUES]
