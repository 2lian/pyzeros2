import inspect
from typing import Any, List, Type, get_args, get_origin

import pytest
from ros2_pyterfaces.core.random import random_message
from ros2_pyterfaces.cyclone import all_msgs as cycl_msgs
from ros2_pyterfaces.cyclone.idl import IdlStruct as CyclStruct
from ros2_pyterfaces.cydr import all_msgs as cydr_msgs
from ros2_pyterfaces.cydr.idl import IdlStruct as CydrStruct

NOT_IN_ROS = [
    # not in ROS yet
    "nav_msgs/msg/Trajectory",
    "nav_msgs/msg/TrajectoryPoint",
    "test_msgs/msg/Builtins",
]

####################### CYLCONE #######################

CYCL_TYPES: List[Type[CyclStruct]] = [
    msg_type
    for msg_type in vars(cycl_msgs).values()
    if inspect.isclass(msg_type)
    and issubclass(msg_type, CyclStruct)
    and msg_type is not CyclStruct
]
CYCL_TYPES = [t for t in CYCL_TYPES if t.get_type_name() not in NOT_IN_ROS]
CYCL_TYPES_ids = [f"cycl-{msg_type.get_type_name()}" for msg_type in CYCL_TYPES]
CYCL_VALUES = [
    t.from_core_message(random_message(t.to_core_schema())) for t in CYCL_TYPES
]
CYCL_VALUES_ids = [f"cycl_val-{t.get_type_name()}" for t in CYCL_VALUES]

####################### CYDR #######################

def _is_supported(value: Any) -> bool:
    return not isinstance(getattr(value, "__unsupported_reason__", None), str)


CYDR_TYPES: List[Type[CydrStruct]] = [
    obj
    for obj in vars(cydr_msgs).values()
    if inspect.isclass(obj)
    and issubclass(obj, CydrStruct)
    and obj is not CydrStruct
    and _is_supported(obj)
]
CYDR_TYPES = [t for t in CYDR_TYPES if t.get_type_name() not in NOT_IN_ROS]
CYDR_TYPES_ids = [f"cydr-{msg_type.get_type_name()}" for msg_type in CYDR_TYPES]
CYDR_VALUES = [
    t.from_core_message(random_message(t.to_core_schema())) for t in CYDR_TYPES
]
CYDR_VALUES_ids = [f"cydr_val-{t.get_type_name()}" for t in CYDR_VALUES]

####################### CYLCONE #######################

ALL_TYPES = CYCL_TYPES + CYDR_TYPES 
ALL_VALUES = CYCL_VALUES + CYDR_VALUES 
ALL_TYPES_ids = CYCL_TYPES_ids + CYDR_TYPES_ids 
ALL_VALUES_ids = CYCL_VALUES_ids + CYDR_VALUES_ids 
