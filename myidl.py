import asyncio
import itertools
import struct
import time
import uuid
from contextlib import suppress
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pprint import pformat, pprint
from typing import ClassVar, Optional, Self

import asyncio_for_robotics.zenoh as afor
import zenoh
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import (
    array,
    bounded_str,
    float32,
    float64,
    int8,
    int16,
    int32,
    int64,
    sequence,
    uint8,
    uint16,
    uint32,
    uint64,
)

@dataclass
class Attachment(IdlStruct):
    """Should be published without header like so:
        attachment=Attachment().serialize()[4:]

    Attributes: 
        sequence_number: 
        source_timestamp: 
        gid_length: 
        source_gid: 
    """
    sequence_number: int64 = 0
    source_timestamp: int64 = 0
    gid_length: uint8 = 16
    source_gid: array[uint8, 16] = field(default_factory=lambda *_: [0] * 16)


@dataclass
class String(IdlStruct):
    data: str = ""

@dataclass
class Time(IdlStruct):
    sec: int32 = 0
    nanosec: int32 = 0

@dataclass
class ParameterValue(IdlStruct):
    type: uint8 
    bool_value: bool
    integer_value: int
    double_value: float
    string_value: str
    byte_array_value: sequence[uint8]
    bool_array_value: sequence[bool]
    integer_array_value: sequence[int]
    double_array_value: sequence[float]
    string_array_value: sequence[str]

@dataclass
class Parameter(IdlStruct):
    name: str
    value: ParameterValue

@dataclass
class ParameterEvent(IdlStruct):
    stamp: Time
    node: str
    new_parameters: sequence[Parameter]
    changed_parameters: sequence[Parameter]
    deleted_parameters: sequence[Parameter]

@dataclass
class Vector3D(IdlStruct):
    x: float64 = 0
    y: float64 = 0
    z: float64 = 0

# ---------------------------------------------------------------------------
# type_description_interfaces/msg/FieldType
# ---------------------------------------------------------------------------

@dataclass
class FieldType(IdlStruct):
    # Constants from the ROS message definition
    # FIELD_TYPE_NOT_SET: ClassVar[int] = 0
    # FIELD_TYPE_NESTED_TYPE: ClassVar[int] = 1
    #
    # FIELD_TYPE_INT8: ClassVar[int] = 2
    # FIELD_TYPE_UINT8: ClassVar[int] = 3
    # FIELD_TYPE_INT16: ClassVar[int] = 4
    # FIELD_TYPE_UINT16: ClassVar[int] = 5
    # FIELD_TYPE_INT32: ClassVar[int] = 6
    # FIELD_TYPE_UINT32: ClassVar[int] = 7
    # FIELD_TYPE_INT64: ClassVar[int] = 8
    # FIELD_TYPE_UINT64: ClassVar[int] = 9
    #
    # FIELD_TYPE_FLOAT: ClassVar[int] = 10
    # FIELD_TYPE_DOUBLE: ClassVar[int] = 11
    # FIELD_TYPE_LONG_DOUBLE: ClassVar[int] = 12
    # FIELD_TYPE_CHAR: ClassVar[int] = 13
    # FIELD_TYPE_WCHAR: ClassVar[int] = 14
    # FIELD_TYPE_BOOLEAN: ClassVar[int] = 15
    # FIELD_TYPE_BYTE: ClassVar[int] = 16
    # FIELD_TYPE_STRING: ClassVar[int] = 17
    # FIELD_TYPE_WSTRING: ClassVar[int] = 18
    # FIELD_TYPE_FIXED_STRING: ClassVar[int] = 19
    # FIELD_TYPE_FIXED_WSTRING: ClassVar[int] = 20
    # FIELD_TYPE_BOUNDED_STRING: ClassVar[int] = 21
    # FIELD_TYPE_BOUNDED_WSTRING: ClassVar[int] = 22
    #
    # FIELD_TYPE_NESTED_TYPE_ARRAY: ClassVar[int] = 49
    # FIELD_TYPE_INT8_ARRAY: ClassVar[int] = 50
    # FIELD_TYPE_UINT8_ARRAY: ClassVar[int] = 51
    # FIELD_TYPE_INT16_ARRAY: ClassVar[int] = 52
    # FIELD_TYPE_UINT16_ARRAY: ClassVar[int] = 53
    # FIELD_TYPE_INT32_ARRAY: ClassVar[int] = 54
    # FIELD_TYPE_UINT32_ARRAY: ClassVar[int] = 55
    # FIELD_TYPE_INT64_ARRAY: ClassVar[int] = 56
    # FIELD_TYPE_UINT64_ARRAY: ClassVar[int] = 57
    # FIELD_TYPE_FLOAT_ARRAY: ClassVar[int] = 58
    # FIELD_TYPE_DOUBLE_ARRAY: ClassVar[int] = 59
    # FIELD_TYPE_LONG_DOUBLE_ARRAY: ClassVar[int] = 60
    # FIELD_TYPE_CHAR_ARRAY: ClassVar[int] = 61
    # FIELD_TYPE_WCHAR_ARRAY: ClassVar[int] = 62
    # FIELD_TYPE_BOOLEAN_ARRAY: ClassVar[int] = 63
    # FIELD_TYPE_BYTE_ARRAY: ClassVar[int] = 64
    # FIELD_TYPE_STRING_ARRAY: ClassVar[int] = 65
    # FIELD_TYPE_WSTRING_ARRAY: ClassVar[int] = 66
    # FIELD_TYPE_FIXED_STRING_ARRAY: ClassVar[int] = 67
    # FIELD_TYPE_FIXED_WSTRING_ARRAY: ClassVar[int] = 68
    # FIELD_TYPE_BOUNDED_STRING_ARRAY: ClassVar[int] = 69
    # FIELD_TYPE_BOUNDED_WSTRING_ARRAY: ClassVar[int] = 70
    #
    # FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE: ClassVar[int] = 97
    # FIELD_TYPE_INT8_BOUNDED_SEQUENCE: ClassVar[int] = 98
    # FIELD_TYPE_UINT8_BOUNDED_SEQUENCE: ClassVar[int] = 99
    # FIELD_TYPE_INT16_BOUNDED_SEQUENCE: ClassVar[int] = 100
    # FIELD_TYPE_UINT16_BOUNDED_SEQUENCE: ClassVar[int] = 101
    # FIELD_TYPE_INT32_BOUNDED_SEQUENCE: ClassVar[int] = 102
    # FIELD_TYPE_UINT32_BOUNDED_SEQUENCE: ClassVar[int] = 103
    # FIELD_TYPE_INT64_BOUNDED_SEQUENCE: ClassVar[int] = 104
    # FIELD_TYPE_UINT64_BOUNDED_SEQUENCE: ClassVar[int] = 105
    # FIELD_TYPE_FLOAT_BOUNDED_SEQUENCE: ClassVar[int] = 106
    # FIELD_TYPE_DOUBLE_BOUNDED_SEQUENCE: ClassVar[int] = 107
    # FIELD_TYPE_LONG_DOUBLE_BOUNDED_SEQUENCE: ClassVar[int] = 108
    # FIELD_TYPE_CHAR_BOUNDED_SEQUENCE: ClassVar[int] = 109
    # FIELD_TYPE_WCHAR_BOUNDED_SEQUENCE: ClassVar[int] = 110
    # FIELD_TYPE_BOOLEAN_BOUNDED_SEQUENCE: ClassVar[int] = 111
    # FIELD_TYPE_BYTE_BOUNDED_SEQUENCE: ClassVar[int] = 112
    # FIELD_TYPE_STRING_BOUNDED_SEQUENCE: ClassVar[int] = 113
    # FIELD_TYPE_WSTRING_BOUNDED_SEQUENCE: ClassVar[int] = 114
    # FIELD_TYPE_FIXED_STRING_BOUNDED_SEQUENCE: ClassVar[int] = 115
    # FIELD_TYPE_FIXED_WSTRING_BOUNDED_SEQUENCE: ClassVar[int] = 116
    # FIELD_TYPE_BOUNDED_STRING_BOUNDED_SEQUENCE: ClassVar[int] = 117
    # FIELD_TYPE_BOUNDED_WSTRING_BOUNDED_SEQUENCE: ClassVar[int] = 118
    #
    # FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE: ClassVar[int] = 145
    # FIELD_TYPE_INT8_UNBOUNDED_SEQUENCE: ClassVar[int] = 146
    # FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE: ClassVar[int] = 147
    # FIELD_TYPE_INT16_UNBOUNDED_SEQUENCE: ClassVar[int] = 148
    # FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE: ClassVar[int] = 149
    # FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE: ClassVar[int] = 150
    # FIELD_TYPE_UINT32_UNBOUNDED_SEQUENCE: ClassVar[int] = 151
    # FIELD_TYPE_INT64_UNBOUNDED_SEQUENCE: ClassVar[int] = 152
    # FIELD_TYPE_UINT64_UNBOUNDED_SEQUENCE: ClassVar[int] = 153
    # FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE: ClassVar[int] = 154
    # FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE: ClassVar[int] = 155
    # FIELD_TYPE_LONG_DOUBLE_UNBOUNDED_SEQUENCE: ClassVar[int] = 156
    # FIELD_TYPE_CHAR_UNBOUNDED_SEQUENCE: ClassVar[int] = 157
    # FIELD_TYPE_WCHAR_UNBOUNDED_SEQUENCE: ClassVar[int] = 158
    # FIELD_TYPE_BOOLEAN_UNBOUNDED_SEQUENCE: ClassVar[int] = 159
    # FIELD_TYPE_BYTE_UNBOUNDED_SEQUENCE: ClassVar[int] = 160
    # FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE: ClassVar[int] = 161
    # FIELD_TYPE_WSTRING_UNBOUNDED_SEQUENCE: ClassVar[int] = 162
    # FIELD_TYPE_FIXED_STRING_UNBOUNDED_SEQUENCE: ClassVar[int] = 163
    # FIELD_TYPE_FIXED_WSTRING_UNBOUNDED_SEQUENCE: ClassVar[int] = 164
    # FIELD_TYPE_BOUNDED_STRING_UNBOUNDED_SEQUENCE: ClassVar[int] = 165
    # FIELD_TYPE_BOUNDED_WSTRING_UNBOUNDED_SEQUENCE: ClassVar[int] = 166

    type_id: uint8 = 0
    capacity: uint64 = 0
    string_capacity: uint64 = 0
    nested_type_name: bounded_str[255] = ""


# ---------------------------------------------------------------------------
# type_description_interfaces/msg/Field
# ---------------------------------------------------------------------------

@dataclass
class Field(IdlStruct):
    name: str = ""
    type: FieldType = field(default_factory=FieldType)
    default_value: str = ""


# ---------------------------------------------------------------------------
# type_description_interfaces/msg/IndividualTypeDescription
# ---------------------------------------------------------------------------

@dataclass
class IndividualTypeDescription(IdlStruct):
    type_name: bounded_str[255] = ""
    fields: sequence[Field] = field(default_factory=list)


# ---------------------------------------------------------------------------
# type_description_interfaces/msg/TypeDescription
# ---------------------------------------------------------------------------

@dataclass
class TypeDescription(IdlStruct):
    type_description: IndividualTypeDescription = field(
        default_factory=IndividualTypeDescription
    )
    referenced_type_descriptions: sequence[IndividualTypeDescription] = field(
        default_factory=list
    )


# ---------------------------------------------------------------------------
# type_description_interfaces/msg/TypeSource
# ---------------------------------------------------------------------------

@dataclass
class TypeSource(IdlStruct):
    type_name: str = ""
    encoding: str = ""
    raw_file_contents: str = ""


# ---------------------------------------------------------------------------
# type_description_interfaces/msg/KeyValue
# ---------------------------------------------------------------------------

@dataclass
class KeyValue(IdlStruct):
    key: str = ""
    value: str = ""


# ---------------------------------------------------------------------------
# type_description_interfaces/srv/GetTypeDescription
# ---------------------------------------------------------------------------

@dataclass
class GetTypeDescription_Request(IdlStruct):
    type_name: str = ""
    type_hash: str = ""
    include_type_sources: bool = True


@dataclass
class GetTypeDescription_Response(IdlStruct):
    successful: bool = False
    failure_reason: str = ""
    type_description: TypeDescription = field(default_factory=TypeDescription)
    type_sources: sequence[TypeSource] = field(default_factory=list)
    extra_information: sequence[KeyValue] = field(default_factory=list)
