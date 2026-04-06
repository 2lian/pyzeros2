import logging
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

import msgspec
import numpy as np
from nptyping import Int8, NDArray, Shape, UInt8
from ros2_pyterfaces.cydr.idl import IdlStruct, types

logger = logging.getLogger(__name__)


class Attachment(IdlStruct):
    """Should be published without header like so:
        attachment=Attachment().serialize()[4:]

    Attributes:
        sequence_number: Everincreasing number per publisher
        source_timestamp: publication time
        gid_length: Deprecated in rolling >Kilted
        source_gid: ID of th epublisher
    """

    sequence_number: types.int64 = types.int64(0)
    source_timestamp: types.int64 = types.int64(0)
    gid_length: types.uint8 = types.uint8(16)
    source_gid: NDArray[Shape["16"], UInt8] = msgspec.field(
        default_factory=lambda *_: np.zeros(16, dtype=np.uint8)
    )
