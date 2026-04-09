from typing import List, Tuple, Type

import pytest
from ros2_pyterfaces.cyclone import idl
from test_utils import ALL_TYPES, ALL_TYPES_ids

import pyzeros
from pyzeros.pub import Pub
from pyzeros.utils import CdrModes, get_type_shim


@pytest.mark.parametrize("msg_type", ALL_TYPES, ids=ALL_TYPES_ids)
async def outdated_test_cdr_modes(msg_type: Type[idl.IdlStruct]):
    py0_topic = pyzeros.utils.TopicInfo(f"/tests/{msg_type.get_type_name()}", msg_type)
    cdr_mode = Pub._deduce_cdr_mode(py0_topic.msg_type, CdrModes.AUTO)
    assert cdr_mode == CdrModes.PYTERFACE
