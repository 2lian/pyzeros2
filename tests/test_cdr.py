from typing import List, Tuple, Type

import pytest
from ros2_pyterfaces import all_msgs, idl
from test_utils import MSG_TYPES, MSG_TYPES_ids

import pyzeros
from pyzeros.pub import ZPublisher
from pyzeros.utils import CdrModes, get_type_shim


@pytest.mark.parametrize("msg_type", MSG_TYPES, ids=MSG_TYPES_ids)
async def ntest_cdr_modes(msg_type: Type[idl.IdlStruct]):
    py0_topic = pyzeros.utils.TopicInfo(f"/tests/{msg_type.get_type_name()}", msg_type)
    cdr_mode = ZPublisher._deduce_cdr_mode(py0_topic.msg_type, CdrModes.AUTO)
    assert cdr_mode == CdrModes.PYTERFACE
