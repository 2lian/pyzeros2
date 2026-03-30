import time
from asyncio_for_robotics import BaseSub
import pytest

from asyncio_for_robotics.core.sub import ConverterSub

import asyncio
import logging
from contextlib import suppress
from typing import Any, AsyncGenerator, Callable, Generator, Optional, Union

from ros2_pyterfaces.cyclone import all_msgs
from ros_z_py import QosProfile

from afor_tests import (
    test_freshness,
    test_listen_one_by_one,
    test_listen_too_fast,
    test_loop_cancellation,
    test_reliable_extremely_fast,
    test_reliable_one_by_one,
    test_reliable_too_fast,
    test_wait_cancellation,
    test_wait_for_value,
    test_wait_new,
    test_wait_next,
)

from pyzeros.pub import ZPublisher
from pyzeros.sub import Sub
from pyzeros.utils import TopicInfo

logger = logging.getLogger("asyncio_for_robotics.test")


@pytest.fixture(scope="module", autouse=True)
def session():
    return

topic = TopicInfo("test/something", all_msgs.String, qos=QosProfile(depth=100000))

@pytest.fixture(scope="module")
def pub(session) -> Generator[Callable[[str], None], Any, Any]:
    pub_topic = "test/something"
    logger.debug("Creating PUB-%s", pub_topic)
    p: ZPublisher = ZPublisher(*topic.as_arg())
    time.sleep(1)

    def pub_func(input: str):
        p.publish(topic.msg_type(data=input))

    yield pub_func

@pytest.fixture
async def sub(session) -> AsyncGenerator[BaseSub[str], Any]:
    inner_sub = Sub(*topic.as_arg())
    s: BaseSub[str] = ConverterSub(inner_sub, lambda msg: msg.data)
    yield s
    inner_sub.close()
    s.close()

