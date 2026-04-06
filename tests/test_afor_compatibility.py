import time
import asyncio
import logging
from contextlib import suppress
from typing import Any, AsyncGenerator, Callable, Generator, Optional, Union

import pytest
from asyncio_for_robotics import BaseSub
from asyncio_for_robotics.core.sub import ConverterSub
from ros2_pyterfaces.cydr import all_msgs

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

from pyzeros.node import Node
from pyzeros.pub import Pub
from pyzeros.sub import Sub
from pyzeros.utils import TopicInfo

logger = logging.getLogger("asyncio_for_robotics.test")


@pytest.fixture(scope="module")
def session():
    node = Node(name="afor_compat", namespace="/tests")
    yield node
    node.undeclare()

topic = TopicInfo("test/something", all_msgs.String)

@pytest.fixture(scope="module")
def pub(session: Node) -> Generator[Callable[[str], None], Any, Any]:
    pub_topic = "test/something"
    logger.debug("Creating PUB-%s", pub_topic)
    p: Pub = session.create_publisher(*topic.as_arg())
    time.sleep(1)

    def pub_func(input: str):
        p.publish(topic.msg_type(data=input.encode("utf-8")))

    yield pub_func
    p.undeclare()

@pytest.fixture
async def sub(session: Node) -> AsyncGenerator[BaseSub[str], Any]:
    inner_sub: Sub = session.create_subscriber(*topic.as_arg())
    s: BaseSub[str] = ConverterSub(inner_sub, lambda msg: msg.data.decode("utf-8"))
    yield s
    inner_sub.close()
    s.close()
