from asyncio import TaskGroup
import asyncio
from pprint import pformat
from typing import Any, NamedTuple, Type

import asyncio_for_robotics.ros2 as afor
import pytest
import pytest_asyncio
from ros2_pyterfaces.cyclone import all_msgs, idl
from test_utils import ALL_TYPES, ALL_TYPES_ids, random_message

from pyzeros.pub import Pub
from pyzeros.sub import Sub
from pyzeros.utils import TopicInfo

FUZZ_MESSAGE_COUNT = 5
PUBLISH_RETRY_HZ = 100
RECV_TIMEOUT_S = 2
BUF_TIME =(1/PUBLISH_RETRY_HZ)/5

pytestmark = pytest.mark.asyncio(loop_scope="module")


class FuzzPubSub(NamedTuple):
    msg_type: Type[idl.IdlStruct]
    ros_topic: afor.TopicInfo
    py0_topic: TopicInfo
    publisher: Any
    subscriber: Any

@pytest.fixture(scope="module", params=ALL_TYPES, ids=ALL_TYPES_ids)
def msg_type(request) -> Type[idl.IdlStruct]:
    return request.param


def topic_name(direction: str, msg_type: Type[idl.IdlStruct]) -> str:
    return f"/tests/{direction}/{msg_type.get_type_name()}"


def fuzzed_messages(msg_type: Type[idl.IdlStruct]) -> list[idl.IdlStruct]:
    return [msg_type()] + [
        msg_type.from_core_message(random_message(msg_type.to_core_schema(), seed=seed))
        for seed in range(FUZZ_MESSAGE_COUNT)
    ]


@pytest_asyncio.fixture(scope="module", loop_scope="module")
async def py0_to_ros_pubsub(rclpy_init, msg_type: Type[idl.IdlStruct]) -> FuzzPubSub:
    ros_topic = afor.TopicInfo(
        topic_name("py0_to_ros", msg_type), msg_type.to_ros_type()
    )
    py0_topic = TopicInfo(topic_name("py0_to_ros", msg_type), msg_type)

    pub = Pub(*py0_topic.as_arg())
    ros_sub = afor.Sub(*ros_topic.as_arg())
    yield FuzzPubSub(msg_type, ros_topic, py0_topic, pub, ros_sub)
    ros_sub.close()


@pytest_asyncio.fixture(scope="module", loop_scope="module")
async def ros_to_py0_pubsub(rclpy_init, msg_type: Type[idl.IdlStruct]) -> FuzzPubSub:
    ros_topic = afor.TopicInfo(
        topic_name("ros_to_py0", msg_type), msg_type.to_ros_type()
    )
    py0_topic = TopicInfo(topic_name("ros_to_py0", msg_type), msg_type)

    with afor.auto_session().lock() as node:
        ros_pub = node.create_publisher(*ros_topic.as_arg())
    sub = Sub(*py0_topic.as_arg())
    yield FuzzPubSub(msg_type, ros_topic, py0_topic, ros_pub, sub)
    with afor.auto_session().lock() as node:
        node.destroy_publisher(ros_pub)
    sub.close()


@pytest_asyncio.fixture(scope="module", loop_scope="module")
async def py0_to_py0_pubsub(msg_type: Type[idl.IdlStruct]) -> FuzzPubSub:
    py0_topic = TopicInfo(topic_name("py0_to_py0", msg_type), msg_type)

    pub = Pub(*py0_topic.as_arg())
    sub = Sub(*py0_topic.as_arg())
    yield FuzzPubSub(msg_type, py0_topic, py0_topic, pub, sub)
    sub.close()


async def test_py0_to_ros_fuzzed(py0_to_ros_pubsub: FuzzPubSub):
    msg_type, ros_topic, py0_topic, pub, ros_sub = py0_to_ros_pubsub

    async def periodic(msg: idl.IdlStruct):
        pub.publish(msg)
        rate = afor.Rate(PUBLISH_RETRY_HZ)
        try:
            async for _ in rate.listen():
                pub.publish(msg)
        finally:
            rate.close()

    for py0_msg in fuzzed_messages(msg_type):
        async with TaskGroup() as tg:
            publish_task = tg.create_task(periodic(py0_msg))
            ros_message = await afor.soft_wait_for(
                ros_sub.wait_for_new(), RECV_TIMEOUT_S
            )
            if isinstance(ros_message, TimeoutError):
                pytest.fail(
                    f"PyZeROS message not received by ROS: \nPy0: {pformat(py0_topic)}\nROS: {pformat(ros_topic)}"
                )
            publish_task.cancel()
            ros_message_as_idl = msg_type.from_ros(ros_message)
        assert ros_message_as_idl.to_core_message() == py0_msg.to_core_message()
        await asyncio.sleep(BUF_TIME)


async def test_ros_to_py0_fuzzed(ros_to_py0_pubsub: FuzzPubSub):
    msg_type, ros_topic, py0_topic, ros_pub, sub = ros_to_py0_pubsub

    async def periodic(msg: idl.IdlStruct):
        ros_pub.publish(msg.to_ros())
        rate = afor.Rate(PUBLISH_RETRY_HZ)
        try:
            async for _ in rate.listen():
                ros_pub.publish(msg.to_ros())
        finally:
            rate.close()

    for py0_msg in fuzzed_messages(msg_type):
        async with TaskGroup() as tg:
            publish_task = tg.create_task(periodic(py0_msg))
            recv_message = await afor.soft_wait_for(sub.wait_for_new(), RECV_TIMEOUT_S)
            if isinstance(recv_message, TimeoutError):
                pytest.fail(
                    f"PyZeROS message not received by ROS: \nPy0: {pformat(py0_topic)}\nROS: {pformat(ros_topic)}"
                )
            publish_task.cancel()
        assert recv_message.to_core_message() == py0_msg.to_core_message()
        await asyncio.sleep(BUF_TIME)


async def test_py0_to_py0_fuzzed(py0_to_py0_pubsub: FuzzPubSub):
    msg_type, _, py0_topic, pub, sub = py0_to_py0_pubsub

    async def periodic(msg: idl.IdlStruct):
        pub.publish(msg)
        rate = afor.Rate(PUBLISH_RETRY_HZ)
        try:
            async for _ in rate.listen():
                pub.publish(msg)
        finally:
            rate.close()

    for py0_msg in fuzzed_messages(msg_type):
        async with TaskGroup() as tg:
            publish_task = tg.create_task(periodic(py0_msg))
            recv_message = await afor.soft_wait_for(sub.wait_for_new(), RECV_TIMEOUT_S)
            if isinstance(recv_message, TimeoutError):
                pytest.fail(
                    f"PyZeROS message not received by PyZeROS: \nPy0: {pformat(py0_topic)}"
                )
            publish_task.cancel()
        assert recv_message.to_core_message() == py0_msg.to_core_message()
        await asyncio.sleep(BUF_TIME)
