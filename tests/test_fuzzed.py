from asyncio import TaskGroup
from pprint import pformat
from typing import Any, NamedTuple, Type

import asyncio_for_robotics.ros2 as afor
import pytest
import pytest_asyncio
import rclpy
from ros2_pyterfaces import idl
from test_utils import MSG_TYPES, MSG_TYPES_ids, message_to_plain_data, random_message

import pyzeros

FUZZ_MESSAGE_COUNT = 10
PUBLISH_RETRY_HZ = 100
RECV_TIMEOUT_S = 2

pytestmark = pytest.mark.asyncio(loop_scope="module")


class FuzzPubSub(NamedTuple):
    msg_type: Type[idl.IdlStruct]
    ros_topic: afor.TopicInfo
    py0_topic: pyzeros.utils.TopicInfo
    publisher: Any
    subscriber: Any


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    ses = afor.auto_session()
    yield
    ses.close()
    rclpy.try_shutdown()


@pytest.fixture(scope="module", params=MSG_TYPES, ids=MSG_TYPES_ids)
def msg_type(request) -> Type[idl.IdlStruct]:
    return request.param


def topic_name(direction: str, msg_type: Type[idl.IdlStruct]) -> str:
    return f"/tests/{direction}/{msg_type.get_type_name()}"


def fuzzed_messages(msg_type: Type[idl.IdlStruct]) -> list[idl.IdlStruct]:
    return [msg_type()] + [
        random_message(msg_type, seed=seed) for seed in range(FUZZ_MESSAGE_COUNT)
    ]


@pytest_asyncio.fixture(scope="module", loop_scope="module")
async def py0_to_ros_pubsub(rclpy_init, msg_type: Type[idl.IdlStruct]) -> FuzzPubSub:
    ros_topic = afor.TopicInfo(
        topic_name("py0_to_ros", msg_type), msg_type.to_ros_type()
    )
    py0_topic = pyzeros.utils.TopicInfo(topic_name("py0_to_ros", msg_type), msg_type)

    pub = pyzeros.pub.ZPublisher(*py0_topic.as_arg())
    ros_sub = afor.Sub(*ros_topic.as_arg())
    yield FuzzPubSub(msg_type, ros_topic, py0_topic, pub, ros_sub)
    ros_sub.close()


@pytest_asyncio.fixture(scope="module", loop_scope="module")
async def ros_to_py0_pubsub(rclpy_init, msg_type: Type[idl.IdlStruct]) -> FuzzPubSub:
    ros_topic = afor.TopicInfo(
        topic_name("ros_to_py0", msg_type), msg_type.to_ros_type()
    )
    py0_topic = pyzeros.utils.TopicInfo(topic_name("ros_to_py0", msg_type), msg_type)

    with afor.auto_session().lock() as node:
        ros_pub = node.create_publisher(*ros_topic.as_arg())
    sub = pyzeros.sub.Sub(*py0_topic.as_arg())
    yield FuzzPubSub(msg_type, ros_topic, py0_topic, ros_pub, sub)
    with afor.auto_session().lock() as node:
        node.destroy_publisher(ros_pub)
    sub.close()


@pytest_asyncio.fixture(scope="module", loop_scope="module")
async def py0_to_py0_pubsub(msg_type: Type[idl.IdlStruct]) -> FuzzPubSub:
    py0_topic = pyzeros.utils.TopicInfo(topic_name("py0_to_py0", msg_type), msg_type)

    pub = pyzeros.pub.ZPublisher(*py0_topic.as_arg())
    sub = pyzeros.sub.Sub(*py0_topic.as_arg())
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
            assert message_to_plain_data(ros_message_as_idl) == message_to_plain_data(
                py0_msg
            )


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
            assert message_to_plain_data(recv_message) == message_to_plain_data(py0_msg)
        # await asyncio.sleep(0.010)


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
            assert message_to_plain_data(recv_message) == message_to_plain_data(py0_msg)
