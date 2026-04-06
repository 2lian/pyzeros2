import asyncio
from asyncio import TaskGroup
from dataclasses import dataclass

import asyncio_for_robotics.ros2 as afor
import pytest
import rclpy
from ros2_pyterfaces.cyclone import all_msgs

from pyzeros.node import Node

RECV_TIMEOUT_S = 2
PUBLISH_RETRY_HZ = 50

pytestmark = pytest.mark.asyncio(loop_scope="module")


@dataclass(frozen=True, slots=True)
class TopicCase:
    namespace: str
    topic: str
    expected_topic: str


CASES = [
    TopicCase("", "chatter", "/chatter"),
    TopicCase("/", "chatter", "/chatter"),
    TopicCase("tests/no_leading", "chatter", "/tests/no_leading/chatter"),
    TopicCase("/tests/ns_rel", "chatter", "/tests/ns_rel/chatter"),
    TopicCase("/tests/ns_nested/", "nested/topic", "/tests/ns_nested/nested/topic"),
    TopicCase("/tests/trailing/", "chatter", "/tests/trailing/chatter"),
    TopicCase("/tests/ns_abs", "/absolute/topic", "/absolute/topic"),
]


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.try_shutdown()


async def _publish_py0_until_cancelled(pub, payload: str) -> None:
    msg = all_msgs.String(data=payload)
    pub.publish(msg)
    rate = afor.Rate(PUBLISH_RETRY_HZ)
    try:
        async for _ in rate.listen():
            pub.publish(msg)
    finally:
        rate.close()


async def _publish_ros_until_cancelled(ros_pub, payload: str) -> None:
    msg = all_msgs.String(data=payload).to_ros()
    ros_pub.publish(msg)
    rate = afor.Rate(PUBLISH_RETRY_HZ)
    try:
        async for _ in rate.listen():
            ros_pub.publish(msg)
    finally:
        rate.close()


@pytest.mark.parametrize("case", CASES, ids=lambda case: f"{case.namespace}:{case.topic}")
async def test_py0_publisher_resolves_to_expected_ros_topic(
    rclpy_init, case: TopicCase
) -> None:
    node = Node(name="py0_to_ros_ns", namespace=case.namespace)
    pub = node.create_publisher(all_msgs.String, case.topic)
    ros_topic = afor.TopicInfo(case.expected_topic, all_msgs.String.to_ros_type())
    ros_sub = afor.Sub(*ros_topic.as_arg())
    payload = f"py0->{case.expected_topic}"

    try:
        async with TaskGroup() as tg:
            publish_task = tg.create_task(_publish_py0_until_cancelled(pub, payload))
            ros_message = await afor.soft_wait_for(ros_sub.wait_for_new(), RECV_TIMEOUT_S)
            if isinstance(ros_message, TimeoutError):
                pytest.fail(f"PyZeROS publisher did not reach ROS topic {case.expected_topic}")
            publish_task.cancel()
        assert ros_message.data == payload
    finally:
        ros_sub.close()
        pub.undeclare()
        node.undeclare()


@pytest.mark.parametrize("case", CASES, ids=lambda case: f"{case.namespace}:{case.topic}")
async def test_ros_publisher_reaches_py0_subscriber_with_namespace_mapping(
    rclpy_init, case: TopicCase
) -> None:
    node = Node(name="ros_to_py0_ns", namespace=case.namespace)
    sub = node.create_subscriber(all_msgs.String, case.topic)
    ros_topic = afor.TopicInfo(case.expected_topic, all_msgs.String.to_ros_type())
    payload = f"ros->{case.expected_topic}"

    with afor.auto_session().lock() as ros_node:
        ros_pub = ros_node.create_publisher(*ros_topic.as_arg())
    try:
        async with TaskGroup() as tg:
            publish_task = tg.create_task(_publish_ros_until_cancelled(ros_pub, payload))
            recv_message = await afor.soft_wait_for(sub.wait_for_new(), RECV_TIMEOUT_S)
            if isinstance(recv_message, TimeoutError):
                pytest.fail(
                    f"ROS publisher on {case.expected_topic} did not reach PyZeROS subscriber"
                )
            publish_task.cancel()
        assert recv_message.data == payload
    finally:
        with afor.auto_session().lock() as ros_node:
            ros_node.destroy_publisher(ros_pub)
        sub.raw_sub.undeclare()
        sub.close()
        node.undeclare()
