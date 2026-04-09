import asyncio
from asyncio import TaskGroup
from dataclasses import dataclass

import asyncio_for_robotics.ros2 as afor
import pytest
from ros2_pyterfaces.cyclone import all_msgs, all_srvs

from pyzeros.node import Node, token_keyexpr as node_token_keyexpr
from pyzeros.pub import token_keyexpr as publisher_token_keyexpr
from pyzeros.service_common import token_keyexpr as service_token_keyexpr
from pyzeros.service_client import Client
from pyzeros.service_server import Server
from pyzeros.sub import token_keyexpr as subscriber_token_keyexpr

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
    TopicCase("%", "chatter", "/chatter"),
    TopicCase("", "/chatter", "/chatter"),
    TopicCase("/", "/chatter", "/chatter"),
    TopicCase("%", "/chatter", "/chatter"),
    TopicCase("", "/chatter/", "/chatter"),
    TopicCase("/", "/chatter/", "/chatter"),
    TopicCase("%", "/chatter/", "/chatter"),
    TopicCase("", "chatter/chatter", "/chatter/chatter"),
    TopicCase("/", "chatter/chatter", "/chatter/chatter"),
    TopicCase("%", "chatter/chatter", "/chatter/chatter"),
    TopicCase("tests/no_leading", "chatter", "/tests/no_leading/chatter"),
    TopicCase("/tests/ns_rel", "chatter", "/tests/ns_rel/chatter"),
    TopicCase("/tests/ns_nested/", "nested/topic", "/tests/ns_nested/nested/topic"),
    TopicCase("/tests/trailing/", "chatter", "/tests/trailing/chatter"),
    TopicCase("/tests/ns_abs", "/absolute/topic", "/absolute/topic"),
]


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


@pytest.mark.parametrize(
    "case", CASES, ids=lambda case: f"{case.namespace}:{case.topic}"
)
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
            ros_message = await afor.soft_wait_for(
                ros_sub.wait_for_new(), RECV_TIMEOUT_S
            )
            if isinstance(ros_message, TimeoutError):
                pytest.fail(
                    f"PyZeROS publisher did not reach ROS topic {case.expected_topic}"
                )
            publish_task.cancel()
        assert ros_message.data == payload
    finally:
        ros_sub.close()
        pub.undeclare()
        node.undeclare()


@pytest.mark.parametrize(
    "case", CASES, ids=lambda case: f"{case.namespace}:{case.topic}"
)
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
            publish_task = tg.create_task(
                _publish_ros_until_cancelled(ros_pub, payload)
            )
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


async def test_root_namespace_token_mangling_does_not_duplicate_root_prefix() -> None:
    token = publisher_token_keyexpr(
        name="leg1/joint_read",
        dds_type="sensor_msgs::msg::dds_::JointState_",
        hash="RIHS01_deadbeef",
        node_name="moonbot_zero",
        domain_id=0,
        namespace="%",
        _enclave="%",
        _node_id=0,
        _zenoh_id="zid",
        _entity_id=1,
    )

    assert "/%/%/moonbot_zero/%leg1%joint_read/" in token
    assert "/%/%/moonbot_zero/%%leg1%joint_read/" not in token


@pytest.mark.parametrize(
    ("label", "token"),
    [
        (
            "node",
            node_token_keyexpr(
                name="moonbot_zero",
                domain_id=0,
                namespace="%",
                _enclave="%",
                _node_id=0,
                _zenoh_id="zid",
                _entity_id=0,
            ),
        ),
        (
            "publisher",
            publisher_token_keyexpr(
                name="leg1/joint_read",
                dds_type="sensor_msgs::msg::dds_::JointState_",
                hash="RIHS01_deadbeef",
                node_name="moonbot_zero",
                domain_id=0,
                namespace="%",
                _enclave="%",
                _node_id=0,
                _zenoh_id="zid",
                _entity_id=1,
            ),
        ),
        (
            "subscriber",
            subscriber_token_keyexpr(
                name="leg1/joint_read",
                dds_type="sensor_msgs::msg::dds_::JointState_",
                hash="RIHS01_deadbeef",
                node_name="moonbot_zero",
                domain_id=0,
                namespace="%",
                _enclave="%",
                _node_id=0,
                _zenoh_id="zid",
                _entity_id=2,
            ),
        ),
        (
            "service_client",
            service_token_keyexpr(
                "SC",
                name="/leg1/joint_read",
                dds_type="example_interfaces::srv::dds_::Trigger_",
                hash="RIHS01_deadbeef",
                node_name="moonbot_zero",
                domain_id=0,
                namespace="%",
                _enclave="%",
                _node_id=0,
                _zenoh_id="zid",
                _entity_id=3,
            ),
        ),
        (
            "service_server",
            service_token_keyexpr(
                "SS",
                name="/leg1/joint_read",
                dds_type="example_interfaces::srv::dds_::Trigger_",
                hash="RIHS01_deadbeef",
                node_name="moonbot_zero",
                domain_id=0,
                namespace="%",
                _enclave="%",
                _node_id=0,
                _zenoh_id="zid",
                _entity_id=4,
            ),
        ),
    ],
)
async def test_root_namespace_token_builders_normalize_percent_namespace(
    label: str, token: str
) -> None:
    assert "/%/%/" in token, label
    assert "/%%" not in token, label


async def test_root_namespace_node_children_have_correct_tokens() -> None:
    node = Node(name="moonbot_zero", namespace="%")
    pub = node.create_publisher(all_msgs.JointState, "leg1/joint_read", defer=True)
    sub = node.create_subscriber(all_msgs.JointState, "leg1/joint_read", defer=True)
    client = node.create_client(all_srvs.Trigger, "leg1/joint_read", defer=True)
    server = node.create_service(all_srvs.Trigger, "leg1/joint_read", defer=True)

    try:
        assert node.token_keyexpr.endswith("/NN/%/%/moonbot_zero")
        assert "/%/%/moonbot_zero/%leg1%joint_read/" in pub.token_keyexpr
        assert "/%/%/moonbot_zero/%%leg1%joint_read/" not in pub.token_keyexpr
        assert "/%/%/moonbot_zero/%leg1%joint_read/" in sub.raw_sub.token_keyexpr
        assert "/%/%/moonbot_zero/%%leg1%joint_read/" not in sub.raw_sub.token_keyexpr
        assert "/%/%/moonbot_zero/%leg1%joint_read/" in client.token_keyexpr
        assert "/%/%/moonbot_zero/%%leg1%joint_read/" not in client.token_keyexpr
        assert "/%/%/moonbot_zero/%leg1%joint_read/" in server.token_keyexpr
        assert "/%/%/moonbot_zero/%%leg1%joint_read/" not in server.token_keyexpr
    finally:
        pub.undeclare()
        sub.raw_sub.undeclare()
        sub.close()
        client.close()
        server.close()
        node.undeclare()


@pytest.mark.parametrize(
    ("label", "token"),
    [
        (
            "node",
            node_token_keyexpr(
                name="moonbot_zero",
                domain_id=0,
                namespace="leg1",
                _enclave="%",
                _node_id=0,
                _zenoh_id="zid",
                _entity_id=0,
            ),
        ),
        (
            "publisher",
            publisher_token_keyexpr(
                name="joint_read",
                dds_type="sensor_msgs::msg::dds_::JointState_",
                hash="RIHS01_deadbeef",
                node_name="moonbot_zero",
                domain_id=0,
                namespace="leg1",
                _enclave="%",
                _node_id=0,
                _zenoh_id="zid",
                _entity_id=1,
            ),
        ),
        (
            "subscriber",
            subscriber_token_keyexpr(
                name="joint_read",
                dds_type="sensor_msgs::msg::dds_::JointState_",
                hash="RIHS01_deadbeef",
                node_name="moonbot_zero",
                domain_id=0,
                namespace="leg1",
                _enclave="%",
                _node_id=0,
                _zenoh_id="zid",
                _entity_id=2,
            ),
        ),
        (
            "service_client",
            service_token_keyexpr(
                "SC",
                name="/leg1/joint_read",
                dds_type="example_interfaces::srv::dds_::Trigger_",
                hash="RIHS01_deadbeef",
                node_name="moonbot_zero",
                domain_id=0,
                namespace="leg1",
                _enclave="%",
                _node_id=0,
                _zenoh_id="zid",
                _entity_id=3,
            ),
        ),
        (
            "service_server",
            service_token_keyexpr(
                "SS",
                name="/leg1/joint_read",
                dds_type="example_interfaces::srv::dds_::Trigger_",
                hash="RIHS01_deadbeef",
                node_name="moonbot_zero",
                domain_id=0,
                namespace="leg1",
                _enclave="%",
                _node_id=0,
                _zenoh_id="zid",
                _entity_id=4,
            ),
        ),
    ],
)
async def test_relative_topic_tokens_keep_a_leading_mangled_slash_under_namespace(
    label: str, token: str
) -> None:
    if label == "node":
        assert token.endswith("/NN/%/%leg1/moonbot_zero"), label
        return
    if label in {"publisher", "subscriber"}:
        assert "/%leg1/moonbot_zero/%leg1%joint_read/" in token, label
        assert "/%leg1/moonbot_zero/leg1%joint_read/" not in token, label
        return

    assert "/%/%/moonbot_zero/%leg1%joint_read/" in token, label
    assert "/%/%/moonbot_zero/leg1%joint_read/" not in token, label


async def test_namespaced_node_children_have_fully_qualified_tokens() -> None:
    node = Node(name="moonbot_zero", namespace="leg1")
    pub = node.create_publisher(all_msgs.JointState, "joint_read", defer=True)
    sub = node.create_subscriber(all_msgs.JointState, "joint_read", defer=True)
    client = node.create_client(all_srvs.Trigger, "joint_read", defer=True)
    server = node.create_service(all_srvs.Trigger, "joint_read", defer=True)

    try:
        assert node.token_keyexpr.endswith("/NN/%/%leg1/moonbot_zero")
        assert "/%leg1/moonbot_zero/%leg1%joint_read/" in pub.token_keyexpr
        assert "/%leg1/moonbot_zero/leg1%joint_read/" not in pub.token_keyexpr
        assert "/%leg1/moonbot_zero/%leg1%joint_read/" in sub.raw_sub.token_keyexpr
        assert "/%leg1/moonbot_zero/leg1%joint_read/" not in sub.raw_sub.token_keyexpr
        assert "/%/%/moonbot_zero/%leg1%joint_read/" in client.token_keyexpr
        assert "/%/%/moonbot_zero/leg1%joint_read/" not in client.token_keyexpr
        assert "/%/%/moonbot_zero/%leg1%joint_read/" in server.token_keyexpr
        assert "/%/%/moonbot_zero/leg1%joint_read/" not in server.token_keyexpr
    finally:
        pub.undeclare()
        sub.raw_sub.undeclare()
        sub.close()
        client.close()
        server.close()
        node.undeclare()
