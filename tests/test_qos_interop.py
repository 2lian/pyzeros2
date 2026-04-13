import asyncio
import threading
import time
from dataclasses import dataclass

import asyncio_for_robotics as afor
import asyncio_for_robotics.ros2 as afor_ros
import pytest
import rclpy.duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as RosNode
from rclpy.qos import (
    DurabilityPolicy as RosDurabilityPolicy,
    HistoryPolicy as RosHistoryPolicy,
    LivelinessPolicy as RosLivelinessPolicy,
    QoSProfile as RosQosProfile,
    ReliabilityPolicy as RosReliabilityPolicy,
)
from ros2_pyterfaces.cyclone import all_msgs

from pyzeros.node import Node
from pyzeros.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QosProfile,
    ReliabilityPolicy,
)
from pyzeros.session import session_context

RECV_TIMEOUT_S = 2
PUBLISH_RETRY_HZ = 50
STRESS_MESSAGE_COUNT = 4000
STRESS_PAYLOAD_BYTES = 16_384
STRESS_SUBSCRIBER_DELAY_S = 0.002
STRESS_WAIT_S = 12


@dataclass(frozen=True, slots=True)
class QosInteropCase:
    name: str
    pyzeros_qos: QosProfile
    ros_qos: RosQosProfile


def _to_ros_qos(qos: QosProfile) -> RosQosProfile:
    qos = qos.normalized()

    ros_history = {
        HistoryPolicy.KEEP_LAST: RosHistoryPolicy.KEEP_LAST,
        HistoryPolicy.KEEP_ALL: RosHistoryPolicy.KEEP_ALL,
    }[qos.history]
    ros_reliability = {
        ReliabilityPolicy.RELIABLE: RosReliabilityPolicy.RELIABLE,
        ReliabilityPolicy.BEST_EFFORT: RosReliabilityPolicy.BEST_EFFORT,
    }[qos.reliability]
    ros_durability = {
        DurabilityPolicy.VOLATILE: RosDurabilityPolicy.VOLATILE,
    }[qos.durability]
    if qos.liveliness == LivelinessPolicy.AUTOMATIC:
        ros_liveliness = RosLivelinessPolicy.AUTOMATIC
    elif qos.liveliness == LivelinessPolicy.MANUAL_BY_TOPIC:
        ros_liveliness = RosLivelinessPolicy.MANUAL_BY_TOPIC
    elif qos.liveliness == LivelinessPolicy.MANUAL_BY_NODE:
        ros_liveliness = getattr(
            RosLivelinessPolicy, "MANUAL_BY_NODE", RosLivelinessPolicy.MANUAL_BY_TOPIC
        )
    else:
        raise ValueError(f"Unsupported liveliness policy for ROS conversion: {qos.liveliness}")
    return RosQosProfile(
        history=ros_history,
        depth=qos.depth,
        reliability=ros_reliability,
        durability=ros_durability,
        liveliness=ros_liveliness,
        deadline=rclpy.duration.Duration(
            seconds=qos.deadline.sec, nanoseconds=qos.deadline.nsec
        ),
        lifespan=rclpy.duration.Duration(
            seconds=qos.lifespan.sec, nanoseconds=qos.lifespan.nsec
        ),
        liveliness_lease_duration=rclpy.duration.Duration(
            seconds=qos.liveliness_lease_duration.sec,
            nanoseconds=qos.liveliness_lease_duration.nsec,
        ),
    )


CASES = [
    QosInteropCase("default", QosProfile.default(), _to_ros_qos(QosProfile.default())),
    QosInteropCase(
        "sensor_data", QosProfile.sensor_data(), _to_ros_qos(QosProfile.sensor_data())
    ),
    QosInteropCase(
        "reliable_keep_all",
        QosProfile(history=HistoryPolicy.KEEP_ALL),
        _to_ros_qos(QosProfile(history=HistoryPolicy.KEEP_ALL)),
    ),
]

async def _publish_pyzeros_until_cancelled(pub, payload: str) -> None:
    msg = all_msgs.String(data=payload)
    pub.publish(msg)
    rate = afor_ros.Rate(PUBLISH_RETRY_HZ)
    try:
        async for _ in rate.listen():
            pub.publish(msg)
    finally:
        rate.close()


async def _publish_ros_until_cancelled(ros_pub, payload: str) -> None:
    msg = all_msgs.String(data=payload).to_ros()
    ros_pub.publish(msg)
    rate = afor_ros.Rate(PUBLISH_RETRY_HZ)
    try:
        async for _ in rate.listen():
            ros_pub.publish(msg)
    finally:
        rate.close()


@pytest.mark.asyncio(loop_scope="module")
@pytest.mark.parametrize("case", CASES, ids=lambda case: case.name)
async def test_pyzeros_publisher_reaches_ros_subscriber_with_matching_qos(
    rclpy_init, case: QosInteropCase
) -> None:
    topic = f"/tests/qos_interop/py_to_ros/{case.name}"
    payload = f"pyzeros->{case.name}"

    with session_context(Node(name=f"py_to_ros_{case.name}", namespace="/")) as node:
        async with afor.Scope() as scope:
            ros_sub = afor_ros.Sub(all_msgs.String.to_ros_type(), topic, case.ros_qos)
            pub = node.create_publisher(all_msgs.String, topic, qos_profile=case.pyzeros_qos)
            tg = scope.task_group
            publish_task = tg.create_task(_publish_pyzeros_until_cancelled(pub, payload))
            ros_message = await afor_ros.soft_wait_for(ros_sub.wait_for_new(), RECV_TIMEOUT_S)
            if isinstance(ros_message, TimeoutError):
                pytest.fail(f"PyZeROS publisher did not reach ROS subscriber for {case.name}")
            publish_task.cancel()
            assert ros_message.data == payload


@pytest.mark.asyncio(loop_scope="module")
@pytest.mark.parametrize("case", CASES, ids=lambda case: case.name)
async def test_ros_publisher_reaches_pyzeros_subscriber_with_matching_qos(
    rclpy_init, case: QosInteropCase
) -> None:
    topic = f"/tests/qos_interop/ros_to_py/{case.name}"
    payload = f"ros->{case.name}"

    with afor_ros.auto_session().lock() as ros_node:
        ros_pub = ros_node.create_publisher(
            all_msgs.String.to_ros_type(), topic, case.ros_qos,
        )
    try:
        with session_context(Node(name=f"ros_to_py_{case.name}", namespace="/")) as node:
            async with afor.Scope() as scope:
                sub = node.create_subscriber(all_msgs.String, topic, qos_profile=case.pyzeros_qos)
                tg = scope.task_group
                publish_task = tg.create_task(_publish_ros_until_cancelled(ros_pub, payload))
                recv_message = await afor_ros.soft_wait_for(sub.wait_for_new(), RECV_TIMEOUT_S)
                if isinstance(recv_message, TimeoutError):
                    pytest.fail(f"ROS publisher did not reach PyZeROS subscriber for {case.name}")
                publish_task.cancel()
                assert recv_message.data == payload
    finally:
        with afor_ros.auto_session().lock() as ros_node:
            ros_node.destroy_publisher(ros_pub)


def _count_ros_messages_from_pyzeros(
    pyzeros_qos: QosProfile,
    ros_qos: RosQosProfile,
    *,
    topic: str,
    subscriber_delay_s: float,
    message_count: int,
    payload_bytes: int,
) -> tuple[int, int]:
    received: list[int] = []
    lock = threading.Lock()

    ros_node = RosNode(f"stress_sub_{int(time.time() * 1_000_000)}")
    executor = SingleThreadedExecutor(context=ros_node.context)
    executor.add_node(ros_node)

    def callback(msg) -> None:
        with lock:
            received.append(int(msg.data.split(":", 1)[0]))
        if subscriber_delay_s > 0:
            time.sleep(subscriber_delay_s)

    sub = ros_node.create_subscription(all_msgs.String.to_ros_type(), topic, callback, ros_qos)
    stop_evt = threading.Event()

    def spin() -> None:
        while not stop_evt.is_set():
            executor.spin_once(timeout_sec=0.05)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()

    with session_context(Node(name="stress_pub", namespace="/")) as node:
        pub = node.create_publisher(all_msgs.String, topic, qos_profile=pyzeros_qos)
        payload = "x" * payload_bytes

        try:
            time.sleep(0.3)
            for index in range(message_count):
                pub.publish(all_msgs.String(data=f"{index}:{payload}"))

            deadline = time.time() + STRESS_WAIT_S
            previous_count = -1
            stable_polls = 0
            while time.time() < deadline:
                with lock:
                    current_count = len(received)
                if current_count == previous_count:
                    stable_polls += 1
                else:
                    stable_polls = 0
                    previous_count = current_count
                if current_count >= message_count or stable_polls >= 10:
                    break
                time.sleep(0.1)

            with lock:
                unique_messages = len(set(received))
                total_messages = len(received)
            return total_messages, unique_messages
        finally:
            pub.undeclare()
            stop_evt.set()
            thread.join(timeout=1)
            executor.remove_node(ros_node)
            executor.shutdown(timeout_sec=0)
            ros_node.destroy_subscription(sub)
            ros_node.destroy_node()


def test_reliable_stress_interop_delivers_full_burst(rclpy_init) -> None:
    qos = QosProfile(history=HistoryPolicy.KEEP_ALL, reliability=ReliabilityPolicy.RELIABLE)
    total_messages, unique_messages = _count_ros_messages_from_pyzeros(
        qos,
        _to_ros_qos(qos),
        topic="/tests/qos_interop/stress/reliable",
        subscriber_delay_s=STRESS_SUBSCRIBER_DELAY_S,
        message_count=STRESS_MESSAGE_COUNT,
        payload_bytes=STRESS_PAYLOAD_BYTES,
    )
    assert total_messages == STRESS_MESSAGE_COUNT
    assert unique_messages == STRESS_MESSAGE_COUNT


def test_best_effort_stress_interop_can_drop_messages(rclpy_init) -> None:
    qos = QosProfile(history=HistoryPolicy.KEEP_ALL, reliability=ReliabilityPolicy.BEST_EFFORT)
    total_messages, unique_messages = _count_ros_messages_from_pyzeros(
        qos,
        _to_ros_qos(qos),
        topic="/tests/qos_interop/stress/best_effort",
        subscriber_delay_s=STRESS_SUBSCRIBER_DELAY_S,
        message_count=STRESS_MESSAGE_COUNT,
        payload_bytes=STRESS_PAYLOAD_BYTES,
    )
    if unique_messages == STRESS_MESSAGE_COUNT:
        pytest.skip("No best-effort drops observed on this localhost setup under stress.")
    assert total_messages < STRESS_MESSAGE_COUNT
    assert unique_messages < STRESS_MESSAGE_COUNT
