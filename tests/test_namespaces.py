import asyncio
import subprocess
import time
import uuid
from collections import Counter
from dataclasses import dataclass

import asyncio_for_robotics as afor
import pytest
from ros2_pyterfaces.cyclone import all_msgs, all_srvs

from pyzeros.node import Node
from pyzeros.session import session_context

DISCOVERY_TIMEOUT_S = 5.0
DISCOVERY_POLL_S = 0.3

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


def _tag(topic: str, tag: str) -> str:
    """Append *tag* to a topic name, preserving leading/trailing slashes."""
    trail = "/" if len(topic) > 1 and topic.endswith("/") else ""
    return f"{topic.rstrip('/')}_{tag}{trail}"


def ros2(*args: str) -> list[str]:
    """Run a ros2 CLI command and return non-empty stripped output lines."""
    result = subprocess.run(
        ["ros2", *args],
        capture_output=True,
        text=True,
        timeout=10,
    )
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


async def wait_for_node(
    expected_node: str,
    timeout: float = DISCOVERY_TIMEOUT_S,
    interval: float = DISCOVERY_POLL_S,
) -> list[str]:
    """Poll ``ros2 node list`` until *expected_node* appears or timeout."""
    deadline = time.monotonic() + timeout
    while True:
        nodes = ros2("node", "list")
        if expected_node in nodes:
            return nodes
        if time.monotonic() >= deadline:
            raise AssertionError(
                f"Node {expected_node!r} not discovered within {timeout}s. "
                f"Visible nodes: {nodes}"
            )
        await asyncio.sleep(interval)


@pytest.mark.parametrize(
    "case", CASES, ids=lambda c: f"{c.namespace}:{c.topic}"
)
async def test_graph_visibility(rclpy_init, case: TopicCase):
    node_name = f"test_{uuid.uuid4().hex[:8]}"
    with session_context(Node(name=node_name, namespace=case.namespace)) as node:
        async with afor.Scope():
            pub = node.create_publisher(all_msgs.String, _tag(case.topic, "pub"))
            sub = node.create_subscriber(all_msgs.String, _tag(case.topic, "sub"))
            client = node.create_client(all_srvs.Trigger, _tag(case.topic, "cli"))
            server = node.create_service(all_srvs.Trigger, _tag(case.topic, "srv"))

            expected_node = node.fully_qualified_name
            expected_pub = f"{case.expected_topic}_pub"
            expected_sub = f"{case.expected_topic}_sub"
            expected_srv = f"{case.expected_topic}_srv"

            # -- Node: wait for discovery, then verify no duplicates --
            nodes = await wait_for_node(expected_node)

            node_counts = Counter(nodes)
            assert node_counts[expected_node] == 1, (
                f"Expected {expected_node!r} exactly once, "
                f"got {node_counts[expected_node]}: {nodes}"
            )

            # Bare /<name> must not leak as a separate entry alongside /<ns>/<name>
            bare_node = f"/{node_name}"
            if expected_node != bare_node:
                assert bare_node not in nodes, (
                    f"Bare node {bare_node!r} leaked alongside "
                    f"namespaced {expected_node!r}: {nodes}"
                )

            # No other entry should contain our unique node name in any form
            duplicates = [n for n in nodes if node_name in n and n != expected_node]
            assert duplicates == [], (
                f"Node name {node_name!r} appears in unexpected entries: {duplicates}"
            )

            # -- Topics: pub and sub independently visible, not duplicated --
            topics = ros2("topic", "list")
            topic_counts = Counter(topics)
            for label, expected in [("pub", expected_pub), ("sub", expected_sub)]:
                assert expected in topics, (
                    f"{label} topic {expected!r} missing from ros2 topic list: {topics}"
                )
                assert topic_counts[expected] == 1, (
                    f"{label} topic {expected!r} duplicated "
                    f"({topic_counts[expected]}x): {topics}"
                )

            # -- Service: server visible, not duplicated --
            services = ros2("service", "list")
            assert expected_srv in services, (
                f"Service {expected_srv!r} missing from ros2 service list: {services}"
            )
            service_counts = Counter(services)
            assert service_counts[expected_srv] == 1, (
                f"Service {expected_srv!r} duplicated "
                f"({service_counts[expected_srv]}x): {services}"
            )
