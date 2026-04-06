"""Minimal typed subscriber example using the current `pyzeros` API."""

import asyncio
from contextlib import suppress

from ros2_pyterfaces.cyclone.all_msgs import String

from pyzeros.node import Node


async def main() -> None:
    """Create one node, subscribe to `/pyzeros/chatter`, and print messages."""
    node = Node(name="listener", namespace="/pyzeros")
    sub = node.create_subscriber(String, "chatter")

    print(
        '\nTalk to me on ROS 2 using:\n'
        'pixi run -e ros ros2 topic pub "/pyzeros/chatter" '
        'std_msgs/msg/String \'{data: "Hello_World"}\'\n'
    )

    async for msg in sub.listen_reliable():
        print(msg.data)


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
