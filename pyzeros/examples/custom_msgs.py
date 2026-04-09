"""Example using a message class defined locally in Python.

The file defines `MyCustomType` as an `idl.IdlStruct` subclass and uses it with
the same `Sub` / `Pub` API as any other message class.
"""

import asyncio
import time
from contextlib import suppress
from dataclasses import dataclass, field
from pprint import pprint

import asyncio_for_robotics as afor
from numpy import floor
from ros2_pyterfaces.cyclone import all_msgs, idl

from pyzeros.node import Node


@dataclass
class MyCustomType(idl.IdlStruct, typename="sensor_msgs/msg/JointState"):
    """Local Python definition of `sensor_msgs/msg/JointState`."""

    header: all_msgs.Header = field(default_factory=all_msgs.Header)
    name: idl.types.sequence[str] = field(default_factory=list)
    position: idl.types.sequence[idl.types.float64] = field(default_factory=list)
    velocity: idl.types.sequence[idl.types.float64] = field(default_factory=list)
    effort: idl.types.sequence[idl.types.float64] = field(default_factory=list)


async def sub_task(node: Node):
    """Subscribe to `/custom_msg` and print decoded `MyCustomType` messages."""
    sub = node.create_subscriber(MyCustomType, "custom_msg")
    async for msg in sub.listen_reliable():
        print(f"Sub on topic [{sub.topic_info.topic}] received:")
        pprint(msg)


async def pub_task(node: Node):
    """Publish a `MyCustomType` sample periodically."""
    pub = node.create_publisher(MyCustomType, "custom_msg")
    async for t_ns in afor.Rate(2).listen():
        t = time.time()
        now = all_msgs.Time(sec=int(floor(t)), nanosec=int((t - floor(t)) * 1e9))
        pub.publish(
            MyCustomType(
                header=all_msgs.Header(stamp=now),
                name=["joint_1", "joint_2"],
                position=[1.0, 2.0],
            )
        )


async def main():
    """Run the publisher and subscriber tasks together."""
    async with asyncio.TaskGroup() as tg:
        node = Node(name="custom_msgs", namespace="/pyzeros")
        tg.create_task(pub_task(node))
        tg.create_task(sub_task(node))


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
