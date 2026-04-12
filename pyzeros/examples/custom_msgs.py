"""Example using a message class defined locally in Python."""

import asyncio
import time
from contextlib import suppress
from dataclasses import dataclass, field
from pprint import pprint

import asyncio_for_robotics as afor
import pyzeros
from numpy import floor
from ros2_pyterfaces.cyclone import all_msgs, idl


@dataclass
class MyCustomType(idl.IdlStruct, typename="sensor_msgs/msg/JointState"):
    """Local Python definition of `sensor_msgs/msg/JointState`."""

    header: all_msgs.Header = field(default_factory=all_msgs.Header)
    name: idl.types.sequence[str] = field(default_factory=list)
    position: idl.types.sequence[idl.types.float64] = field(default_factory=list)
    velocity: idl.types.sequence[idl.types.float64] = field(default_factory=list)
    effort: idl.types.sequence[idl.types.float64] = field(default_factory=list)


def make_sample() -> MyCustomType:
    """Build one sample payload for the custom message example."""
    t = time.time()
    now = all_msgs.Time(sec=int(floor(t)), nanosec=int((t - floor(t)) * 1e9))
    return MyCustomType(
        header=all_msgs.Header(stamp=now),
        name=["joint_1", "joint_2"],
        position=[1.0, 2.0],
    )


async def sub_task() -> None:
    """Subscribe to `custom_msg` and print decoded `MyCustomType` messages."""
    sub = pyzeros.Sub(MyCustomType, "custom_msg")
    async for msg in sub.listen_reliable():
        print(f"Sub on topic [{sub.topic_info.topic}] received:")
        pprint(msg)


async def pub_task() -> None:
    """Publish a `MyCustomType` sample periodically."""
    pub = pyzeros.Pub(MyCustomType, "custom_msg")
    async for _ in afor.Rate(2).listen():
        pub.publish(make_sample())


@afor.scoped
async def main() -> None:
    """Run the publisher and subscriber tasks together."""
    tg = afor.Scope.current().task_group
    tg.create_task(pub_task())
    tg.create_task(sub_task())
    await asyncio.Future()


if __name__ == "__main__":
    with pyzeros.auto_context(node="custom_msgs", namespace="/pyzeros"):
        with suppress(KeyboardInterrupt):
            asyncio.run(main())
