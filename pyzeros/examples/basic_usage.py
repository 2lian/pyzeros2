"""Basic repeater example using one lexical PyZeROS session."""

import asyncio
from contextlib import suppress
from dataclasses import dataclass

import asyncio_for_robotics as afor
from ros2_pyterfaces.cyclone.idl import IdlStruct

import pyzeros


@dataclass
class MyCustomString(IdlStruct, typename="std_msgs/msg/String"):
    """Python-defined message mirroring a ROS standard message.

    "std_msgs/msg/String" is already available in ros2_pyterfaces.cyclone.all_msgs
    """

    data: str = ""


async def repeat_task() -> None:
    """Listen on `listener` and publishes to `repeater`."""
    sub = pyzeros.Sub(MyCustomString, "listener")
    pub = pyzeros.Pub(MyCustomString, "repeater")
    print("Listen to me using:")
    print(
        f'pixi run -e ros ros2 topic echo "{pub.fully_qualified_name}" {pub.topic_info.msg_type.get_type_name()}'
    )
    async for msg in sub.listen_reliable():
        pub.publish(MyCustomString(data=f"repeating: {msg.data}"))


async def pub_task() -> None:
    """Publish on `listener` at a constant rate."""
    pub = pyzeros.Pub(MyCustomString, "listener")
    counter = 0
    async for _ in afor.Rate(1).listen():
        pub.publish(MyCustomString(f"Hello World! #{counter}"))
        counter += 1


# all afor ressources (pub/sub/srv/rate) and tasks created inside this scope
# will be destroyed automatically on scope exit (the end of the function)
@afor.scoped
async def main() -> None:
    """Run the infinite publisher + repeater example."""
    tg = afor.Scope.current().task_group
    tg.create_task(pub_task())
    tg.create_task(repeat_task())
    # we block indefinitely to let our tasks above run indefinitely
    await asyncio.Future()


if __name__ == "__main__":
    # This creates our pyzeros session ( = zenoh session + ROS node + more)
    # inside this context, this pyzeros session becomes the default
    with pyzeros.auto_context(node="my_node", namespace="/pyzeros"):
        with suppress(KeyboardInterrupt):
            asyncio.run(main())
