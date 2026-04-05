import asyncio
from dataclasses import dataclass

import asyncio_for_robotics as afor
from ros2_pyterfaces.cyclone.all_msgs import String
from ros2_pyterfaces.cyclone.idl import IdlStruct

from pyzeros.pub import Pub
from pyzeros.sub import Sub


@dataclass
class MyCustomString(IdlStruct, typename="std_msgs/msg/String"):
    """ros2_pyterfaces allow us to re-create any ROS message.
    We could have also used
        `from ros2_pyterfaces.cyclone.all_msgs import String`
    """

    data: str = ""


async def repeat_task():
    """Listen on a topic, repeats on another"""
    # PyZeROS2 will translate the ros2_pyterfaces message into ros-z format,
    # and declare publisher / subscriber .
    sub = Sub(MyCustomString, "/listener")
    pub = Pub(MyCustomString, "/repeater")
    # asyncio_for_robotics allows us to use ayncio syntax.
    # here we iterate every incomming messages
    async for msg in sub.listen_reliable():
        pub.publish(MyCustomString(data=f"repeating: {msg.data}"))


async def pub_task():
    """Just a publisher publishing at a constant rate"""
    pub = Pub(MyCustomString, "/listener")
    # asyncio_for_robotics constant rate, like a ros timer
    async for t_ns in afor.Rate(1).listen():
        pub.publish(MyCustomString("Hello World!"))


async def main():
    # with asyncio can register several tasks that execute concurently
    async with asyncio.TaskGroup() as tg:
        tg.create_task(pub_task())
        tg.create_task(repeat_task())
        print("Listen to me using: ")
        print("""pixi run -e ros ros2 topic echo "/repeater" std_msgs/msg/String""")


if __name__ == "__main__":
    asyncio.run(main())
