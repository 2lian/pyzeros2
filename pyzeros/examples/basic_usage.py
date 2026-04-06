import asyncio
from dataclasses import dataclass

import asyncio_for_robotics as afor
from ros2_pyterfaces.cyclone.idl import IdlStruct

from pyzeros.node import Node


@dataclass
class MyCustomString(IdlStruct, typename="std_msgs/msg/String"):
    """ros2_pyterfaces allow us to re-create any ROS message.
    We could have also used
        `from ros2_pyterfaces.cyclone.all_msgs import String`
    """

    data: str = ""


async def repeat_task(node: Node):
    """Listen on a topic, repeats on another"""
    # The node manages publisher and subscriber declaration for us.
    sub = node.create_subscriber(MyCustomString, "listener")
    pub = node.create_publisher(MyCustomString, "repeater")
    # asyncio_for_robotics allows us to use ayncio syntax.
    # here we iterate every incomming messages
    async for msg in sub.listen_reliable():
        pub.publish(MyCustomString(data=f"repeating: {msg.data}"))


async def pub_task(node: Node):
    """Just a publisher publishing at a constant rate"""
    pub = node.create_publisher(MyCustomString, "listener")
    counter = 0
    # asyncio_for_robotics constant rate, like a ros timer
    async for t_ns in afor.Rate(1).listen():
        pub.publish(MyCustomString(f"Hello World! #{counter}"))
        counter += 1


async def main():
    # with asyncio can register several tasks that execute concurently
    async with asyncio.TaskGroup() as tg:
        node = Node(name="my_node", namespace="/pyzeros")
        tg.create_task(pub_task(node))
        tg.create_task(repeat_task(node))
        print("Listen to me using: ")
        print("""pixi run -e ros ros2 topic echo "/pyzeros/repeater" std_msgs/msg/String""")


if __name__ == "__main__":
    asyncio.run(main())
