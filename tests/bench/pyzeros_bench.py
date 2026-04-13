import asyncio
from contextlib import suppress
from typing import Any, ClassVar, Literal

import asyncio_for_robotics as afor
import cydr._runtime as cy_impl
import msgspec
import numpy as np
import ros2_pyterfaces.cydr.all_msgs as cydr_msgs
import uvloop
from base import MyNode
from nptyping import Bytes, Float64, NDArray, Shape
from pyinstrument import Profiler
from rclpy.serialization import deserialize_message, serialize_message
from ros2_pyterfaces.cyclone.all_msgs import Header, JointState
from ros2_pyterfaces.cyclone.all_msgs import String as IdlString
from ros2_pyterfaces.cyclone.all_msgs import Time
from ros2_pyterfaces.cydr.idl import IdlStruct as CydrStruct
from ros2_pyterfaces.cydr.idl import StringCollectionMode
from sensor_msgs.msg import JointState as RosJointState

import pyzeros
from pyzeros.utils import TopicInfo

cy_impl.DEFAULT_STRING_COLLECTION_MODE = StringCollectionMode.NUMPY
mul = 3

cyclone_msg: JointState = JointState(
    header=Header(
        stamp=Time(sec=17000, nanosec=1234),
        frame_id="base_link",
    ),
    name=["joint_a", "joint_b", "joint_c"] * mul,
    position=[0.5, 1.5, 2.5] * mul,
    velocity=[0.5, 1.5, 2.5] * mul,
    effort=[0.5, 1.5, 2.5] * mul,
)


cydr_msgs.JointState.brew()

jit_msg = cydr_msgs.JointState(
    header=cydr_msgs.Header(
        stamp=cydr_msgs.Time(sec=np.int32(17000), nanosec=np.uint32(1234)),
        frame_id=b"base_link",
    ),
    name=np.array([b"joint_a", b"joint_b", b"joint_c"] * mul, dtype=np.bytes_),
    position=np.array([0.5, 1.5, 2.5] * mul, dtype=np.float64),
    velocity=np.array([0.5, 1.5, 2.5] * mul, dtype=np.float64),
    effort=np.array([0.5, 1.5, 2.5] * mul, dtype=np.float64),
)

raw = bool(0)

case = 1
if case == 0:
    rust_mode = False
    my_msg = cyclone_msg
    my_type = JointState
    TOPIC_HELLO = TopicInfo(msg_type=my_type, topic="hello")
    TOPIC_WORLD = TopicInfo(msg_type=my_type, topic="world")
elif case == 1:
    rust_mode = False
    my_msg = jit_msg
    print(cydr_msgs.JointState.deserialize(my_msg.serialize()))
    print(cy_impl.DEFAULT_STRING_COLLECTION_MODE.name)
    my_type = cydr_msgs.JointState
    TOPIC_HELLO = TopicInfo(msg_type=my_type, topic="hello")
    TOPIC_WORLD = TopicInfo(msg_type=my_type, topic="world")
    my_type.brew()
else:
    raise

try:
    print(len(my_msg.serialize()))
except:
    print("cannot estimate size")


if raw:
    payload = my_msg.serialize()
    # payload =serialize_message(my_msg)
else:
    payload = my_msg


class PyZNode(MyNode):
    def __init__(self):
        super().__init__()

    def world_send(self):
        # my_msg.serialize()
        # np.zeros(300, dtype=float)
        # serialize_message(my_msg)
        self.world_pub.publish(payload)

    def hello_send(self):
        # my_msg.serialize()
        # np.zeros(300, dtype=float)
        # serialize_message(my_msg)
        self.hello_pub.publish(payload)

    def finish(self):
        self.fut.set_result(None)

    async def w_iter(self):
        async for msg in self.world_sub.listen_reliable():
            # n = msg.name[0]
            # p = msg.position[0]
            self.world_cbk(msg)

    async def h_iter(self):
        async for msg in self.hello_sub.listen_reliable():
            # n = msg.name[0]
            # p = msg.position[0]
            self.hello_cbk(msg)

    async def task(self, profiler=None):
        if profiler is None:
            profiler = suppress(KeyboardInterrupt)
        async with afor.Scope() as scope:
            tg = scope.task_group
            with profiler:
                self.hello_pub = pyzeros.Pub(*TOPIC_HELLO.as_arg())
                self.world_pub = pyzeros.Pub(*TOPIC_WORLD.as_arg())
                self.hello_sub = pyzeros.Sub(*TOPIC_HELLO.as_arg())
                self.world_sub = pyzeros.Sub(*TOPIC_WORLD.as_arg())
                self.fut = asyncio.Future()
                tg.create_task(self.w_iter())
                tg.create_task(self.h_iter())
                await asyncio.sleep(0.1)
                self.hello_send()
                await asyncio.sleep(60)
            self.results()

    def run(self):
        with pyzeros.auto_context():
            # asyncio.run(self.task())
            uvloop.run(self.task())
            # profiler = Profiler(async_mode="enabled", interval=0.000_001)
            # uvloop.run(self.task(profiler))
            # profiler.print(show_all=True)


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        n = PyZNode()
        n.run()
