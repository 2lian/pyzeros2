import asyncio
from contextlib import suppress
from typing import Any, ClassVar, Literal

import cydr._runtime as cy_impl
import msgspec
import numpy as np
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

from pyzeros.pub import Pub
from pyzeros.sub import Sub
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


class JitTime(CydrStruct):
    __idl_typename__: ClassVar[Literal["builtin_interfaces/msg/Time"]] = (
        "builtin_interfaces/msg/Time"
    )
    sec: np.int32 = np.int32(0)
    nanosec: np.uint32 = np.uint32(0)


class JitHeader(CydrStruct):
    __idl_typename__: ClassVar[Literal["std_msgs/msg/Header"]] = "std_msgs/msg/Header"
    stamp: JitTime = msgspec.field(default_factory=JitTime)
    frame_id: bytes = b""


class JitJointState(CydrStruct):
    __idl_typename__: ClassVar[Literal["sensor_msgs/msg/JointState"]] = (
        "sensor_msgs/msg/JointState"
    )
    header: JitHeader = msgspec.field(default_factory=JitHeader)
    name: NDArray[Any, Bytes] = msgspec.field(
        default_factory=lambda: np.empty(0, Bytes)
    )
    position: NDArray[Any, Float64] = msgspec.field(
        default_factory=lambda: np.empty(0, Float64)
    )
    velocity: NDArray[Any, Float64] = msgspec.field(
        default_factory=lambda: np.empty(0, Float64)
    )
    effort: NDArray[Any, Float64] = msgspec.field(
        default_factory=lambda: np.empty(0, Float64)
    )


JitJointState.brew()

jit_msg = JitJointState(
    header=JitHeader(
        stamp=JitTime(sec=np.int32(17000), nanosec=np.uint32(1234)),
        frame_id=b"base_link",
    ),
    name=np.array([b"joint_a", b"joint_b", b"joint_c"] * mul, dtype=np.bytes_),
    position=np.array([0.5, 1.5, 2.5] * mul, dtype=np.float64),
    velocity=np.array([0.5, 1.5, 2.5] * mul, dtype=np.float64),
    effort=np.array([0.5, 1.5, 2.5] * mul, dtype=np.float64),
)

TOPIC_HELLO = TopicInfo(msg_type=JointState, topic="bench/hello_ros")
TOPIC_WORLD = TopicInfo(msg_type=JointState, topic="bench/world_ros")
raw = bool(0)

case = 1
if case == 0:
    rust_mode = False
    my_msg = cyclone_msg
    my_type = JointState
elif case == 1:
    rust_mode = False
    my_msg = jit_msg
    print(JitJointState.deserialize(my_msg.serialize()))
    print(cy_impl.DEFAULT_STRING_COLLECTION_MODE.name)
    my_type = JitJointState
elif case == 3:
    rust_mode = False
    my_msg = cyclone_msg.to_ros()
    my_type = RosJointState

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
        self.world_pub.publish(payload if raw or rust_mode else payload.serialize())

    def hello_send(self):
        # my_msg.serialize()
        # np.zeros(300, dtype=float)
        # serialize_message(my_msg)
        self.hello_pub.publish(payload if raw or rust_mode else payload.serialize())

    def finish(self):
        self.fut.set_result(None)

    async def w_iter(self):
        async for msg in self.world_sub.listen_reliable():
            if not rust_mode:
                msg = memoryview(msg) if raw else my_type.deserialize(memoryview(msg))
            if not raw:
                n = msg.name[0]
                p = msg.position[0]
            self.world_cbk(msg)

    async def h_iter(self):
        async for msg in self.hello_sub.listen_reliable():
            msg: JointState
            if not rust_mode:
                msg = memoryview(msg) if raw else my_type.deserialize(memoryview(msg))
            if not raw:
                n = msg.name[0]
                p = msg.position[0]
            self.hello_cbk(msg)

    async def task(self):
        self.hello_pub = Pub(*TOPIC_HELLO.as_arg())
        self.world_pub = Pub(*TOPIC_WORLD.as_arg())
        self.hello_sub = Sub(
            *TOPIC_HELLO.as_arg()
        )  # always raw except for not raw + rust
        self.world_sub = Sub(
            *TOPIC_WORLD.as_arg()
        )  # always raw except for not raw + rust
        self.fut = asyncio.Future()
        tasks = [
            asyncio.create_task(self.w_iter()),
            asyncio.create_task(self.h_iter()),
        ]
        await asyncio.sleep(0.1)
        self.hello_send()
        # await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        await asyncio.sleep(60)
        self.results()
        for t in tasks:
            t.cancel()

    def run(self):
        # asyncio.run(self.task())
        uvloop.run(self.task())
        # profiler = Profiler(async_mode="enabled", interval=0.000_001)
        # with profiler:
        #     uvloop.run(self.task())
        # profiler.print(show_all=False)


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        n = PyZNode()
        n.run()
