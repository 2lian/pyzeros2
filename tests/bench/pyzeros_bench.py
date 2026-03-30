import asyncio
from typing import Any, ClassVar, Literal

import msgspec
import numpy as np
import uvloop
from base import MyNode
from nptyping import Bytes, Float64, NDArray, Shape
from ros2_pyterfaces.cyclone.all_msgs import Header, JointState
from ros2_pyterfaces.cyclone.all_msgs import String as IdlString
from ros2_pyterfaces.cyclone.all_msgs import Time
from ros2_pyterfaces.jitcdr.idl import JitStruct
from ros_z_msgs_py.types.builtin_interfaces import Time as RustTime
from ros_z_msgs_py.types.sensor_msgs import JointState as RustJointState
from ros_z_msgs_py.types.std_msgs import Header as RustHeader
from ros_z_msgs_py.types.std_msgs import String as RustString

from pyzeros.pub import ZPublisher
from pyzeros.sub import Sub
from pyzeros.utils import TopicInfo

cyclone_msg: JointState = JointState(
    header=Header(
        stamp=Time(sec=17000, nanosec=1234),
        frame_id="base_link",
    ),
    name=["joint_a", "joint_b", "joint_c"] * 4,
    position=[0.5, 1.5, 2.5] * 4,
)


class JitTime(JitStruct):
    __idl_typename__: ClassVar[Literal["builtin_interfaces/msg/Time"]] = (
        "builtin_interfaces/msg/Time"
    )
    sec: np.int32 = np.int32(0)
    nanosec: np.uint32 = np.uint32(0)


class JitHeader(JitStruct):
    __idl_typename__: ClassVar[Literal["std_msgs/msg/Header"]] = "std_msgs/msg/Header"
    stamp: JitTime = msgspec.field(default_factory=JitTime)
    frame_id: bytes = b""


class JitJointState(JitStruct):
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


jit_msg = JitJointState(
    header=JitHeader(
        stamp=JitTime(sec=np.int32(17000), nanosec=np.uint32(1234)),
        frame_id=b"base_link",
    ),
    name=np.array([b"joint_a", b"joint_b", b"joint_c"] * 4, dtype=np.bytes_),
    position=np.array([0.5, 1.5, 2.5] * 4, dtype=np.float64),
)

rust_msg = RustJointState(
    header=RustHeader(
        stamp=RustTime(sec=17000, nanosec=1234),
        frame_id="base_link",
    ),
    name=["joint_a", "joint_b", "joint_c"] * 4,
    position=np.array([0.5, 1.5, 2.5] * 4, dtype=np.float64),
)

TOPIC_HELLO = TopicInfo(msg_type=JointState, topic="bench/hello_ros")
TOPIC_WORLD = TopicInfo(msg_type=JointState, topic="bench/world_ros")
raw = False

case = 2
if case == 0: # 587 Hz
    rust_mode = False
    my_msg = cyclone_msg
    my_type = JointState
if case == 1: # 5944 Hz
    rust_mode = False
    my_msg = jit_msg
    my_type = JitJointState
if case == 2: # 1999 Hz
    rust_mode = True
    my_msg = rust_msg
    my_type = RustJointState
    TOPIC_HELLO = TopicInfo(msg_type=RustJointState, topic="bench/hello_ros")
    TOPIC_WORLD = TopicInfo(msg_type=RustJointState, topic="bench/world_ros")

try:
    print(len(my_msg.serialize()))
except:
    print("cannot estimate size")


if raw:
    payload = my_msg.serialize()
else:
    payload = my_msg


class PyZNode(MyNode):
    def __init__(self):
        super().__init__()

    def world_send(self):
        self.world_pub.publish(payload if raw or rust_mode else payload.serialize())

    def hello_send(self):
        self.hello_pub.publish(payload if raw or rust_mode else payload.serialize())

    def finish(self):
        self.fut.set_result(None)

    async def w_iter(self):
        async for msg in self.world_sub.listen_reliable():
            if not rust_mode:
                msg = memoryview(msg) if raw else my_type.deserialize(memoryview(msg))
            self.world_cbk(msg)

    async def h_iter(self):
        async for msg in self.hello_sub.listen_reliable():
            if not rust_mode:
                msg = memoryview(msg) if raw else my_type.deserialize(memoryview(msg))
            self.hello_cbk(msg)

    async def task(self):
        self.hello_pub = ZPublisher(*TOPIC_HELLO.as_arg())
        self.world_pub = ZPublisher(*TOPIC_WORLD.as_arg())
        self.hello_sub = Sub(
            *TOPIC_HELLO.as_arg(), raw=not rust_mode or raw
        )  # always raw except for not raw + rust
        self.world_sub = Sub(
            *TOPIC_WORLD.as_arg(), raw=not rust_mode or raw
        )  # always raw except for not raw + rust
        self.fut = asyncio.Future()
        tasks = [
            asyncio.create_task(self.w_iter()),
            asyncio.create_task(self.h_iter()),
            self.fut,
        ]
        await asyncio.sleep(0.1)
        self.hello_send()
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        for t in tasks:
            t.cancel()

    def run(self):
        uvloop.run(self.task())


if __name__ == "__main__":
    n = PyZNode()
    n.run()

# 18_446_7440_7369_2774_399
