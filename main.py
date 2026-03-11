import asyncio
import struct
import time
import uuid
from contextlib import suppress
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pprint import pformat, pprint
from typing import Self

import asyncio_for_robotics.zenoh as afor
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import (
    array,
    float32,
    float64,
    int8,
    int16,
    int32,
    int64,
    sequence,
    uint8,
    uint16,
    uint32,
    uint64,
)

from myidl import Attachment, ParameterEvent, String, Vector3D

ID = uuid.uuid4().bytes


async def pub_loop():
    """Makes a publisher without using ROS"""
    # return
    ses = afor.auto_session()
    pub = ses.declare_publisher(
        "0/not_ros_pub/geometry_msgs::msg::dds_::Vector3_/RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d"
    )
    att = Attachment(
        sequence_number=-1,
        source_timestamp=time.time_ns(),
        gid_length=16,
        source_gid=ID,
    )
    async for t_ns in afor.Rate(2).listen():
        att.sequence_number += 1
        att.source_timestamp = time.time_ns()
        pub.put(
            Vector3D(att.sequence_number, 10, 10).serialize(),
            attachment=att.serialize()[4:],
        )


async def listen_str():
    sub = afor.Sub(
        "0/not_ros_sub/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18/**"
    )
    async for s in sub.listen_reliable():
        assert s.attachment is not None
        print(
            "\nHear on topic not_ros_sub\n",
            "- ROS2 Parsed Payload: \n  ",
            pformat(String.deserialize(s.payload.to_bytes())),
            "\n",
            "- ROS2 Payload Metadata: \n  ",
            pformat(Attachment.deserialize(s.attachment.to_bytes(), has_header=False)),
        )


async def listen_param_event():
    sub = afor.Sub("0/parameter_events/rcl_interfaces::msg::dds_::ParameterEvent_/**")
    async for s in sub.listen_reliable():
        assert s.attachment is not None
        pprint(s.key_expr)
        pprint(
            (ParameterEvent.deserialize(s.payload.to_bytes(), has_header=True)),
        )


async def sub_alive():
    ses = afor.auto_session()
    tok = ses.liveliness().declare_token(
        f"@ros2_lv/0/{ID.hex}/0/5/MP/%/%/not_ros_node/%not_ros_sub/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18/::,10:,:,:,,"
    )
    try:
        await asyncio.Future()
    finally:
        tok.undeclare()


async def pub_alive():
    ses = afor.auto_session()
    tok = ses.liveliness().declare_token(
        f"@ros2_lv/0/{ID.hex}/0/5/MP/%/%/not_ros_node/%not_ros_pub/geometry_msgs::msg::dds_::Vector3_/RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d/::,10:,:,:,,"
    )
    try:
        await asyncio.Future()
    finally:
        tok.undeclare()


async def node_alive():
    ses = afor.auto_session()
    tok = ses.liveliness().declare_token(
        f"@ros2_lv/0/{ID.hex}/0/5/NN/%/%/not_ros_node"
    )
    try:
        await asyncio.Future()
    finally:
        tok.undeclare()


async def listen_all():
    sub = afor.Sub("**")
    async for s in sub.listen_reliable():
        assert s.attachment is not None
        print("\nSample: ", s)
        print("Bytes: ", s.payload.to_bytes())
        print("Bytes len: ", len(s.payload.to_bytes()))
        print("Attachement: ", s.attachment.to_bytes())
        print("Attachement len: ", len(s.attachment.to_bytes()))
        print(
            "Attachment idl parse: ",
            pformat(Attachment.deserialize(s.attachment.to_bytes(), has_header=False)),
        )
        # print(
        # "Payload idl parse: ",
        # pformat(ParameterEvent.deserialize(s.payload.to_bytes(), has_header=True)),
        # )
        # print("Idl: ", pformat(d))


async def main():
    async with asyncio.TaskGroup() as tg:
        # tg.create_task(listen_all())
        # tg.create_task(listen_param_event())
        tg.create_task(listen_str())
        tg.create_task(node_alive())
        tg.create_task(pub_alive())
        tg.create_task(sub_alive())
        tg.create_task(pub_loop())
        print("Pure zenoh ROS 2 node started")
        print("""Checkout:
- pixi run ros2 node list
- pixi run ros2 topic list -t
- pixi run ros2 topic info /not_ros_pub -vv
- pixi run ros2 topic echo '/not_ros_pub'
- pixi run ros2 topic pub /not_ros_sub std_msgs/msg/String '{data: "Hello World!"}'
              """)


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
        # afor.auto_session().close()
