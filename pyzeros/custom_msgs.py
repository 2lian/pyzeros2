"""
Hack showing how to use a custom ROS 2 message with ros-z. (can't believe this
                                                            works so easily)

This works by mixing three pieces:

- `cyclonedds.idl` handles binary serialization/deserialization
- `msgspec.Struct` gives ros-z the ROS message type and type hash
- `publish_raw()` / `recv_serialized()` bypass ros-z message serialization

This is a hack, not a production-ready interface. In particular, the message
hash is copied from ROS 2 cli instead of being generated here.

How to get the hash:
    You cannot generate it easily. Copy it from ROS 2.

Example for `diagnostic_msgs/msg/KeyValue`:
    Terminal 1:
        ros2 topic pub "/custom_key_val" diagnostic_msgs/msg/KeyValue

    Terminal 2:
        ros2 topic info "/custom_key_val" -vv
"""

import asyncio
import janus
from contextlib import suppress
from pprint import pprint
from dataclasses import dataclass
from typing import ClassVar

import asyncio_for_robotics as afor
import msgspec
from cyclonedds.idl import IdlStruct
import threading

from .session import auto_session


@dataclass
class CycloneKeyValue(IdlStruct):
    """Cyclone DDS IDL view of diagnostic_msgs/msg/KeyValue.

    This KeyValue message type is not part of ros-z!
    """
    key: str = ""
    value: str = ""


class KeyValue(msgspec.Struct, frozen=True, kw_only=True):
    """ROS-facing message wrapper used by ros-z.

    This KeyValue message type is not part of ros-z!
    """
    key: str = ""
    value: str = ""
    __msgtype__: ClassVar[str] = "diagnostic_msgs/msg/KeyValue"
    __hash__: ClassVar[str] = (
        # I blatantly took this hash from ROS 2 cli
        "RIHS01_d68081eaa540288c5440753baecef0c4e16e81a5f78ad68902ded5100413bb42"
    )

    def serialize(self):
        """Serialize using Cyclone DDS IDL."""
        return CycloneKeyValue(self.key, self.value).serialize()

    @classmethod
    def deserialize(cls, data):
        """Deserialize bytes produced by Cyclone DDS IDL."""
        ckv = CycloneKeyValue.deserialize(data)
        return cls(key=ckv.key, value=ckv.value)


async def sub_task():
    """Receive raw bytes, then decode them as KeyValue.

    We need a worker thread for this as no callback is available on raw subs.
    So it is a bit more involved.
    """
    node = auto_session()
    raw_sub = node.create_subscriber("/custom_key_val", KeyValue)
    sub: afor.BaseSub[bytes] = afor.BaseSub()
    jq = janus.Queue()

    def ingress_backgroud_thread():
        """Poll the raw subscriber and feed bytes into afor."""
        while 1:
            # No async/callback API here, so we poll the sub.
            msg: bytes | None = raw_sub.recv_serialized(0.3)
            if msg is None:
                continue
            # We put the result in a threadsafe+async queue
            try:
                jq.sync_q.put(msg)
            except janus.AsyncQueueShutDown:
                return

    async def ingress_afor():
        try:
            while 1:
                sub.input_data(await jq.async_q.get())
                jq.async_q.task_done()
        except:
            jq.shutdown()

    thd = threading.Thread(target=ingress_backgroud_thread, daemon=True)
    async with asyncio.TaskGroup() as tg:
        tg.create_task(ingress_afor())
        thd.start()
        
        # our normal afor sub is (finally) ready
        async for msg in sub.listen_reliable():
            print("Sub deserializing data:")
            pprint(KeyValue.deserialize(msg))


async def pub_task():
    """Publish a KeyValue message periodically."""
    node = auto_session()
    pub = node.create_publisher("/custom_key_val", KeyValue)
    async for t_ns in afor.Rate(2).listen():
        pub.publish_raw(KeyValue(key="time_ns", value=str(t_ns)).serialize())


async def main():
    """Run publisher and subscriber together."""
    async with asyncio.TaskGroup() as tg:
        tg.create_task(pub_task())
        tg.create_task(sub_task())


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
