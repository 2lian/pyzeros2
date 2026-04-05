import asyncio
import os
import uuid
from contextlib import suppress

import asyncio_for_robotics.zenoh as afor
from ros2_pyterfaces.cyclone import all_msgs
import zenoh

from pyzeros.node import Node


async def main():
    async with asyncio.TaskGroup() as tg:
        for k in range(1):
            node = Node()
            pub = node.create_publisher(all_msgs.String, f"heyyo_{k}")
            tg.create_task(node.async_bind())
            tg.create_task(pub.async_bind())

            async for _ in afor.Rate(10).listen():
                pub.publish(all_msgs.String("Hello World!"))


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
