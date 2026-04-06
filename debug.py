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
            node = Node(defer=True)
            pub = node.create_publisher(all_msgs.String, f"heyyo_{k}", defer=True)
            sub = node.create_subscriber(all_msgs.String, f"heyyo_{k}", defer=True)
            tg.create_task(node.async_bind())
            tg.create_task(pub.async_bind())
            tg.create_task(sub.async_bind())

            async for _ in afor.Rate(10).listen():
                pub.publish(all_msgs.String(f"{pub.count}"))
                print(sub._value)


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
