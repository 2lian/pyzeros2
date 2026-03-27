import asyncio

import uvloop

from base import MyNode
from ros2_pyterfaces.all_msgs import String as IdlString
from ros_z_msgs_py.types.std_msgs import String as RustString

String = RustString
from pyzeros.pub import ZPublisher
from pyzeros.sub import Sub
from pyzeros.utils import TopicInfo


TOPIC_HELLO = TopicInfo(msg_type=String, topic="bench/hello_ros")
TOPIC_WORLD = TopicInfo(msg_type=String, topic="bench/world_ros")
payload = "Unga Bunga"*100_000
payload = memoryview(payload.encode())
print(len(payload))
payload = String(data="Unga Bunga"*1)

class PyZNode(MyNode):
    def __init__(self):
        super().__init__()

    def world_send(self):
        self.world_pub.publish(payload)

    def hello_send(self):
        self.hello_pub.publish(payload)

    def finish(self):
        self.fut.set_result(None)

    async def w_iter(self):
        async for msg in self.world_sub.listen_reliable():
            self.world_cbk(msg)

    async def h_iter(self):
        async for msg in self.hello_sub.listen_reliable():
            self.hello_cbk(msg)

    async def task(self):
        self.hello_pub = ZPublisher(*TOPIC_HELLO.as_arg())
        self.world_pub = ZPublisher(*TOPIC_WORLD.as_arg())
        self.hello_sub = Sub(*TOPIC_HELLO.as_arg(), raw=False)
        self.world_sub = Sub(*TOPIC_WORLD.as_arg(), raw=False)
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
