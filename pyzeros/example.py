import asyncio
from pprint import pformat, pprint

import asyncio_for_robotics as afor
import ros_z_py

from .session import auto_session
from .sub import Sub, TopicInfo
from .utils import QOS_DEFAULT


async def no_helper_main():
    """No frill implementation"""
    # this is our universal afor subscriber
    sub = afor.BaseSub()
    # this is the context, node and sub
    builder = ros_z_py.ZContextBuilder()
    ctx = builder.build()
    node = ctx.create_node("pyzeros").build()
    # the subscriber inputs messages into our afor subscriber with it's callback
    node.create_subscriber(
        "/chatter",
        ros_z_py.std_msgs.String,
        callback=sub.input_data,  # thread safe btw
    )
    # we can now use standard asyncio syntax
    async for msg in sub.listen_reliable():
        pprint(msg)


my_topic = TopicInfo(
    topic="/chatter",
    msg_type=ros_z_py.std_msgs.String,
    qos=QOS_DEFAULT,
)


async def implemented_main():
    """More abstracted implementation"""
    # context, node and sub is created automatically
    sub = Sub(
        **my_topic.as_kwarg(),
        session=None,  # Still possible to provide a custom context/node here
    )
    # we can now use standard asyncio syntax
    async for msg in sub.listen_reliable():
        pprint(msg)


if __name__ == "__main__":
    try:
        asyncio.run(implemented_main())
    except KeyboardInterrupt:
        print("\nStopped.")
