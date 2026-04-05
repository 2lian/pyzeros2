import asyncio
from pprint import pformat, pprint

import asyncio_for_robotics as afor
import ros_z_py
from ros2_pyterfaces.cyclone import all_msgs

from .session import auto_session
from .sub import Sub, TopicInfo
from .utils import QOS_DEFAULT, CdrModes, get_type_shim


async def no_helper_main():
    """Manual setup using a raw subscription and explicit deserialization."""
    # this is our universal afor subscriber
    sub = afor.BaseSub()
    # this is the context, node and sub
    # using this you can name the node, namespace and all
    builder = ros_z_py.ZContextBuilder()
    ctx = builder.build()
    node = ctx.create_node("pyzeros").build()
    # the Z subscriber inputs messages into our afor subscriber with it's callback
    raw_sub = node.create_subscriber(
        "/chatter",
        get_type_shim(all_msgs.String),
        callback=sub.input_data,  # thread safe btw
        raw=True,  # this will give us a zero-copy view of the payload, no de-serialization
    )
    # we can now use standard asyncio syntax
    async for msg in sub.listen_reliable():
        pprint(all_msgs.String.deserialize(msg))


my_topic = TopicInfo(
    topic="/chatter",
    msg_type=all_msgs.String,
    qos=QOS_DEFAULT,
)


async def implemented_main():
    """Use `Sub` directly with a typed message class."""
    # context, node and sub is created automatically
    sub = Sub(
        my_topic.msg_type,
        my_topic.topic,
        my_topic.qos,
        session=None,  # Optional, custom context/node here
        cdr_mode=CdrModes.AUTO,  # Optional, python or rust can handle the cdr depending on the msg type you are using and your build of ros-z.
        raw=False,  # Optional, here the payload will be serialized into the msg_type
    )
    # standard asyncio syntax
    async for msg in sub.listen_reliable():
        pprint(msg)


async def helpful_main():
    """Same typed subscriber example with a friendlier startup error."""
    try:
        node = auto_session()
    except ros_z_py.RosZError as e:
        raise ros_z_py.RosZError(
            "You likely forgot to start the router! "
            "In another terminal run `pixi run router`"
        ) from e
    sub = Sub(
        *my_topic.as_arg(),
        session=node,  # Optional custom context/node here
    )
    print(
        """\nTalk to me on ROS 2 using :\npixi run -e ros ros2 topic pub "/chatter" std_msgs/msg/String "{data: "Hello_World"}"\n"""
    )
    async for msg in sub.listen_reliable():
        pprint(msg)


if __name__ == "__main__":
    try:
        asyncio.run(helpful_main())
    except KeyboardInterrupt:
        print("\nStopped.")
