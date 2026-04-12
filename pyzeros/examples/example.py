"""Minimal session-based subscriber example."""

import asyncio
from contextlib import suppress

import asyncio_for_robotics as afor
from ros2_pyterfaces.cyclone.all_msgs import String

import pyzeros


@afor.scoped
async def main() -> None:
    """Create one session, subscribe to `/pyzeros/chatter`, and print messages."""
    session = pyzeros.current_session()
    print(
        f"Listening as node `{session.name}` in namespace `{session.namespace}`"
    )
    sub = pyzeros.Sub(String, "chatter")

    print(
        "\nTalk to me on ROS 2 using:\n"
        f"pixi run -e ros ros2 topic pub \"{sub.fully_qualified_name}\" "
        "std_msgs/msg/String '{data: \"Hello_World\"}'\n"
    )

    async for msg in sub.listen_reliable():
        print(msg.data)


if __name__ == "__main__":
    with pyzeros.auto_context(node="listener", namespace="/pyzeros"):
        with suppress(KeyboardInterrupt):
            asyncio.run(main())
