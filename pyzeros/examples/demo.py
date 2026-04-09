"""Small ring demo for the current `pyzeros` asyncio pub/sub API.

A set of participants is arranged in a ring:

    participant_0 -> participant_1 -> ... -> participant_N-1 -> participant_0

Each participant listens on its own topic and forwards a progressively longer
string to the next participant. Starting from an initial payload, the ring grows
the message one character at a time until it matches the requested target
string, then resets back to the empty string.

This demo is intentionally simple. It shows:
- the composition of many non-blocking asyncio tasks,
- one subscriber and publisher per task,
- non-blocking delays between sub->pub,
- all running in a single thread.

You can abuse the number of participant and sleep time, no problem ;)
"""

import argparse
import asyncio
from contextlib import suppress

from ros2_pyterfaces.cyclone.all_msgs import String

from pyzeros.node import Node

from ._demo_utils import funny_sleep


def next_payload(payload: str, target_string: str) -> str:
    """Return the next payload to circulate in the ring."""
    if payload == target_string:
        return ""
    if target_string.startswith(payload) and len(payload) < len(target_string):
        return payload + target_string[len(payload)]
    # Fallback for unexpected input: restart from the beginning.
    return target_string[:1]


async def pass_along(
    topic: str,
    sub,
    pub,
    *,
    target: str,
    sleep_time: float,
) -> None:
    """Listen on one subscriber and forwards to a publisher.

    Args:
        topic: Input topic for this participant.
        sub: Typed subscriber for this participant's input topic.
        pub: Publisher targeting the next participant.
        target: Final string the ring tries to assemble.
        sleep_time: Artificial delay inserted before forwarding.
    """
    async for ros_msg in sub.listen_reliable():
        payload = ros_msg.data
        print(f"[{topic}] I heard `{payload}`")

        if payload == target:
            print("WOW!")

        new_payload = next_payload(payload, target)

        # Equivalent to asyncio.sleep, this illustrates the capability of
        # non-blocking sleep so other tasks can continue running.
        await funny_sleep(sleep_time, f"[{topic}] Sending in")
        pub.publish(String(data=new_payload))


async def main(args: argparse.Namespace) -> None:
    """Create the ring topology and run all participant tasks."""

    if args.participants < 1:
        raise SystemExit("--participants must be >= 1")
    if args.sleep < 0:
        raise SystemExit("--sleep must be >= 0")

    async with asyncio.TaskGroup() as tg:
        node = Node(name="ring_demo", namespace="/pyzeros")
        tg.create_task(node.async_bind())
        topics = [f"{args.topic_prefix}{k}" for k in range(args.participants)]
        subs = [node.create_subscriber(String, topic) for topic in topics]
        pubs = [node.create_publisher(String, topic) for topic in topics]

        for participant in range(args.participants):
            topic = topics[participant]
            sub = subs[participant]
            # Participant k publishes to participant k+1, with wrap-around at
            # the end to close the ring.
            pub = pubs[(participant + 1) % args.participants]
            tg.create_task(
                pass_along(
                    topic,
                    sub,
                    pub,
                    target=args.target,
                    sleep_time=args.sleep,
                )
            )
            tg.create_task(pub.async_bind())
            tg.create_task(sub.async_bind())

        # Inject the initial payload to start the ring.
        pubs[0].publish(String(data=args.initial))

        # Awaits indefinitely.
        await asyncio.Future()


def build_parser() -> argparse.ArgumentParser:
    """Create the command-line interface for the demo."""
    parser = argparse.ArgumentParser(
        description="Pass a message around a ring of participants."
    )
    parser.add_argument(
        "-n",
        "--participants",
        type=int,
        default=4,
        metavar="N",
        help="Number of participants in the ring.",
    )
    parser.add_argument(
        "-s",
        "--sleep",
        type=float,
        default=1.0,
        metavar="SECONDS",
        help="Delay before forwarding each message.",
    )
    parser.add_argument(
        "-t",
        "--target",
        type=str,
        default="Hello World!",
        metavar="TEXT",
        help="Target message to build progressively.",
    )
    parser.add_argument(
        "--initial",
        type=str,
        default="",
        metavar="TEXT",
        help="Initial message injected into the ring.",
    )
    parser.add_argument(
        "--topic-prefix",
        type=str,
        default="participant_",
        metavar="PREFIX",
        help="Prefix used to build participant topic names.",
    )
    return parser


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        args = build_parser().parse_args()
        asyncio.run(main(args))
