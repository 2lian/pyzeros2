import asyncio
import asyncio_for_robotics as afor
import time
import itertools
import sys
from contextlib import suppress

async def funny_sleep(seconds: float, text: str = "Working...") -> None:
    if seconds < 0.0001:
        return
    if not sys.stdout.isatty():
        await asyncio.sleep(seconds)
        return

    frames = [
        "⠋ ",
        "⠙ ",
        "⠸ ",
        "⠰⠄",
        "⠠⠆",
        " ⠇",
        " ⠋",
        " ⠙",
        " ⠸",
        " ⠴",
        " ⠦",
        " ⠇",
        "⠈⠃",
        "⠘⠁",
        "⠸ ",
        "⠴ ",
        "⠦ ",
        "⠇",
    ]
    start = time.perf_counter()
    i = 0
    last_len = 0

    rate = afor.Rate(18/seconds)
    try:
        async for t_ns in rate.listen_reliable():
            elapsed = time.perf_counter() - start
            remaining = seconds - elapsed
            if remaining <= 0:
                break

            line = f"\r{text} {remaining:3.1f}s {frames[i % len(frames)]}"
            pad = max(0, last_len - len(line))
            sys.stdout.write(line + " " * pad)
            sys.stdout.flush()
            last_len = len(line)

            i += 1
    finally:
        rate.close()
        sys.stdout.write("\r" + " " * last_len + "\r")
        sys.stdout.flush()


async def with_throbber(awaitable, text: str = "Working..."):
    stop = asyncio.Event()
    spinner = asyncio.create_task(_throbber(text, stop))
    try:
        return await awaitable
    finally:
        stop.set()
        with suppress(asyncio.CancelledError):
            await spinner
