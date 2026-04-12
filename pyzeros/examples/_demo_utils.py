import asyncio
import itertools
import sys
import time
from contextlib import suppress

import asyncio_for_robotics as afor


@afor.scoped
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
    last_len = 0

    try:
        async for _ in afor.Rate(30).listen_reliable():
            elapsed = time.perf_counter() - start
            i = int(elapsed * len(frames)) % len(frames)
            remaining = seconds - elapsed
            if remaining <= 0:
                break

            line = f"\r{text} {remaining:3.1f}s {frames[i]}"
            pad = max(0, last_len - len(line))
            sys.stdout.write(line + " " * pad)
            sys.stdout.flush()
            last_len = len(line)
    finally:
        sys.stdout.write("\r" + " " * last_len + "\r")
        sys.stdout.flush()
