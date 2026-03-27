import logging
from os import environ
import subprocess

import pytest
from asyncio_for_robotics.core._logger import setup_logger

setup_logger(debug_path=".")
logger = logging.getLogger("asyncio_for_robotics.test")


# @pytest.fixture(scope="session", autouse=True)
def zenoh_router():
    """ """
    if environ.get("RMW_IMPLEMENTATION") != "rmw_zenoh_cpp":
        yield None
        return
    logger.info("Starting zenoh router")
    proc = subprocess.Popen(
        ["pixi", "run", "router"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    for line in iter(proc.stdout.readline, ""):
        if "Zenoh can be reached at" in line:
            logger.info("Router started!")
            break
    yield proc
    logger.info("Closing zenoh router")
    proc.terminate()
    proc.wait()
    logger.info("Closed zenoh router")
