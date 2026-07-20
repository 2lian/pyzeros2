import logging
import importlib.util
import subprocess
import time

import asyncio_for_robotics.zenoh as afor_zenoh
import pytest
from asyncio_for_robotics.core._logger import setup_logger

setup_logger(debug_path=".")
logger = logging.getLogger("asyncio_for_robotics.test")


def pytest_collection_modifyitems(config, items):
    """Skip marked ROS interop tests when the environment has no ``rclpy``."""
    if importlib.util.find_spec("rclpy") is not None:
        return
    skip_ros = pytest.mark.skip(reason="rclpy is not installed")
    for item in items:
        if "interop" in item.keywords:
            item.add_marker(skip_ros)


@pytest.fixture(scope="session", autouse=True)
def zenoh_router():
    """Run one local Zenoh router for the duration of the pytest session."""
    logger.info("Starting Zenoh router")
    proc = subprocess.Popen(
        ["zenohd", "-c", "./router.json5"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(1)
    if proc.poll() is not None:
        proc.terminate()
        proc.wait()
        raise RuntimeError(f"Zenoh router exited during startup: {proc.returncode}")

    try:
        yield proc
    finally:
        logger.info("Closing Zenoh router")
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        logger.info("Closed Zenoh router")


@pytest.fixture(scope="session", autouse=True)
def zenoh_session(zenoh_router):
    """Provide one explicitly owned Zenoh transport for the test session."""
    with afor_zenoh.auto_context() as session:
        yield session
