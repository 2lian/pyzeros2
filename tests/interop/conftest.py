import pytest


@pytest.fixture(scope="session")
def rclpy_init():
    """Initialize the ROS 2 test context for interop tests only."""
    afor = pytest.importorskip("asyncio_for_robotics.ros2")
    with afor.auto_context():
        yield
