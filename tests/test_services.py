import asyncio
from asyncio import TaskGroup

import asyncio_for_robotics as afor
import asyncio_for_robotics.ros2 as afor_ros
import pytest
from ros2_pyterfaces.cyclone import all_srvs

from pyzeros.node import Node
from pyzeros.service_client import Client
from pyzeros.service_server import Server
from pyzeros.session import session_context

RECV_TIMEOUT_S = 3

pytestmark = pytest.mark.asyncio(loop_scope="module")


async def _serve_pyzeros_trigger(server, label: str, started: asyncio.Event) -> None:
    responders = server.listen_reliable(queue_size=4, exit_on_close=True)
    started.set()
    async for responder in responders:
        responder.response.success = True
        responder.response.message = label
        responder.send()


async def _serve_ros_trigger(server, label: str, started: asyncio.Event) -> None:
    responders = server.listen_reliable(queue_size=4, exit_on_close=True)
    started.set()
    async for responder in responders:
        responder.response.success = True
        responder.response.message = label
        responder.send()


async def test_pyzeros_client_calls_ros_service(rclpy_init) -> None:
    service_name = "/tests/services/client/trigger"
    ros_service_type = all_srvs.Trigger.to_ros_type()

    with session_context(
        Node(name="pyzeros_client_node", namespace="/tests/services/client")
    ):
        async with afor.Scope() as scope:
            ros_server = afor_ros.Server(ros_service_type, service_name)
            tg = scope.task_group
            client = Client(all_srvs.Trigger, "trigger")
            tg.create_task(
                _serve_ros_trigger(ros_server, "ros-server", asyncio.Event())
            )

            print(0)
            wait_result = await afor_ros.soft_wait_for(
                client.wait_for_service(), RECV_TIMEOUT_S
            )
            if isinstance(wait_result, TimeoutError):
                pytest.fail("PyZeROS client did not observe the ROS service in time.")
            response = await afor_ros.soft_wait_for(
                client.call_async(all_srvs.Trigger.Request()),
                RECV_TIMEOUT_S,
            )
            print(response)
            if isinstance(response, TimeoutError):
                pytest.fail(
                    "PyZeROS client did not receive the ROS service response in time."
                )

            assert response.success is True
            assert response.message == "ros-server"
            print(1)

        print(2)
        assert client.token is None
        assert client.zenoh_cli is None
    print("test_pyzeros_client_calls_ros_service done")


async def test_ros_client_calls_pyzeros_service(rclpy_init) -> None:
    service_name = "/tests/services/server/trigger"
    ros_service_type = all_srvs.Trigger.to_ros_type()

    with session_context(
        Node(name="pyzeros_server_node", namespace="/tests/services/server")
    ):
        async with afor.Scope() as scope:
            tg = scope.task_group
            ros_client = afor_ros.Client(ros_service_type, service_name)
            server_ready = asyncio.Event()
            server = Server(all_srvs.Trigger, "trigger")
            tg.create_task(
                _serve_pyzeros_trigger(server, "pyzeros-server", server_ready)
            )

            wait_result = await afor_ros.soft_wait_for(
                ros_client.wait_for_service(), RECV_TIMEOUT_S
            )
            if isinstance(wait_result, TimeoutError):
                pytest.fail("ROS client did not observe the PyZeROS service in time.")
            response = await afor_ros.soft_wait_for(
                ros_client.call(ros_service_type.Request()),
                RECV_TIMEOUT_S,
            )
            if isinstance(response, TimeoutError):
                pytest.fail(
                    "ROS client did not receive the PyZeROS service response in time."
                )

            assert response.success is True
            assert response.message == "pyzeros-server"
        assert server.token is None
        assert server.zenoh_srv is None


async def test_pyzeros_client_calls_pyzeros_service() -> None:
    with (
        session_context(
            Node(
                name="pyzeros_client_node2", namespace="/tests/services/pyzeros_client"
            )
        ) as client_node,
        session_context(
            Node(
                name="pyzeros_server_node2", namespace="/tests/services/pyzeros_server"
            )
        ) as server_node,
    ):
        async with afor.Scope() as scope:
            tg = scope.task_group
            client = client_node.create_client(
                all_srvs.Trigger, "/tests/services/pyzeros/trigger"
            )
            server = server_node.create_service(
                all_srvs.Trigger, "/tests/services/pyzeros/trigger"
            )
            tg.create_task(
                _serve_pyzeros_trigger(server, "pyzeros-server", asyncio.Event())
            )

            wait_result = await afor_ros.soft_wait_for(
                client.wait_for_service(), RECV_TIMEOUT_S
            )
            if isinstance(wait_result, TimeoutError):
                pytest.fail(
                    "PyZeROS client did not observe the PyZeROS service in time."
                )
            response = await afor_ros.soft_wait_for(
                client.call_async(all_srvs.Trigger.Request()),
                RECV_TIMEOUT_S,
            )
            if isinstance(response, TimeoutError):
                pytest.fail(
                    "PyZeROS client did not receive the PyZeROS service response in time."
                )

            assert response.success is True
            assert response.message == "pyzeros-server"
        assert client.token is None
        assert client.zenoh_cli is None
        assert server.token is None
        assert server.zenoh_srv is None
