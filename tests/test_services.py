import asyncio
from asyncio import TaskGroup

import asyncio_for_robotics.ros2 as afor
import pytest
from ros2_pyterfaces.cyclone import all_srvs

from pyzeros.node import Node

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


async def test_pyzeros_client_calls_ros_service_with_async_bind(rclpy_init) -> None:
    node = Node(name="pyzeros_client_node", namespace="/tests/services/client")
    client = node.create_client(all_srvs.Trigger, "trigger", defer=True, timeout=RECV_TIMEOUT_S)
    service_name = "/tests/services/client/trigger"
    ros_service_type = all_srvs.Trigger.to_ros_type()
    ros_server = afor.Server(ros_service_type, service_name)
    server_ready = asyncio.Event()

    try:
        async with TaskGroup() as tg:
            client.declare()
            bind_task = tg.create_task(client.async_bind())
            tg.create_task(_serve_ros_trigger(ros_server, "ros-server", server_ready))

            await server_ready.wait()
            await asyncio.wait_for(client.wait_for_service(), timeout=RECV_TIMEOUT_S)
            response = await asyncio.wait_for(
                client.call(all_srvs.Trigger.Request()),
                timeout=RECV_TIMEOUT_S,
            )

            assert response.success is True
            assert response.message == "ros-server"

            bind_task.cancel()
            ros_server.close()
        assert client.token is None
        assert client.zenoh_cli is None
    finally:
        ros_server.close()
        node.undeclare()


async def test_ros_client_calls_pyzeros_service_with_async_bind(rclpy_init) -> None:
    node = Node(name="pyzeros_server_node", namespace="/tests/services/server")
    server = node.create_service(all_srvs.Trigger, "trigger", defer=True)
    service_name = "/tests/services/server/trigger"
    ros_service_type = all_srvs.Trigger.to_ros_type()
    ros_client = afor.Client(ros_service_type, service_name)
    server_ready = asyncio.Event()

    try:
        async with TaskGroup() as tg:
            server.declare()
            bind_task = tg.create_task(server.async_bind())
            tg.create_task(_serve_pyzeros_trigger(server, "pyzeros-server", server_ready))

            await server_ready.wait()
            await asyncio.wait_for(ros_client.wait_for_service(), timeout=RECV_TIMEOUT_S)
            response = await asyncio.wait_for(
                ros_client.call(ros_service_type.Request()),
                timeout=RECV_TIMEOUT_S,
            )

            assert response.success is True
            assert response.message == "pyzeros-server"

            bind_task.cancel()
            server.close()
        assert server.token is None
        assert server.zenoh_srv is None
    finally:
        ros_client.close()
        server.close()
        node.undeclare()
