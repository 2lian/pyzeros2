import pytest
import asyncio_for_robotics as afor
from ros2_pyterfaces.cyclone import all_msgs, all_srvs

import pyzeros
from pyzeros.pub import Pub
from pyzeros.service_client import Client
from pyzeros.service_server import Server
from pyzeros.session import session_context
from pyzeros.sub import RawSub, Sub


@pytest.mark.asyncio
async def test_scope_auto_attaches_and_closes_resources() -> None:
    with session_context(pyzeros.Session(node="scope_owner", namespace="/tests/scope")) as session:
        async with afor.Scope() as scope:
            pub = Pub(all_msgs.String, "pub", session=session)
            raw_sub = RawSub(
                all_msgs.String,
                "raw_sub",
                callback=lambda _sample: None,
                session=session,
            )
            sub = Sub(all_msgs.String, "sub", session=session)
            client = Client(all_srvs.Trigger, "/tests/scope/service", session=session)
            server = Server(all_srvs.Trigger, "/tests/scope/service", session=session)

            assert pub._scope is scope
            assert raw_sub._scope is scope
            assert sub._scope is scope
            assert sub.sample_sub._scope is scope
            assert sub.raw_sub._scope is scope
            assert client._scope is scope
            assert server._scope is scope
            assert session.token is not None
            assert pub.token is not None
            assert raw_sub.token is not None
            assert sub.raw_sub.token is not None
            assert client.token is not None
            assert server.token is not None

        assert session.token is not None
        assert pub.token is None
        assert raw_sub.token is None
        assert sub.raw_sub.token is None
        assert client.token is None
        assert server.token is None
        assert pub.lifetime.done()
        assert raw_sub.lifetime.done()
        assert sub.lifetime.done()
        assert client.lifetime.done()
        assert server.lifetime.done()


@pytest.mark.asyncio
async def test_scope_none_keeps_resource_manual() -> None:
    with session_context(pyzeros.Session(node="scope_opt_out")) as session:
        async with afor.Scope():
            pub = Pub(all_msgs.String, "pub", session=session, scope=None)
            assert pub.token is not None
            assert pub._scope is None

        assert pub.token is not None
        pub.close()
        assert pub.token is None


@pytest.mark.asyncio
async def test_attach_scope_later() -> None:
    with session_context(pyzeros.Session(node="scope_attach")) as session:
        pub = Pub(all_msgs.String, "pub", session=session, scope=None)
        assert pub.token is not None
        assert pub._scope is None

        async with afor.Scope() as scope:
            pub.attach(scope)
            assert pub._scope is scope

        assert pub.token is None
