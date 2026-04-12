import asyncio

import asyncio_for_robotics as afor
import pytest
import zenoh
from ros2_pyterfaces.cyclone import all_msgs, all_srvs

import pyzeros
from pyzeros.pub import Pub, token_keyexpr as publisher_token_keyexpr
from pyzeros.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QosDuration,
    QosProfile,
    ReliabilityPolicy,
)
from pyzeros.service_client import Client
from pyzeros.service_server import Server
from pyzeros.session import session_context
from pyzeros.sub import RawSub, token_keyexpr as subscriber_token_keyexpr


def test_default_qos_encodes_with_rmw_zenoh_compatible_default_depth():
    assert QosProfile.default().encode() == "::,10:,:,:,,"


def test_sensor_data_qos_encodes_best_effort_depth():
    assert QosProfile.sensor_data().encode() == "2::,5:,:,:,,"


def test_qos_encodes_non_default_deadline_lifespan_and_liveliness():
    qos = QosProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_ALL,
        deadline=QosDuration(5, 0),
        lifespan=QosDuration(60, 3000),
        liveliness=LivelinessPolicy.MANUAL_BY_TOPIC,
        liveliness_lease_duration=QosDuration.from_seconds(0.25),
    )
    assert qos.encode() == "2::2,:5,0:60,3000:3,0,250000000"


def test_pub_token_keyexpr_embeds_encoded_qos():
    qos = QosProfile.sensor_data()
    token = publisher_token_keyexpr(
        name="chatter",
        dds_type="std_msgs::msg::dds_::String_",
        hash="RIHS01_deadbeef",
        qos_profile=qos,
        node_name="talker",
        domain_id=0,
        namespace="/tests",
        _enclave="%",
        _node_id=1,
        _zenoh_id="zid",
        _entity_id=2,
    )
    assert token.endswith("/2::,5:,:,:,,")


def test_sub_token_keyexpr_embeds_encoded_qos():
    qos = QosProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_ALL,
    )
    token = subscriber_token_keyexpr(
        name="chatter",
        dds_type="std_msgs::msg::dds_::String_",
        hash="RIHS01_deadbeef",
        qos_profile=qos,
        node_name="listener",
        domain_id=0,
        namespace="/tests",
        _enclave="%",
        _node_id=1,
        _zenoh_id="zid",
        _entity_id=2,
    )
    assert token.endswith("/::2,:,:,:,,")


def test_publisher_declares_best_effort_qos():
    with session_context(pyzeros.Session(node="qos_be")) as node:
        pub = node.create_publisher(
            all_msgs.String,
            "/tests/qos_best_effort",
            qos_profile=QosProfile.sensor_data(),
            defer=True,
        )
        pub.declare()
        assert pub.zenoh_pub is not None
        assert pub.zenoh_pub.reliability == zenoh.Reliability.BEST_EFFORT
        assert pub.zenoh_pub.congestion_control == zenoh.CongestionControl.DROP
        pub.close()


def test_publisher_declares_reliable_keep_all_with_blocking():
    with session_context(pyzeros.Session(node="qos_ka")) as node:
        pub = node.create_publisher(
            all_msgs.String,
            "/tests/qos_keep_all",
            qos_profile=QosProfile(history=HistoryPolicy.KEEP_ALL),
            defer=True,
        )
        pub.declare()
        assert pub.zenoh_pub is not None
        assert pub.zenoh_pub.reliability == zenoh.Reliability.RELIABLE
        assert pub.zenoh_pub.congestion_control == zenoh.CongestionControl.BLOCK
        pub.close()


def test_subscriber_queue_size_tracks_history():
    assert QosProfile(depth=7).subscriber_queue_size == 7
    assert QosProfile(history=HistoryPolicy.KEEP_ALL).subscriber_queue_size == 0


def test_transient_local_is_rejected_for_publishers_and_subscribers():
    with session_context(pyzeros.Session(node="qos_tl")) as node:
        pub = node.create_publisher(
            all_msgs.String,
            "/tests/transient_pub",
            qos_profile=QosProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL),
            defer=True,
        )
        sub = RawSub(
            all_msgs.String,
            "/tests/transient_sub",
            callback=lambda _sample: None,
            qos_profile=QosProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL),
            session=node,
            defer=True,
        )

        try:
            pub.declare()
            raise AssertionError("Publisher declaration unexpectedly succeeded.")
        except NotImplementedError:
            pass

        try:
            sub.declare()
            raise AssertionError("Subscriber declaration unexpectedly succeeded.")
        except NotImplementedError:
            pass

        pub.close()
        sub.close()


@pytest.mark.asyncio
async def test_scope_closes_resources_but_not_nodes():
    with session_context(pyzeros.Session(node="bind_node", namespace="/")) as node:
        async with afor.Scope():
            pub = Pub(all_msgs.String, "/tests/bind/pub", session=node)
            sub = RawSub(
                all_msgs.String,
                "/tests/bind/sub",
                callback=lambda _sample: None,
                session=node,
            )
            client = Client(
                all_srvs.Trigger,
                "/tests/bind/service",
                session=node,
            )
            server = Server(
                all_srvs.Trigger,
                "/tests/bind/service",
                session=node,
            )

            assert node.token is not None
            assert pub.token is not None
            assert pub.zenoh_pub is not None
            assert sub.token is not None
            assert sub.zenoh_sub is not None
            assert client.token is not None
            assert client.zenoh_cli is not None
            assert server.token is not None
            assert server.zenoh_srv is not None

        assert node.token is not None
        assert pub.token is None
        assert pub.zenoh_pub is None
        assert sub.token is None
        assert sub.zenoh_sub is None
        assert client.token is None
        assert client.zenoh_cli is None
        assert server.token is None
        assert server.zenoh_srv is None


@pytest.mark.asyncio
async def test_scope_none_opt_out_keeps_resource_alive() -> None:
    with session_context(pyzeros.Session(node="bind_node_opt_out")) as node:
        async with afor.Scope():
            pub = Pub(
                all_msgs.String,
                "/tests/bind/pub",
                session=node,
                scope=None,
            )
            assert pub.token is not None

        assert pub.token is not None
        assert pub.zenoh_pub is not None
        pub.close()
        assert pub.token is None
        assert pub.zenoh_pub is None
