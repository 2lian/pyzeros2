import zenoh
from ros2_pyterfaces.cyclone import all_msgs

from pyzeros.pub import Pub, token_keyexpr as publisher_token_keyexpr
from pyzeros.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QosDuration,
    QosProfile,
    ReliabilityPolicy,
)
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
    session = zenoh.open(zenoh.Config())
    pub = Pub(
        all_msgs.String,
        "/tests/qos_best_effort",
        qos_profile=QosProfile.sensor_data(),
        session=session,
        defer=True,
    )
    try:
        pub.declare()
        assert pub.zenoh_pub is not None
        assert pub.zenoh_pub.reliability == zenoh.Reliability.BEST_EFFORT
        assert pub.zenoh_pub.congestion_control == zenoh.CongestionControl.DROP
    finally:
        pub.undeclare()
        session.close()


def test_publisher_declares_reliable_keep_all_with_blocking():
    session = zenoh.open(zenoh.Config())
    pub = Pub(
        all_msgs.String,
        "/tests/qos_keep_all",
        qos_profile=QosProfile(history=HistoryPolicy.KEEP_ALL),
        session=session,
        defer=True,
    )
    try:
        pub.declare()
        assert pub.zenoh_pub is not None
        assert pub.zenoh_pub.reliability == zenoh.Reliability.RELIABLE
        assert pub.zenoh_pub.congestion_control == zenoh.CongestionControl.BLOCK
    finally:
        pub.undeclare()
        session.close()


def test_subscriber_queue_size_tracks_history():
    assert QosProfile(depth=7).subscriber_queue_size == 7
    assert QosProfile(history=HistoryPolicy.KEEP_ALL).subscriber_queue_size == 0


def test_transient_local_is_rejected_for_publishers_and_subscribers():
    session = zenoh.open(zenoh.Config())
    pub = Pub(
        all_msgs.String,
        "/tests/transient_pub",
        qos_profile=QosProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL),
        session=session,
        defer=True,
    )
    sub = RawSub(
        all_msgs.String,
        "/tests/transient_sub",
        callback=lambda _sample: None,
        qos_profile=QosProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL),
        session=session,
        defer=True,
    )
    try:
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
    finally:
        pub.undeclare()
        sub.undeclare()
        session.close()
