import pytest
import zenoh
from ros2_pyterfaces.cyclone import all_msgs

import pyzeros.session as session_module
from pyzeros import (
    Node,
    Pub,
    Session,
    Sub,
    auto_context,
    current_session,
)


def test_current_session_requires_context_by_default() -> None:
    with pytest.raises(RuntimeError, match="No active PyZeROS session context"):
        current_session()


def test_auto_context_sets_resets_and_nests() -> None:
    with auto_context(node="outer_session") as outer:
        assert current_session() is outer
        with auto_context(node="inner_session") as inner:
            assert current_session() is inner
        assert current_session() is outer
        assert inner.token is None
    with pytest.raises(RuntimeError, match="No active PyZeROS session context"):
        current_session()
    assert outer.token is None


def test_auto_context_does_not_close_explicit_node() -> None:
    node = Session(node="explicit_owner")
    try:
        with auto_context(node) as active:
            assert active is node
            assert current_session() is node
        assert node.token is not None
    finally:
        node.close()


async def test_auto_context_propagates_node_identity_to_entities() -> None:
    with auto_context(node="context_node") as session:
        pub = Pub(all_msgs.String, "context_topic", defer=True)
        sub = Sub(all_msgs.String, "context_topic", defer=True)

        assert session.name == "context_node"
        assert pub.node_name == "context_node"
        assert sub.raw_sub.node_name == "context_node"
        assert pub.namespace == session.namespace
        assert sub.raw_sub.namespace == session.namespace


async def test_entities_resolve_explicit_then_lexical_then_auto_create() -> None:
    lexical_session = Session(node="lexical_node")
    explicit_session = Session(node="explicit_node")
    auto_created: Node | None = None
    context_token = session_module._CURRENT_SESSION.set(None)

    try:
        explicit_pub = Pub(
            all_msgs.String,
            "resolution_topic",
            session=explicit_session,
            defer=True,
        )
        assert explicit_pub.node_name == "explicit_node"
        explicit_pub.close()

        with auto_context(lexical_session):
            lexical_pub = Pub(all_msgs.String, "resolution_topic", defer=True)
            assert lexical_pub.node_name == "lexical_node"
            lexical_pub.close()

        with pytest.warns(UserWarning, match="PyZeROS session was never declared"):
            auto_pub = Pub(all_msgs.String, "resolution_topic", defer=True)
        auto_created = current_session()
        assert auto_created is not None
        assert auto_pub.node_name == auto_created.name
        auto_pub.close()
    finally:
        session_module._CURRENT_SESSION.reset(context_token)
        explicit_session.close()
        lexical_session.close()
        if auto_created is not None:
            auto_created.close()


def test_session_uses_explicit_zenoh_session() -> None:
    zenoh_session = zenoh.open(zenoh.Config())
    with auto_context(node="manual_transport", zenoh_session=zenoh_session) as session:
        assert session.zenoh_session is zenoh_session
        assert session.session is zenoh_session
    assert not zenoh_session.is_closed()
    zenoh_session.close()


async def test_node_create_subscriber_still_uses_node_identity() -> None:
    with auto_context(node="compat_node", namespace="/tests/session") as node:
        sub = node.create_subscriber(all_msgs.String, "chatter", defer=True)
        assert sub.raw_sub.session is node.session
        assert sub.raw_sub.node_name == node.name
        assert sub.raw_sub.namespace == node.namespace
        assert sub.raw_sub.domain_id == node.domain_id
