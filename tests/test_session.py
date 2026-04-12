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
    session_context,
)


def test_current_session_requires_context_by_default() -> None:
    assert current_session(None) is None
    with pytest.raises(RuntimeError, match="No active pyzeros session context"):
        current_session()


def test_session_context_sets_resets_and_nests() -> None:
    outer = Session(node="outer_session")
    inner = Session(node="inner_session")

    try:
        with session_context(outer, close_on_exit=False):
            assert current_session() is outer
            with session_context(inner, close_on_exit=False):
                assert current_session() is inner
            assert current_session() is outer
        assert current_session(None) is None
    finally:
        inner.close()
        outer.close()


def test_auto_context_reuses_existing_lexical_session() -> None:
    with session_context(Session(node="lexical_owner")) as session:
        with auto_context(node="ignored_name") as active:
            assert active is session
            assert current_session() is session


@pytest.mark.asyncio
async def test_auto_context_propagates_node_identity_to_entities() -> None:
    with auto_context(node="context_node") as session:
        pub = Pub(all_msgs.String, "context_topic", defer=True)
        sub = Sub(all_msgs.String, "context_topic", defer=True)

        assert session.name == "context_node"
        assert pub.node_name == "context_node"
        assert sub.raw_sub.node_name == "context_node"
        assert pub.namespace == session.namespace
        assert sub.raw_sub.namespace == session.namespace


def test_entities_resolve_explicit_then_lexical_then_global_then_auto_create(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    auto_zenoh = zenoh.open(zenoh.Config())
    global_session = Session(node="global_node")
    lexical_session = Session(node="lexical_node")
    explicit_session = Session(node="explicit_node")
    auto_created: Node | None = None

    monkeypatch.setattr(session_module, "GLOBAL_SESSION", global_session)
    monkeypatch.setattr(
        session_module.afor,
        "auto_session",
        lambda session=None: auto_zenoh if session is None else session,
    )

    try:
        explicit_pub = Pub(
            all_msgs.String,
            "resolution_topic",
            session=explicit_session,
            defer=True,
        )
        assert explicit_pub.node_name == "explicit_node"
        explicit_pub.close()

        with session_context(lexical_session, close_on_exit=False):
            lexical_pub = Pub(all_msgs.String, "resolution_topic", defer=True)
            assert lexical_pub.node_name == "lexical_node"
            lexical_pub.close()

        global_pub = Pub(all_msgs.String, "resolution_topic", defer=True)
        assert global_pub.node_name == "global_node"
        global_pub.close()

        monkeypatch.setattr(session_module, "GLOBAL_SESSION", None)
        auto_pub = Pub(all_msgs.String, "resolution_topic", defer=True)
        auto_created = session_module.GLOBAL_SESSION
        assert auto_created is not None
        assert auto_pub.node_name == auto_created.name
        assert auto_created.zenoh_session is auto_zenoh
        auto_pub.close()
    finally:
        explicit_session.close()
        lexical_session.close()
        global_session.close()
        if auto_created is not None:
            auto_created.close()
        if not auto_zenoh.is_closed():
            auto_zenoh.close()


def test_session_uses_explicit_zenoh_session() -> None:
    zenoh_session = zenoh.open(zenoh.Config())
    with session_context(Session(node="manual_transport", zenoh_session=zenoh_session)) as session:
        assert session.zenoh_session is zenoh_session
        assert session.session is zenoh_session
    zenoh_session.close()


def test_node_create_subscriber_still_uses_node_identity() -> None:
    with session_context(Session(node="compat_node", namespace="/tests/session")) as node:
        sub = node.create_subscriber(all_msgs.String, "chatter", defer=True)
        assert sub.raw_sub.session is node.session
        assert sub.raw_sub.node_name == node.name
        assert sub.raw_sub.namespace == node.namespace
        assert sub.raw_sub.domain_id == node.domain_id
