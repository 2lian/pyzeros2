from __future__ import annotations

import contextvars
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Any, Generator, TYPE_CHECKING

import asyncio_for_robotics.zenoh as afor
import zenoh

if TYPE_CHECKING:
    from .node import Node


@dataclass
class _PySession:
    session: zenoh.Session
    entity_counter: int = 0
    nodes: list = field(default_factory=list)
    publishers: list = field(default_factory=list)
    subscribers: list = field(default_factory=list)


library: dict[str, _PySession] = {}


def Session(
    node: str | Node | None = None,
    zenoh_session: zenoh.Session | None = None,
    namespace: str = "%",
    domain_id: int | str | None = None,
) -> Node:
    """Create or reuse a node-backed PyZeROS session.

    A PyZeROS "session" is just a `Node`: it already carries the Zenoh
    session, namespace, domain id, and node identity needed by resources.
    """
    from .node import Node

    if isinstance(node, Node):
        if zenoh_session is not None and zenoh_session is not node.session:
            raise ValueError(
                "The provided node is already bound to another zenoh session."
            )
        return node
    return Node(
        name=node,
        session=zenoh_session if zenoh_session is not None else afor.auto_session(),
        namespace=namespace,
        domain_id=domain_id,
    )


GLOBAL_SESSION: Node | None = None

_MISSING = object()
_CURRENT_SESSION: contextvars.ContextVar[Node | None] = contextvars.ContextVar(
    "pyzeros_current_session",
    default=None,
)


def current_session(_default=_MISSING) -> Node:
    """Return the current lexical PyZeROS session node."""
    session = _CURRENT_SESSION.get()
    if session is None:
        if _default is _MISSING:
            raise RuntimeError("No active pyzeros session context")
        return _default
    return session


@contextmanager
def session_context(
    session: Node,
    close_on_exit: bool = True,
) -> Generator[Node, Any, Any]:
    """Bind a session node lexically for the duration of this block."""
    token = _CURRENT_SESSION.set(session)
    try:
        yield session
    finally:
        _CURRENT_SESSION.reset(token)
        if close_on_exit:
            session.undeclare()


@contextmanager
def auto_context(
    node: str | Node | None = None,
    zenoh_session: zenoh.Session | None = None,
    namespace: str = "%",
    domain_id: int | str | None = None,
) -> Generator[Node, Any, Any]:
    """Reuse the current lexical session node or create one for this block."""
    session = current_session(_default=None)
    if session is not None:
        yield session
        return

    session = Session(
        node=node,
        zenoh_session=zenoh_session,
        namespace=namespace,
        domain_id=domain_id,
    )
    with session_context(session) as active:
        yield active


def auto_session(session: Node | None = None) -> Node:
    """Return or create a PyZeROS session node."""
    global GLOBAL_SESSION

    if session is not None:
        return session

    current = current_session(_default=None)
    if current is not None:
        return current

    if GLOBAL_SESSION is not None and GLOBAL_SESSION.token is not None:
        return GLOBAL_SESSION

    GLOBAL_SESSION = Session()
    return GLOBAL_SESSION


