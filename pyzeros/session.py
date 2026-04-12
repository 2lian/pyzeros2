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
    """Create a PyZeROS session (which is just a ``Node``).

    A session carries the Zenoh transport, ROS namespace, domain id, and node
    identity. Every ``Pub``, ``Sub``, ``Client``, ``Server`` created inside
    a ``session_context`` resolves to it automatically.

    If *node* is already a ``Node``, it is returned as-is (useful for
    pass-through in generic code).

    Args:
        node: Node name for the ROS graph, or an existing ``Node`` to reuse.
            ``None`` generates a random name.
        zenoh_session: Explicit Zenoh session.  When omitted, one is obtained
            from ``afor.auto_session()``.
        namespace: ROS namespace advertised by the node.
        domain_id: ROS domain id.  Defaults to ``$ROS_DOMAIN_ID`` or ``0``.

    Returns:
        A ``Node`` ready to be used with ``session_context``.

    Example::

        with session_context(Session(node="talker", namespace="/demo")) as node:
            pub = Pub(String, "chatter", session=node)
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
    """Return the session node bound by the nearest ``session_context``.

    Args:
        _default: Value returned when no session is active.  If omitted,
            ``RuntimeError`` is raised instead.

    Raises:
        RuntimeError: No ``session_context`` is active and no default given.
    """
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
    """Bind *session* as the active session for this block.

    Entities created inside the block resolve to this session automatically
    (via ``auto_session``).  On exit the session's liveliness token is
    undeclared unless *close_on_exit* is ``False``.

    Nesting is supported: inner contexts shadow outer ones and restore them
    on exit.

    Args:
        session: The session node to activate.
        close_on_exit: Whether to call ``session.undeclare()`` when leaving
            the block.  Set to ``False`` when the caller manages the session
            lifetime separately.

    Example::

        with session_context(Session(node="my_node")) as node:
            pub = Pub(String, "topic")  # auto-resolves to node
    """
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
    """Reuse the current lexical session or create one for this block.

    If a ``session_context`` is already active, its session is yielded and
    nothing is closed on exit.  Otherwise a new session is created from the
    provided arguments and bound as a ``session_context`` for the block.

    This is the recommended entry-point for application code::

        with auto_context(node="listener", namespace="/demo"):
            asyncio.run(main())

    Args:
        node: Forwarded to ``Session()`` when creating a new session.
        zenoh_session: Forwarded to ``Session()``.
        namespace: Forwarded to ``Session()``.
        domain_id: Forwarded to ``Session()``.
    """
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
    """Resolve a session node, creating one as a last resort.

    Resolution order:
        1. Explicit *session* argument (pass-through).
        2. Current ``session_context`` (lexical).
        3. ``GLOBAL_SESSION`` module-level singleton.
        4. Auto-create a new ``GLOBAL_SESSION``.

    This is the function that ``Pub``, ``Sub``, ``Client``, and ``Server``
    call internally when no explicit session is provided.
    """
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


