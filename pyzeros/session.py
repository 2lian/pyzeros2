from __future__ import annotations

import contextvars
import warnings
from contextlib import contextmanager
from typing import Any, Generator, TYPE_CHECKING

import asyncio_for_robotics.zenoh as afor
import zenoh

if TYPE_CHECKING:
    from .node import Node


_CURRENT_SESSION: contextvars.ContextVar[Node | None] = contextvars.ContextVar(
    "pyzeros_current_session",
    default=None,
)


def Session(
    node: str | Node | None = None,
    zenoh_session: zenoh.Session | None = None,
    namespace: str = "%",
    domain_id: int | str | None = None,
) -> Node:
    """Create a PyZeROS node/session.

    Prefer :func:`auto_context` for normal lifecycle management. If *node* is
    already a :class:`Node`, it is returned unchanged.
    """
    from .node import Node

    session: Node | None
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


def current_session() -> Node:
    """Return the current lexical PyZeROS session."""
    session = _CURRENT_SESSION.get()
    if session is None:
        raise RuntimeError("No active PyZeROS session context")
    return session


@contextmanager
def auto_context(
    node: str | Node | None = None,
    zenoh_session: zenoh.Session | None = None,
    namespace: str = "%",
    domain_id: int | str | None = None,
) -> Generator[Node, Any, Any]:
    """Bind the default PyZeROS session for this block.

    Passing a :class:`Node` binds it without taking ownership. Passing a node
    name or no node creates a Node and closes it on exit. Likewise, an
    explicit Zenoh session remains caller-owned; otherwise the underlying
    afor Zenoh context owns and closes its transport.
    """
    from .node import Node

    if isinstance(node, Node):
        if zenoh_session is not None and zenoh_session is not node.session:
            raise ValueError(
                "The provided node is already bound to another zenoh session."
            )
        session = node
        close_on_exit = False
        transport = node.session
    else:
        session = None
        close_on_exit = True
        transport = zenoh_session

    with afor.auto_context(transport) as active_transport:
        if session is None:
            session = Session(
                node=node,
                zenoh_session=active_transport,
                namespace=namespace,
                domain_id=domain_id,
            )

        assert session is not None
        token = _CURRENT_SESSION.set(session)
        try:
            yield session
        finally:
            _CURRENT_SESSION.reset(token)
            if close_on_exit:
                session.close()


def auto_session(session: Node | None = None) -> Node:
    """Use an explicit or current session, creating a warned fallback.

    Resolution order:
        1. Explicit *session* argument.
        2. Current lexical session context.
        3. Create a context-local fallback session.
    """
    if session is not None:
        return session
    try:
        return current_session()
    except RuntimeError:
        warnings.warn(
            "A PyZeROS session was never declared. A fallback one is now "
            "instantiated. Prefer entering a context using "
            "`with auto_context(node=...)`",
            stacklevel=2,
        )
        session = Session()
        _CURRENT_SESSION.set(session)
        return session
