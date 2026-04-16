from __future__ import annotations

import asyncio

from asyncio_for_robotics import Scope
from asyncio_for_robotics.core.sub import _AUTO_SCOPE


class ScopeOwned:
    """Mixin for pyzeros resources that attach to an ``afor.Scope``.

    ``Pub``, ``RawSub``, and ``Client`` inherit from this.  ``Server`` and
    ``Sub`` inherit from ``BaseSub`` instead (which has the same scope
    mechanics built in).

    The mixin provides:

    - ``lifetime`` future — resolves when the resource is closed or fails.
    - ``attach(scope)`` — registers cleanup on the scope's exit stack and
      creates a watcher task so scope teardown propagates here.
    - Auto-attach: if ``scope`` is ``_AUTO_SCOPE`` at init time, the current
      ``afor.Scope`` is used (or ``None`` if none is active).
    """

    def _init_scope(self, scope: Scope | None | object = _AUTO_SCOPE) -> None:
        """Initialize scope tracking.  Call this from ``__init__``."""
        self._scope: Scope | None = None
        self.lifetime: asyncio.Future[bool] = self._get_loop().create_future()
        self.lifetime.add_done_callback(lambda *_: self.close())
        if scope is _AUTO_SCOPE:
            scope = Scope.current(default=None)
        if scope is not None:
            self.attach(scope)

    def attach(self, scope: Scope) -> None:
        """Bind this resource to *scope* for automatic cleanup.

        Registers ``self.close`` on the scope's exit stack and creates a
        watcher task that propagates lifetime failures into the scope's
        ``TaskGroup``.

        Raises:
            RuntimeError: If already attached to a scope.
        """
        if self._scope is not None:
            raise RuntimeError(
                f"{type(self).__name__} is already attached to an afor.Scope"
            )
        self._scope = scope
        assert scope.exit_stack is not None
        assert scope.task_group is not None
        scope.exit_stack.callback(self.close)
        scope.task_group.create_task(self._watch_lifetime())

    async def _watch_lifetime(self) -> bool:
        """Task that awaits ``lifetime`` inside the scope's TaskGroup."""
        return await asyncio.shield(self.lifetime)

    def _set_lifetime_result(self, result: bool = True) -> None:
        """Resolve ``lifetime`` with a normal result.  Idempotent."""
        if not self.lifetime.done():
            self.lifetime.set_result(result)

    def _set_lifetime_exception(self, exc: BaseException) -> None:
        """Resolve ``lifetime`` with an exception.  Idempotent."""
        if not self.lifetime.done():
            self.lifetime.set_exception(exc)

    @staticmethod
    def _get_loop() -> asyncio.AbstractEventLoop:
        try:
            return asyncio.get_running_loop()
        except RuntimeError:
            return asyncio.get_event_loop()


__all__ = ["ScopeOwned", "_AUTO_SCOPE"]
