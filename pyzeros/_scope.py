from __future__ import annotations

import asyncio

from asyncio_for_robotics import Scope

_AUTO_SCOPE = object()


class ScopeOwned:
    """Small internal helper for resources owned by an afor scope."""

    def _init_scope(self, scope: Scope | None | object = _AUTO_SCOPE) -> None:
        self._scope: Scope | None = None
        self.lifetime: asyncio.Future[bool] = self._get_loop().create_future()
        self.lifetime.add_done_callback(lambda *_: self.close())
        if scope is _AUTO_SCOPE:
            scope = Scope.current(default=None)
        if scope is not None:
            self.attach(scope)

    def attach(self, scope: Scope) -> None:
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
        return await asyncio.shield(self.lifetime)

    def _set_lifetime_result(self, result: bool = True) -> None:
        if not self.lifetime.done():
            self.lifetime.set_result(result)

    def _set_lifetime_exception(self, exc: BaseException) -> None:
        if not self.lifetime.done():
            self.lifetime.set_exception(exc)

    @staticmethod
    def _get_loop() -> asyncio.AbstractEventLoop:
        try:
            return asyncio.get_running_loop()
        except RuntimeError:
            return asyncio.get_event_loop()

__all__ = ["ScopeOwned", "_AUTO_SCOPE"]
