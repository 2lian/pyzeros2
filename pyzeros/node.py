import asyncio
import os
import uuid
from typing import TypeVar

import asyncio_for_robotics.zenoh as afor
import zenoh

from pyzeros.pub import Pub

from .session import _PySession, library

_MsgType = TypeVar("_MsgType")


class Node:
    def __init__(
        self,
        name: str | None = None,
        session: zenoh.Session | None = None,
        domain_id: int | str | None = None,
        namespace: str = "%",
        defer: bool = False,
        _enclave: str = "%",
        _node_id: str | int | None = None,
        _zenoh_id: str | None = None,
        _entity_id: int | str | None = None,
    ):
        self.session = afor.auto_session(session)
        self.namespace = namespace
        self._enclave = _enclave
        self.domain_id = os.environ.get("ROS_DOMAIN_ID", 0) if domain_id is None else domain_id
        self._zenoh_id = str(self.session.zid()) if _zenoh_id is None else _zenoh_id
        self.name = f"ros_ez_{uuid.uuid4().hex[:8]}" if name is None else name
        if _node_id is None:
            bookkeeper = library.get(self._zenoh_id, None)
            if bookkeeper is None:
                library[self._zenoh_id] = _PySession(self.session)
                self._node_id = 0
            else:
                bookkeeper.entity_counter += 1
                self._node_id = bookkeeper.entity_counter
        else:
            self._node_id = _node_id
        self._entity_id = self._node_id

        self.token: zenoh.LivelinessToken | None = None
        if not defer:
            self.declare()

    async def async_bind(self):
        try:
            if self.token is None:
                self.declare()
            await asyncio.Future()
        finally:
            self.undeclare()

    def create_publisher(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: None = None,
        defer: bool = False,
    ):
        return Pub(
            msg_type,
            topic,
            qos_profile,
            session=self.session,
            domain_id=self.domain_id,
            namespace=self.namespace,
            node_name=self.name,
            defer=defer,
            _enclave=self._enclave,
            _node_id=self._node_id,
        )

    def declare(self):
        self.token = self.declare_token(
            self.name,
            self.session,
            self.domain_id,
            self.namespace,
            self._enclave,
            self._node_id,
            self._zenoh_id,
            self._entity_id,
        )
        return self.token

    def undeclare(self):
        if self.token is None:
            return
        self.token.undeclare()
        self.token = None

    @staticmethod
    def declare_token(
        name: str | None = None,
        session: zenoh.Session | None = None,
        domain_id: int | str | None = None,
        namespace: str = "%",
        _enclave: str = "%",
        _node_id: str | int | None = None,
        _zenoh_id: str | None = None,
        _entity_id: int | str | None = None,
    ):
        ses = afor.auto_session(session)
        if domain_id is None:
            domain_id = os.environ.get("ROS_DOMAIN_ID", 0)
        if _zenoh_id is None:
            _zenoh_id = str(ses.zid())
        if name is None:
            name = f"ros_ez_{uuid.uuid4().hex[:8]}"
        if _entity_id is None:
            bookkeeper = library.get(_zenoh_id, None)
            if bookkeeper is None:
                library[_zenoh_id] = _PySession(ses)
                _entity_id = 0
            else:
                bookkeeper.entity_counter += 1
                _entity_id = bookkeeper.entity_counter
        if _node_id is None:
            _node_id = _entity_id
        tok = ses.liveliness().declare_token(
            "/".join(
                [
                    "@ros2_lv",
                    str(domain_id),
                    _zenoh_id,
                    str(_node_id),
                    str(_entity_id),
                    "NN",
                    _enclave,
                    namespace,
                    name,
                ]
            )
        )
        return tok
