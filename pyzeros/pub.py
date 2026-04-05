import asyncio
import numpy as np
import os
import time
import uuid
from typing import Any, Generic, Literal, Optional, TypeVar

import asyncio_for_robotics.zenoh as afor
import ros_z_py
import zenoh
from ros2_pyterfaces.cyclone.idl import IdlStruct
from ros2_pyterfaces.cydr.idl import types

from .session import _PySession, library
from .utils import Attachment, TopicInfo, ros_type_to_dds_type

_MsgType = TypeVar("_MsgType")


class Pub(Generic[_MsgType]):
    """Small wrapper around a `ros_z_py` publisher.

    `msg_type` can be a native `ros_z_py` message class or a
    `ros2_pyterfaces.idl.IdlStruct` subclass. In the latter case,
    `publish()` serializes the Python object before forwarding it to `ros-z`.
    """

    def __init__(
        self,
        msg_type: type[_MsgType],
        topic: str,
        qos_profile: None = None,
        session: zenoh.Session | None = None,
        domain_id: int | str | None = None,
        namespace: str = "%",
        node_name: str | None = None,
        defer: bool = False,
        _topic_hash: str | None = None,
        _enclave: str = "%",
        _node_id: str | int | None = None,
        _zenoh_id: str | None = None,
        _entity_id: int | str | None = None,
    ):
        self.session = afor.auto_session(session)
        self.namespace = namespace
        self._enclave = _enclave
        self.domain_id = (
            os.environ.get("ROS_DOMAIN_ID", 0) if domain_id is None else domain_id
        )
        self._zenoh_id = str(self.session.zid()) if _zenoh_id is None else _zenoh_id
        self.node_name = (
            f"ros_ez_{uuid.uuid4().hex[:8]}" if node_name is None else node_name
        )
        self._node_id = 0 if _node_id is None else _node_id
        if _entity_id is None:
            bookkeeper = library.get(self._zenoh_id, None)
            if bookkeeper is None:
                library[self._zenoh_id] = _PySession(self.session)
                self._entity_id = 0
            else:
                bookkeeper.entity_counter += 1
                self._entity_id = bookkeeper.entity_counter
        else:
            self._entity_id = _entity_id
        self.dds_type = ros_type_to_dds_type(msg_type.get_type_name())
        self.hash = _topic_hash if _topic_hash is not None else msg_type.hash_rihs01()
        self.topic_info: TopicInfo[type[_MsgType]] = TopicInfo(
            topic=topic, msg_type=msg_type, qos=qos_profile
        )
        self.token: zenoh.LivelinessToken | None = None
        self.zenoh_pub: zenoh.Publisher | None = None
        if defer == False:
            self.declare()

        self.count = 0
        self.gid = np.zeros(16, dtype=np.uint8)

    def publish(self, msg: _MsgType):
        if self.zenoh_pub is None:
            raise ValueError("Publisher not declared.")
        self.zenoh_pub.put(
            msg.serialize(),
            attachment=Attachment(
                sequence_number=types.int64(self.count),
                source_timestamp=types.int64(time.time_ns()),
                source_gid=self.gid,
            ).serialize()[4:],
        )

    async def async_bind(self):
        try:
            if self.token is None:
                self.declare()
            await asyncio.Future()
        finally:
            self.undeclare()

    def declare(self):
        self.token = self.declare_token(
            name=self.topic_info.topic,
            dds_type=self.dds_type,
            hash=self.hash,
            node_name=self.node_name,
            session=self.session,
            domain_id=self.domain_id,
            namespace=self.namespace,
            _enclave=self._enclave,
            _node_id=self._node_id,
            _zenoh_id=self._zenoh_id,
            _entity_id=self._entity_id,
        )
        self.zenoh_pub = self.declare_publisher(
            name=self.topic_info.topic,
            dds_type=self.dds_type,
            hash=self.hash,
            session=self.session,
            domain_id=self.domain_id,
        )

    def undeclare(self):
        if self.token is not None:
            self.token.undeclare()
            self.token = None

    @staticmethod
    def declare_publisher(
        name: str,
        dds_type: str,
        hash: str,
        session: zenoh.Session | None = None,
        domain_id: int | str | None = None,
    ):
        ses = afor.auto_session(session)
        if domain_id is None:
            domain_id = os.environ.get("ROS_DOMAIN_ID", 0)
        pub = ses.declare_publisher(
            "/".join(
                [
                    str(domain_id),
                    name,
                    dds_type,
                    hash,
                ]
            )
        )
        return pub

    @staticmethod
    def declare_token(
        name: str,
        dds_type: str,
        hash: str,
        node_name: str | None = None,
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
        if node_name is None:
            node_name = f"naked_pub_{uuid.uuid4().hex[:8]}"
        if _node_id is None:
            _node_id = 0
        if _entity_id is None:
            bookkeeper = library.get(_zenoh_id, None)
            if bookkeeper is None:
                library[_zenoh_id] = _PySession(ses)
                _entity_id = 0
            else:
                bookkeeper.entity_counter += 1
                _entity_id = bookkeeper.entity_counter
        tok = ses.liveliness().declare_token(
            "/".join(
                [
                    "@ros2_lv",
                    str(domain_id),
                    _zenoh_id,
                    str(_node_id),
                    str(_entity_id),
                    "MP",
                    _enclave,
                    namespace,
                    node_name,
                    namespace + name.replace("/", "%"),
                    dds_type,
                    hash,
                    "::,10:,:,:,,",  # placeholder
                ]
            )
        )
        return tok
