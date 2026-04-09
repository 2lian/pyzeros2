import os
import logging
from dataclasses import dataclass, field
from enum import StrEnum
from typing import (
    Dict,
    Generic,
    Literal,
    Tuple,
    Any,
)

import asyncio_for_robotics.zenoh as afor
import msgspec
import numpy as np
import zenoh
from asyncio_for_robotics.core.sub import _MsgType
from nptyping import Int8, NDArray, Shape, UInt8
from ros2_pyterfaces.cydr.idl import IdlStruct, types

from .qos import QosProfile
from .session import _PySession, library

logger = logging.getLogger(__name__)
import sys

import xxhash

_MASK64 = (1 << 64) - 1


def rmw_zenoh_gid(keyexpr: str | bytes) -> bytes:
    """
    Reproduce the current rmw_zenoh publisher/client/service GID generation.

    Upstream logic:
      gid = XXH3_128(exact_liveliness_keyexpr_utf8_bytes)
      bytes = memcpy(low64) + memcpy(high64)

    Notes:
    - The input must be the exact liveliness token string.
    - Any mismatch in mangling, QoS encoding, type hash, etc. changes the GID.
    - We pack each 64-bit lane using native endianness to mirror upstream memcpy.
      On your Linux/x86_64 machine this is little-endian.
    """
    if isinstance(keyexpr, str):
        data = keyexpr.encode("utf-8")
    else:
        data = keyexpr

    h128 = xxhash.xxh3_128(data).intdigest()

    low64 = h128 & _MASK64
    high64 = (h128 >> 64) & _MASK64

    return low64.to_bytes(8, byteorder=sys.byteorder) + high64.to_bytes(
        8, byteorder=sys.byteorder
    )


def ros_type_to_dds_type(ros_type: str) -> str:
    parts = ros_type.strip("/").split("/")
    if len(parts) != 3:
        raise ValueError(
            f"Expected 'package/msg/Type' or 'package/srv/Type', got: {ros_type!r}"
        )

    package, interface_kind, type_name = parts
    if interface_kind not in {"msg", "srv", "action"}:
        raise ValueError(
            f"Unsupported interface kind {interface_kind!r} in {ros_type!r}"
        )

    return f"{package}::{interface_kind}::dds_::{type_name}_"


def resolve_liveliness_identity(
    session: zenoh.Session | None = None,
    domain_id: int | str | None = None,
    _node_id: str | int | None = None,
    _zenoh_id: str | None = None,
    _entity_id: int | str | None = None,
    *,
    node_id_from_entity: bool = False,
) -> tuple[int | str, str, str | int, str | int]:
    """Resolve the shared identity fields used in ROS liveliness keyexprs."""
    ses = afor.auto_session(session)
    if domain_id is None:
        domain_id = os.environ.get("ROS_DOMAIN_ID", 0)
    if _zenoh_id is None:
        _zenoh_id = str(ses.zid())
    if _entity_id is None:
        bookkeeper = library.get(_zenoh_id, None)
        if bookkeeper is None:
            library[_zenoh_id] = _PySession(ses)
            _entity_id = 0
        else:
            bookkeeper.entity_counter += 1
            _entity_id = bookkeeper.entity_counter
    if _node_id is None:
        _node_id = _entity_id if node_id_from_entity else 0
    return domain_id, _zenoh_id, _node_id, _entity_id


@dataclass(frozen=True, slots=True)
class LivelinessContext:
    """Resolved runtime context shared by node, publisher, and subscriber objects."""

    session: zenoh.Session
    domain_id: int | str
    namespace: str
    enclave: str
    zenoh_id: str
    node_id: str | int
    entity_id: str | int


def normalize_namespace(namespace: str) -> str:
    """Normalize a namespace string for root-namespace handling."""
    if namespace in {"", "%"}:
        return "/"
    if not namespace.startswith("/"):
        namespace = f"/{namespace}"
    if namespace != "/":
        namespace = namespace.removesuffix("/")
    return namespace


def resolve_liveliness_context(
    session: zenoh.Session | None = None,
    domain_id: int | str | None = None,
    namespace: str = "%",
    _enclave: str = "%",
    _node_id: str | int | None = None,
    _zenoh_id: str | None = None,
    _entity_id: int | str | None = None,
    *,
    node_id_from_entity: bool = False,
) -> LivelinessContext:
    """Resolve the shared constructor state for ROS entities backed by Zenoh."""
    ses = afor.auto_session(session)
    domain_id, _zenoh_id, _node_id, _entity_id = resolve_liveliness_identity(
        session=ses,
        domain_id=domain_id,
        _node_id=_node_id,
        _zenoh_id=_zenoh_id,
        _entity_id=_entity_id,
        node_id_from_entity=node_id_from_entity,
    )
    return LivelinessContext(
        session=ses,
        domain_id=domain_id,
        namespace=normalize_namespace(namespace),
        enclave=_enclave,
        zenoh_id=_zenoh_id,
        node_id=_node_id,
        entity_id=_entity_id,
    )


def mangle_liveliness_topic(name: str, namespace: str) -> tuple[str, str]:
    """Encode namespace and topic path segments for ROS liveliness keyexprs."""
    name = name.removesuffix("/")
    namespace = normalize_namespace(namespace)
    if name[0] == "/":
        qualified_name = name
        encoded_namespace = "/"
    elif namespace == "/":
        qualified_name = f"/{name}"
        encoded_namespace = namespace
    else:
        qualified_name = f"{namespace.removesuffix('/')}/{name}"
        encoded_namespace = namespace
    return encoded_namespace.replace("/", "%"), qualified_name.replace("/", "%")


class CdrModes(StrEnum):
    """Select how message bytes are encoded or decoded."""

    AUTO = "auto"
    ROS_Z = "ros_z"
    PYTERFACE = "pyterface"


def is_ros2pyterfaces(msg_type: type) -> bool:
    return (
        getattr(msg_type, "serialize", None) is not None
        and getattr(msg_type, "deserialize", None) is not None
        and getattr(msg_type, "get_type_name", None) is not None
        and getattr(msg_type, "hash_rihs01", None) is not None
    )


def deduce_cdr_mode(
    msg_type: type[_MsgType], cdr_mode: CdrModes
) -> Literal[CdrModes.ROS_Z, CdrModes.PYTERFACE]:
    """Resolve `AUTO` into the concrete serialization mode for `msg_type`."""
    if cdr_mode != CdrModes.AUTO:
        return cdr_mode
    if is_ros2pyterfaces(msg_type):
        return CdrModes.PYTERFACE
    else:
        return CdrModes.ROS_Z


def get_type_shim(msg_type: Any, cdr_mode: CdrModes = CdrModes.AUTO):
    """Return the class object that should be passed to the underlying binding.

    Native binding message classes are returned unchanged. `IdlStruct` classes
    are converted into a shim exposing the ROS type metadata expected by the
    transport layer.
    """
    cdr_mode = deduce_cdr_mode(msg_type, cdr_mode)

    if cdr_mode == CdrModes.AUTO:
        raise ValueError()
    elif cdr_mode == CdrModes.ROS_Z:
        type_dummy = msg_type
    elif cdr_mode == CdrModes.PYTERFACE:
        type_dummy = make_ros_z_shim_type(msg_type)
    else:
        raise ValueError()

    return type_dummy


def make_ros_z_shim_type(msg_type: Any) -> type[Any]:
    """Build a lightweight shim class exposing `__msgtype__` and `__hash__`."""
    return type(
        f"{msg_type.get_type_name().replace("/", "__")}_RosZShim",
        (),
        {
            "__msgtype__": msg_type.get_type_name(),
            "__hash__": msg_type.hash_rihs01(),
        },
    )


@dataclass(frozen=True, slots=True)
class TopicInfo(Generic[_MsgType]):
    """Container for a topic name, its message class, and its QoS profile."""

    topic: str
    msg_type: _MsgType
    qos: QosProfile = field(default_factory=lambda *_, **__: None)

    def as_arg(self) -> Tuple[_MsgType, str, QosProfile]:
        return (self.msg_type, self.topic, self.qos)

    def as_kwarg(self) -> Dict[str, Any]:
        return {"msg_type": self.msg_type, "topic": self.topic, "qos_profile": self.qos}
