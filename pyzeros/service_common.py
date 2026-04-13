from __future__ import annotations

import uuid
from typing import Literal, Protocol, TypeVar

import zenoh

from pyzeros.qos import QosProfile
from pyzeros.utils import normalize_namespace, resolve_liveliness_identity

_ReqT = TypeVar("_ReqT")
_ResT = TypeVar("_ResT")
_EvnT = TypeVar("_EvnT")


class ServiceType(Protocol[_ReqT, _ResT, _EvnT]):
    """Protocol for ROS service types (e.g. ``AddTwoInts``, ``Trigger``).

    Any class that exposes ``Request``, ``Response``, ``get_type_name()``,
    and ``hash_rihs01()`` satisfies this protocol.  Standard service types
    from ``ros2_pyterfaces`` implement it out of the box.
    """

    Request: type[_ReqT]
    Response: type[_ResT]

    @classmethod
    def get_type_name(cls) -> str: ...

    @classmethod
    def hash_rihs01(cls) -> str: ...


def _is_valid_name_component(component: str) -> bool:
    if component == "":
        return False
    first = component[0]
    if not (first.isalpha() or first == "_"):
        return False
    return all(char.isalnum() or char == "_" for char in component[1:])


def qualify_service_name(name: str, namespace: str, node_name: str) -> str:
    """Expand a service name to its absolute form following ROS 2 rules.

    - **Absolute** (``/add``) — returned as-is.
    - **Relative** (``add``) — prepended with *namespace*.
    - **Private** (``~/add``) — prepended with *namespace* + *node_name*.

    Args:
        name: Service name, relative, absolute, or private (``~`` prefix).
        namespace: Node namespace (e.g. ``/demo``).
        node_name: Node name (e.g. ``my_node``).

    Returns:
        The fully qualified service name (always starts with ``/``).

    Raises:
        ValueError: If name, namespace, or node_name are invalid.
    """
    if name == "":
        raise ValueError("Service name is empty.")
    if node_name == "" or not _is_valid_name_component(node_name):
        raise ValueError(f"Invalid node name: {node_name!r}")

    if namespace not in ("", "/"):
        if namespace.endswith("/"):
            raise ValueError(f"Invalid namespace ending with '/': {namespace!r}")
        for part in namespace.split("/"):
            if part != "" and not _is_valid_name_component(part):
                raise ValueError(f"Invalid namespace component: {part!r}")

    normalized_namespace = namespace
    if normalized_namespace != "" and not normalized_namespace.startswith("/"):
        normalized_namespace = f"/{normalized_namespace}"

    if name.startswith("/"):
        qualified_name = name.removesuffix("/")
        if qualified_name in ("", "/"):
            raise ValueError("Service name cannot be '/'.")
        return qualified_name

    if name.startswith("~"):
        suffix = name.removeprefix("~").removeprefix("/")
        if suffix != "":
            for part in suffix.split("/"):
                if part != "" and not _is_valid_name_component(part):
                    raise ValueError(f"Invalid private service component: {part!r}")
        if normalized_namespace in ("", "/"):
            return f"/{node_name}" if suffix == "" else f"/{node_name}/{suffix}"
        return (
            f"{normalized_namespace}/{node_name}"
            if suffix == ""
            else f"{normalized_namespace}/{node_name}/{suffix}"
        )

    qualified_name = name.removesuffix("/")
    for part in qualified_name.split("/"):
        if part != "" and not _is_valid_name_component(part):
            raise ValueError(f"Invalid service component: {part!r}")
    if normalized_namespace in ("", "/"):
        return f"/{qualified_name}"
    return f"{normalized_namespace}/{qualified_name}"


def token_keyexpr(
    entity_kind: Literal["SC", "SS"],
    name: str,
    dds_type: str,
    hash: str,
    qos_profile: QosProfile | None = None,
    node_name: str | None = None,
    session: zenoh.Session | None = None,
    domain_id: int | str | None = None,
    namespace: str = "%",
    _enclave: str = "%",
    _node_id: str | int | None = None,
    _zenoh_id: str | None = None,
    _entity_id: int | str | None = None,
) -> str:
    """Build the ROS graph liveliness token for a service client or server.

    Args:
        entity_kind: `"SC"` for a service client or `"SS"` for a service
            server.
        name: Fully qualified ROS service name.
        dds_type: DDS type name associated with the service.
        hash: ROS type hash associated with the service.
        qos_profile: QoS profile to encode into the token.
        node_name: Node name to advertise in the ROS graph token.
        session: Zenoh session used to derive the Zenoh id when needed.
        domain_id: ROS domain id to encode in the token.
        namespace: Node namespace to advertise in the token.
        _enclave: Internal enclave segment.
        _node_id: Internal node id override.
        _zenoh_id: Internal Zenoh id override.
        _entity_id: Internal entity id override.

    Returns:
        The complete ROS graph liveliness token key expression.
    """
    qos_profile = QosProfile.default() if qos_profile is None else qos_profile.normalized()
    if node_name is None:
        node_name = f"naked_service_{uuid.uuid4().hex[:8]}"
    domain_id, _zenoh_id, _node_id, _entity_id = resolve_liveliness_identity(
        session=session,
        domain_id=domain_id,
        _node_id=_node_id,
        _zenoh_id=_zenoh_id,
        _entity_id=_entity_id,
    )
    encoded_namespace = normalize_namespace(namespace).replace("/", "%")
    encoded_name = name.replace("/", "%")
    return "/".join(
        [
            "@ros2_lv",
            str(domain_id),
            _zenoh_id,
            str(_node_id),
            str(_entity_id),
            entity_kind,
            _enclave,
            encoded_namespace,
            node_name,
            encoded_name,
            dds_type,
            hash,
            qos_profile.encode(),
        ]
    )
