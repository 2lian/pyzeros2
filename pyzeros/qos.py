from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Final

import zenoh

DEFAULT_HISTORY_DEPTH: Final[int] = 10
RMW_ZENOH_DEFAULT_HISTORY_DEPTH: Final[int] = 42
RMW_DURATION_INFINITE_SEC: Final[int] = 9223372036
RMW_DURATION_INFINITE_NSEC: Final[int] = 854775807


class ReliabilityPolicy(IntEnum):
    """ROS 2 reliability policy values."""

    SYSTEM_DEFAULT = 0
    RELIABLE = 1
    BEST_EFFORT = 2
    UNKNOWN = 3
    BEST_AVAILABLE = 4


class HistoryPolicy(IntEnum):
    """ROS 2 history policy values."""

    SYSTEM_DEFAULT = 0
    KEEP_LAST = 1
    KEEP_ALL = 2
    UNKNOWN = 3


class DurabilityPolicy(IntEnum):
    """ROS 2 durability policy values."""

    SYSTEM_DEFAULT = 0
    TRANSIENT_LOCAL = 1
    VOLATILE = 2
    UNKNOWN = 3
    BEST_AVAILABLE = 4


class LivelinessPolicy(IntEnum):
    """ROS 2 liveliness policy values."""

    SYSTEM_DEFAULT = 0
    AUTOMATIC = 1
    MANUAL_BY_NODE = 2
    MANUAL_BY_TOPIC = 3
    UNKNOWN = 4
    BEST_AVAILABLE = 5


@dataclass(frozen=True, slots=True)
class QosDuration:
    """QoS duration represented as seconds and nanoseconds."""

    sec: int = RMW_DURATION_INFINITE_SEC
    nsec: int = RMW_DURATION_INFINITE_NSEC

    def __post_init__(self):
        if self.sec < 0:
            raise ValueError("Duration seconds must be non-negative.")
        if not 0 <= self.nsec <= 999_999_999:
            raise ValueError("Duration nanoseconds must be between 0 and 999999999.")

    @classmethod
    def from_seconds(cls, secs: float | int) -> "QosDuration":
        """Build a duration from a floating-point second count."""
        if secs < 0:
            raise ValueError("Duration seconds must be non-negative.")
        sec = int(secs)
        nsec = int(round((float(secs) - sec) * 1_000_000_000))
        if nsec == 1_000_000_000:
            sec += 1
            nsec = 0
        return cls(sec=sec, nsec=nsec)


QosDuration.INFINITE = QosDuration()


def _coerce_policy(
    value: Any, enum_type: type[IntEnum], aliases: dict[str, IntEnum]
) -> IntEnum:
    if isinstance(value, enum_type):
        return value
    if isinstance(value, str):
        key = value.strip().lower()
        try:
            return aliases[key]
        except KeyError as exc:
            raise ValueError(f"Invalid {enum_type.__name__}: {value!r}") from exc
    if isinstance(value, int):
        try:
            return enum_type(value)
        except ValueError as exc:
            raise ValueError(f"Invalid {enum_type.__name__}: {value!r}") from exc
    raise TypeError(f"Unsupported {enum_type.__name__} value: {value!r}")


def _coerce_duration(value: Any) -> QosDuration:
    if isinstance(value, QosDuration):
        return value
    if isinstance(value, (int, float)):
        return QosDuration.from_seconds(value)
    if isinstance(value, tuple) and len(value) == 2:
        return QosDuration(sec=int(value[0]), nsec=int(value[1]))
    raise TypeError(f"Unsupported QosDuration value: {value!r}")


@dataclass(frozen=True, slots=True)
class QosProfile:
    """ROS 2 QoS profile for publishers and subscribers."""

    history: HistoryPolicy | str | int = HistoryPolicy.KEEP_LAST
    depth: int = DEFAULT_HISTORY_DEPTH
    reliability: ReliabilityPolicy | str | int = ReliabilityPolicy.RELIABLE
    durability: DurabilityPolicy | str | int = DurabilityPolicy.VOLATILE
    deadline: QosDuration | tuple[int, int] | float | int = field(
        default_factory=lambda: QosDuration.INFINITE
    )
    lifespan: QosDuration | tuple[int, int] | float | int = field(
        default_factory=lambda: QosDuration.INFINITE
    )
    liveliness: LivelinessPolicy | str | int = LivelinessPolicy.AUTOMATIC
    liveliness_lease_duration: QosDuration | tuple[int, int] | float | int = field(
        default_factory=lambda: QosDuration.INFINITE
    )
    avoid_ros_namespace_conventions: bool = False

    def __post_init__(self):
        history = _coerce_policy(
            self.history,
            HistoryPolicy,
            {
                "system_default": HistoryPolicy.SYSTEM_DEFAULT,
                "keep_last": HistoryPolicy.KEEP_LAST,
                "keep_all": HistoryPolicy.KEEP_ALL,
                "unknown": HistoryPolicy.UNKNOWN,
            },
        )
        reliability = _coerce_policy(
            self.reliability,
            ReliabilityPolicy,
            {
                "system_default": ReliabilityPolicy.SYSTEM_DEFAULT,
                "reliable": ReliabilityPolicy.RELIABLE,
                "best_effort": ReliabilityPolicy.BEST_EFFORT,
                "unknown": ReliabilityPolicy.UNKNOWN,
                "best_available": ReliabilityPolicy.BEST_AVAILABLE,
            },
        )
        durability = _coerce_policy(
            self.durability,
            DurabilityPolicy,
            {
                "system_default": DurabilityPolicy.SYSTEM_DEFAULT,
                "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
                "volatile": DurabilityPolicy.VOLATILE,
                "unknown": DurabilityPolicy.UNKNOWN,
                "best_available": DurabilityPolicy.BEST_AVAILABLE,
            },
        )
        liveliness = _coerce_policy(
            self.liveliness,
            LivelinessPolicy,
            {
                "system_default": LivelinessPolicy.SYSTEM_DEFAULT,
                "automatic": LivelinessPolicy.AUTOMATIC,
                "manual_by_node": LivelinessPolicy.MANUAL_BY_NODE,
                "manual_by_topic": LivelinessPolicy.MANUAL_BY_TOPIC,
                "unknown": LivelinessPolicy.UNKNOWN,
                "best_available": LivelinessPolicy.BEST_AVAILABLE,
            },
        )
        depth = int(self.depth)
        if history == HistoryPolicy.KEEP_LAST and depth <= 0:
            depth = DEFAULT_HISTORY_DEPTH
        if history == HistoryPolicy.KEEP_ALL and depth < 0:
            raise ValueError("QoS depth must be non-negative.")

        object.__setattr__(self, "history", history)
        object.__setattr__(self, "depth", depth)
        object.__setattr__(self, "reliability", reliability)
        object.__setattr__(self, "durability", durability)
        object.__setattr__(self, "deadline", _coerce_duration(self.deadline))
        object.__setattr__(self, "lifespan", _coerce_duration(self.lifespan))
        object.__setattr__(self, "liveliness", liveliness)
        object.__setattr__(
            self,
            "liveliness_lease_duration",
            _coerce_duration(self.liveliness_lease_duration),
        )

    @staticmethod
    def default() -> "QosProfile":
        """Default ROS topic QoS."""
        return QosProfile()

    @staticmethod
    def sensor_data() -> "QosProfile":
        """QoS preset for lossy sensor streams."""
        return QosProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

    @staticmethod
    def parameters() -> "QosProfile":
        """QoS preset for parameter traffic."""
        return QosProfile(depth=1000)

    @staticmethod
    def services() -> "QosProfile":
        """QoS preset for service traffic."""
        return QosProfile(depth=10)

    @staticmethod
    def rmw_zenoh_default() -> "QosProfile":
        """The normalization baseline used by rmw_zenoh token encoding."""
        return QosProfile(depth=RMW_ZENOH_DEFAULT_HISTORY_DEPTH)

    def normalized(self) -> "QosProfile":
        """Resolve system-default and best-available markers into concrete values."""
        history = self.history
        if history in (HistoryPolicy.SYSTEM_DEFAULT, HistoryPolicy.UNKNOWN):
            history = HistoryPolicy.KEEP_LAST

        reliability = self.reliability
        if reliability in (
            ReliabilityPolicy.SYSTEM_DEFAULT,
            ReliabilityPolicy.UNKNOWN,
            ReliabilityPolicy.BEST_AVAILABLE,
        ):
            reliability = ReliabilityPolicy.RELIABLE

        durability = self.durability
        if durability in (
            DurabilityPolicy.SYSTEM_DEFAULT,
            DurabilityPolicy.UNKNOWN,
            DurabilityPolicy.BEST_AVAILABLE,
        ):
            durability = DurabilityPolicy.VOLATILE

        liveliness = self.liveliness
        if liveliness in (
            LivelinessPolicy.SYSTEM_DEFAULT,
            LivelinessPolicy.UNKNOWN,
            LivelinessPolicy.BEST_AVAILABLE,
        ):
            liveliness = LivelinessPolicy.AUTOMATIC

        depth = self.depth
        if history == HistoryPolicy.KEEP_LAST and depth <= 0:
            depth = DEFAULT_HISTORY_DEPTH

        return QosProfile(
            history=history,
            depth=depth,
            reliability=reliability,
            durability=durability,
            deadline=self.deadline,
            lifespan=self.lifespan,
            liveliness=liveliness,
            liveliness_lease_duration=self.liveliness_lease_duration,
            avoid_ros_namespace_conventions=self.avoid_ros_namespace_conventions,
        )

    def validate_runtime_support(self) -> None:
        """Reject QoS features that pyzeros does not implement yet."""
        if self.durability != DurabilityPolicy.VOLATILE:
            raise NotImplementedError(
                "TRANSIENT_LOCAL durability is not implemented in pyzeros yet."
            )

    def encode(self) -> str:
        """Encode this QoS profile in rmw_zenoh liveliness-token format."""
        qos = self.normalized()
        default_qos = QosProfile.rmw_zenoh_default()

        reliability = (
            ""
            if qos.reliability == default_qos.reliability
            else str(int(qos.reliability))
        )
        durability = (
            "" if qos.durability == default_qos.durability else str(int(qos.durability))
        )

        history_kind = (
            "" if qos.history == default_qos.history else str(int(qos.history))
        )
        history_depth = ""
        if qos.history == HistoryPolicy.KEEP_LAST and qos.depth != default_qos.depth:
            history_depth = str(qos.depth)

        deadline = (
            (
                "",
                "",
            )
            if qos.deadline == default_qos.deadline
            else (str(qos.deadline.sec), str(qos.deadline.nsec))
        )
        lifespan = (
            (
                "",
                "",
            )
            if qos.lifespan == default_qos.lifespan
            else (str(qos.lifespan.sec), str(qos.lifespan.nsec))
        )

        liveliness_kind = (
            "" if qos.liveliness == default_qos.liveliness else str(int(qos.liveliness))
        )
        lease = (
            (
                "",
                "",
            )
            if qos.liveliness_lease_duration == default_qos.liveliness_lease_duration
            else (
                str(qos.liveliness_lease_duration.sec),
                str(qos.liveliness_lease_duration.nsec),
            )
        )

        return (
            f"{reliability}:{durability}:{history_kind},{history_depth}:"
            f"{deadline[0]},{deadline[1]}:{lifespan[0]},{lifespan[1]}:"
            f"{liveliness_kind},{lease[0]},{lease[1]}"
        )

    def publisher_options(self) -> dict[str, object]:
        """Map this profile to Zenoh publisher declaration options."""
        self.validate_runtime_support()
        options: dict[str, object] = {
            "reliability": (
                zenoh.Reliability.RELIABLE
                if self.reliability == ReliabilityPolicy.RELIABLE
                else zenoh.Reliability.BEST_EFFORT
            ),
            "congestion_control": zenoh.CongestionControl.DROP,
        }
        if (
            self.reliability == ReliabilityPolicy.RELIABLE
            and self.history == HistoryPolicy.KEEP_ALL
        ):
            options["congestion_control"] = zenoh.CongestionControl.BLOCK
        return options

    def subscriber_options(self) -> dict[str, object]:
        """Map this profile to Zenoh subscriber declaration options."""
        self.validate_runtime_support()
        return {}

    @property
    def subscriber_queue_size(self) -> int:
        """Local typed-subscriber queue size derived from history settings."""
        if self.history == HistoryPolicy.KEEP_ALL:
            return 0
        return self.depth
