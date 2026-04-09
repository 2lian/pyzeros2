from dataclasses import dataclass, field

import zenoh

@dataclass
class _PySession:
    session: zenoh.Session
    entity_counter: int = 0
    nodes: list = field(default_factory=list)
    publishers: list = field(default_factory=list)
    subscribers: list = field(default_factory=list)


library: dict[str, _PySession] = {}

