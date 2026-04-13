# PyZeROS

Python-only ROS 2. No `rclpy`, no ROS installation, no message compilation. Just `pip install` and go.

Built on [Zenoh](https://zenoh.io/), [`asyncio-for-robotics`](https://github.com/2lian/asyncio-for-robotics), and [`ros2_pyterfaces`](https://github.com/2lian/ros2_pyterfaces).

```python
import asyncio, pyzeros
import asyncio_for_robotics as afor
from ros2_pyterfaces.cyclone.all_msgs import String

@afor.scoped
async def main():
    sub = pyzeros.Sub(String, "chatter")
    async for msg in sub.listen_reliable():
        print(msg.data)

with pyzeros.auto_context(node="listener", namespace="/demo"):
    asyncio.run(main())
```

Features:
- Topics and services, fully interoperable with ROS 2 nodes
- `asyncio` execution model. No callbacks, no spinners, just Python
- Define ROS messages in Python with [`ros2_pyterfaces`](https://github.com/2lian/ros2_pyterfaces)
- Resource lifecycle via sessions and [scopes](https://github.com/2lian/asyncio-for-robotics)

> [!NOTE]
> Experimental. Actions and zero-copy are planned.

---

## ROS 2 interop

PyZeROS talks to ROS 2 through [Zenoh](https://zenoh.io/). The ROS 2 side **must** use [`rmw_zenoh_cpp`](https://github.com/ros2/rmw_zenoh):

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

Both sides must share the same `ROS_DOMAIN_ID` (defaults to `0`) and be on the same Zenoh network. In practice this means running a [Zenoh router](https://zenoh.io/docs/getting-started/quick-test/) and configuring both sides as clients to it, or using a peer-to-peer Zenoh config for DDS-like multicast discovery.

See the [rmw_zenoh docs](https://github.com/ros2/rmw_zenoh) for router setup and configuration.

## Install

PyZeROS is a normal Python package. Add it as a dependency to your project with your tool of choice. No colcon, no workspace, no overlay.

```bash
pip install git+https://github.com/2lian/pyzeros2
```

> [!NOTE]
> `pip install pyzeros` is not yet available. For now, we prioritise install from source using pixi.

### From source

```bash
git clone https://github.com/2lian/pyzeros2
cd pyzeros2
pixi install
pixi run router   # start a local Zenoh router
pixi run example  # run the minimal subscriber
```

---

## Tutorial

This mirrors the [official ROS 2 tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html), but without C++, build systems, or boilerplate.

### 1. Publisher and subscriber

The ROS 2 tutorial for this is [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html). Below is the PyZeROS equivalent.

**Publisher:**

```python
import asyncio
import pyzeros
import asyncio_for_robotics as afor
from ros2_pyterfaces.cyclone.all_msgs import String

@afor.scoped
async def main():
    pub = pyzeros.Pub(String, "chatter")
    counter = 0
    async for _ in afor.Rate(2).listen():
        pub.publish(String(data=f"Hello World: {counter}"))
        print(f"Publishing: Hello World: {counter}")
        counter += 1

with pyzeros.auto_context(node="talker", namespace="/demo"):
    asyncio.run(main())
```

**Subscriber:**

```python
import asyncio
import pyzeros
import asyncio_for_robotics as afor
from ros2_pyterfaces.cyclone.all_msgs import String

@afor.scoped
async def main():
    sub = pyzeros.Sub(String, "chatter")
    async for msg in sub.listen_reliable():
        print(f"I heard: {msg.data}")

with pyzeros.auto_context(node="listener", namespace="/demo"):
    asyncio.run(main())
```

That's it. No `rclpy.init()`, no `spin()`, no executor. The `async for` loop **is** the executor.

`auto_context` creates a session (Zenoh transport + ROS node identity) and makes it the default for everything inside. `@afor.scoped` ensures all resources created inside are cleaned up when the function returns. See [`asyncio-for-robotics`](https://github.com/2lian/asyncio-for-robotics) for details on scopes.

### 2. Service and client

The ROS 2 tutorial for this is [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html). Below is the PyZeROS equivalent.

**Server:**

```python
import asyncio
import pyzeros
import asyncio_for_robotics as afor
from ros2_pyterfaces.cyclone.all_srvs import AddTwoInts

@afor.scoped
async def main():
    server = pyzeros.Server(AddTwoInts, "add_two_ints")
    print("Service ready.")
    async for responder in server.listen_reliable():
        result = responder.request.a + responder.request.b
        responder.response.sum = result
        responder.send()
        print(f"{responder.request.a} + {responder.request.b} = {result}")

with pyzeros.auto_context(node="add_server", namespace="/demo"):
    asyncio.run(main())
```

**Client:**

```python
import asyncio
import pyzeros
import asyncio_for_robotics as afor
from ros2_pyterfaces.cyclone.all_srvs import AddTwoInts

@afor.scoped
async def main():
    client = pyzeros.Client(AddTwoInts, "add_two_ints")
    await client.wait_for_service()
    response = await client.call_async(AddTwoInts.Request(a=2, b=3))
    print(f"Result: {response.sum}")

with pyzeros.auto_context(node="add_client", namespace="/demo"):
    asyncio.run(main())
```

Services follow the same `async for` pattern as topics. The server yields `Responder` objects: read `responder.request`, fill `responder.response`, call `responder.send()`.

### 3. Custom messages

The ROS 2 tutorial for this is [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html). In ROS 2 this involves `.msg` files, CMake, and `colcon build`. In PyZeROS, it's a dataclass.

[`ros2_pyterfaces`](https://github.com/2lian/ros2_pyterfaces) provides two backends for message definitions:

| Backend | Import | Speed | Compatibility |
|---------|--------|-------|---------------|
| **cyclone** | `ros2_pyterfaces.cyclone` | Good | Full ROS 2 interop |
| **cydr** | `ros2_pyterfaces.cydr` | Faster | Slightly less compatible with edge cases |

Both backends ship pre-built standard messages (`all_msgs`, `all_srvs`) and let you define your own.

> [!IMPORTANT]
> For ROS 2 interop, the `typename` and field names **must** match the ROS message definition exactly.

**Defining a message:**

```python
from dataclasses import dataclass, field
from ros2_pyterfaces.cyclone import idl, all_msgs

@dataclass
class MyStatus(idl.IdlStruct, typename="my_package/msg/MyStatus"):
    header: all_msgs.Header = field(default_factory=all_msgs.Header)
    temperature: idl.types.float64 = 0.0
    labels: idl.types.sequence[str] = field(default_factory=list)
    active: bool = False
```

Use it like any other message:

```python
pub = pyzeros.Pub(MyStatus, "status")
pub.publish(MyStatus(temperature=36.5, labels=["sensor_a"], active=True))
```

**Defining a service:**

```python
from dataclasses import dataclass
from ros2_pyterfaces.cyclone.idl import IdlStruct

@dataclass
class Request(IdlStruct, typename="my_package/srv/Calibrate_Request"):
    target: str = ""

@dataclass
class Response(IdlStruct, typename="my_package/srv/Calibrate_Response"):
    success: bool = False

class Calibrate:
    Request = Request
    Response = Response

    @staticmethod
    def get_type_name() -> str:
        return "my_package/srv/Calibrate"

    @staticmethod
    def hash_rihs01() -> str:
        return Request.hash_rihs01()  # derived from the wire format
```

See [`ros2_pyterfaces`](https://github.com/2lian/ros2_pyterfaces) for the full type system.

---

## Putting it together

A more realistic node combining topics and services with proper resource management:

```python
import asyncio
from contextlib import suppress

import asyncio_for_robotics as afor
import pyzeros
from ros2_pyterfaces.cyclone.all_msgs import String
from ros2_pyterfaces.cyclone.all_srvs import Trigger

@afor.scoped
async def main():
    tg = afor.Scope.current().task_group
    tg.create_task(publisher())
    tg.create_task(listener())
    tg.create_task(serve_trigger())
    await asyncio.Future()

@afor.scoped
async def publisher():
    pub = pyzeros.Pub(String, "heartbeat")
    counter = 0
    async for _ in afor.Rate(1).listen():
        pub.publish(String(data=f"alive #{counter}"))
        counter += 1

async def listener():
    sub = pyzeros.Sub(String, "commands")
    async for msg in sub.listen_reliable():
        print(f"Command: {msg.data}")

async def serve_trigger():
    server = pyzeros.Server(Trigger, "reset")
    async for responder in server.listen_reliable():
        print("Reset triggered!")
        responder.response.success = True
        responder.response.message = "done"
        responder.send()

if __name__ == "__main__":
    with pyzeros.auto_context(node="my_robot", namespace="/robot"):
        with suppress(KeyboardInterrupt):
            asyncio.run(main())
```

What this gives you over standard ROS 2 Python:

- **Scoped cleanup**: `@afor.scoped` closes all publishers, subscribers, servers, and rates when the function exits. No dangling resources, no manual `destroy_*` calls.
- **TaskGroup structure**: Tasks run concurrently inside the scope's `TaskGroup`. If one crashes, the others are cancelled and the error propagates cleanly. In standard ROS 2, a crashed callback silently dies.
- **Session resolution**: `auto_context` binds a session (node identity + transport) for the block. Every `Pub`, `Sub`, `Client`, `Server` created inside auto-resolves to that session. No passing `self.node` around.
- **Thread safety by default**: Everything runs on one asyncio event loop. No GIL juggling, no executor threads, no callback reentrancy bugs.

For the scope and session system in detail, see [`asyncio-for-robotics`](https://github.com/2lian/asyncio-for-robotics).

---

## Examples

> [!NOTE]
> Those examples are meant to run out-of-the-box with an installation from
> source using pixi. With just a pip install you'll need to setup your
> environment and ROS yourself.

Examples live under `pyzeros.examples.*`:

| Example | Run | Description |
|---------|-----|-------------|
| [example.py](./pyzeros/examples/example.py) | `pixi run example` | Minimal subscriber |
| [basic_usage.py](./pyzeros/examples/basic_usage.py) | `pixi run python -m pyzeros.examples.basic_usage` | Repeater with custom message |
| [demo.py](./pyzeros/examples/demo.py) | `pixi run demo` | Ring of async tasks |
| [custom_msgs.py](./pyzeros/examples/custom_msgs.py) | `pixi run python -m pyzeros.examples.custom_msgs` | Python-defined JointState |

Inspect from the ROS 2 side:

```bash
pixi run -e ros ros2 topic list
pixi run -e ros ros2 topic echo /demo/chatter std_msgs/msg/String
```

## License

MIT
