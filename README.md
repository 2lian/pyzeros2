# PyZeROS

| Requirements | Interoperability | Test Matrix |
|---|:---:|:---:|
| [![python](https://img.shields.io/pypi/pyversions/pyzeros?logo=python&logoColor=white&label=Python&color=%20blue)](https://pypi.org/project/pyzeros/) <br> [![RMW Zenoh](https://img.shields.io/badge/ROS_RMW-Zenoh-%20blue)](https://github.com/ros2/rmw_zenoh) <br> [![license](https://img.shields.io/badge/License-MIT-gold)](https://opensource.org/license/mit) | [![ros](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros)](https://github.com/ros2) <br> [![ros](https://img.shields.io/badge/ROS_2-Lyrical-blue?logo=ros)](https://github.com/ros2) <br> [![Interop Tests](https://github.com/2lian/pyzeros2/actions/workflows/ros-interop.yml/badge.svg)](https://github.com/2lian/pyzeros2/actions/workflows/ros-interop.yml) | [![linux](https://img.shields.io/badge/OS-Linux-black?logo=linux&logoColor=white)](https://github.com/2lian/pyzeros2/actions/workflows/python-tests.yml) <br> [![Windows](https://custom-icon-badges.demolab.com/badge/OS-Windows-black?logo=windows11&logoColor=white)](https://github.com/2lian/pyzeros2/actions/workflows/python-tests.yml) <br> [![macOS_ARM](https://img.shields.io/badge/OS-macOS_ARM-000000?logo=apple&logoColor=white)](https://github.com/2lian/pyzeros2/actions/workflows/python-tests.yml) <br> [![Tests](https://github.com/2lian/pyzeros2/actions/workflows/python-tests.yml/badge.svg)](https://github.com/2lian/pyzeros2/actions/workflows/python-tests.yml) |

An alternative to ROS 2 `rclpy`. Minimal dependencies, no ROS installation, and an asyncio execution model. Just `pip install` and talk to your favorite ROS network.

Built on [Zenoh](https://zenoh.io/), [`asyncio-for-robotics`](https://github.com/2lian/asyncio-for-robotics), and [`ros2-pyterfaces`](https://github.com/2lian/ros2-pyterfaces).

```python
import asyncio, pyzeros
import asyncio_for_robotics as afor
from ros2_pyterfaces.cyclone.all_msgs import String

@afor.scoped
async def main():
    sub = pyzeros.Sub(String, "chatter")
    async for msg in sub.listen_reliable():
        print(msg.data)

with pyzeros.auto_context(node="listener", namespace="/demo") as node:
    asyncio.run(main())
```

Features:

- Topics and services, fully interoperable with ROS 2 nodes.
- `asyncio` execution model. No callbacks, no spinners, just Python.
- Define ROS messages in Python with [`ros2_pyterfaces`](https://github.com/2lian/ros2_pyterfaces).
- Resource lifecycle via [sessions and scopes](https://github.com/2lian/asyncio-for-robotics).

> [!IMPORTANT]
> - Experimental, API is still subject to changes. Please lock this dependency.
> - QoS Transient, Actions and zero-copy are not implemented yet.

## ROS 2 interop

PyZeROS talks to ROS 2 through [Zenoh](https://zenoh.io/). This means:
- The ROS 2 side **must** use [`rmw_zenoh_cpp`](https://github.com/ros2/rmw_zenoh)
- Both sides must share the same `ROS_DOMAIN_ID` (PyZeROS reads your environment variable like ROS for its default).
- You must have a [Zenoh router](https://zenoh.io/docs/getting-started/installation/) running (alternatively, you can setup Zenoh as peer-to-peer).

> [!NOTE]
> In short for Ubuntu: set `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` in your `~/.bashrc` and run `zenohd` (or `ros2 run rmw_zenoh_cpp rmw_zenohd`) in the background.

See the [rmw_zenoh docs](https://github.com/ros2/rmw_zenoh) for router/client/peer setup and configuration.

## Install

PyZeROS is a normal Python package. Add it as a dependency to your project with your tool of choice (`pip`, `uv`, `pixi`, ...). No colcon, no workspace, no compilation.

```bash
pip install pyzeros
```

### From source

```bash
git clone https://github.com/2lian/pyzeros2
cd pyzeros2
pixi install
pixi run router   # start a local Zenoh router
pixi run example  # run the minimal subscriber
```

## Tutorial

This mirrors the [official ROS 2 tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html).

### 1. Publisher and subscriber

The ROS 2 tutorial for this is [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html). Below is the PyZeROS equivalent.

#### Publisher:

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

#### Subscriber:

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

`auto_context` creates a node (e.g., a PyZeROS session) and sets it as default,
while `@afor.scoped` cleans up its async resources. See the documentation of the executor --[`asyncio-for-robotics`](https://github.com/2lian/asyncio-for-robotics) -- for details on scopes and sessions.

### 2. Service and client

The ROS 2 tutorial for this is [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html). Below is the PyZeROS equivalent.

#### Server:

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

#### Client:

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

The ROS 2 tutorial for this is [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html). In ROS 2 this involves `.msg` files, CMake, and `colcon build`. In PyZeROS, it's just a Python class.

[`ros2_pyterfaces`](https://github.com/2lian/ros2_pyterfaces) provides two backends for message definitions:

| Backend     | Import                    | Speed  | Compatibility                            |
| ----------- | ------------------------- | ------ | ---------------------------------------- |
| **cyclone** | `ros2_pyterfaces.cyclone` | Fair   | Full ROS 2 interop                       |
| **cydr**    | `ros2_pyterfaces.cydr`    | Very fast | Not all messages are supported; uses just-in-time compilation. |

Both backends ship pre-built standard messages (`all_msgs`, `all_srvs`) and let you define your own. You can find message definitions for multiple ROS distros if you need to juggle between them: `Humble`, `Jazzy`, `Kilted`, `Lyrical`.

#### Defining a message:

```python
import pyzeros
from dataclasses import dataclass, field
from ros2_pyterfaces.cyclone import idl, all_msgs
from ros2_pyterfaces.cydr import idl as cydr_idl, all_msgs as cydr_all_msgs

@dataclass
class Num(idl.IdlStruct, typename="tutorial_interfaces/msg/Num"):
    num: idl.types.int64 = 0

@dataclass
class NumCydr(cydr_idl.IdlStruct, typename="tutorial_interfaces/msg/Num"):
    num: cydr_idl.types.int64 = 0

@dataclass
class Sphere(idl.IdlStruct, typename="tutorial_interfaces/msg/Sphere"):
    center: all_msgs.Point = field(default_factory=all_msgs.Point)
    radius: idl.types.float64 = 0.0

@dataclass
class SphereCydr(cydr_idl.IdlStruct, typename="tutorial_interfaces/msg/Sphere"):
    center: cydr_all_msgs.Point = field(default_factory=cydr_all_msgs.Point)
    radius: cydr_idl.types.float64 = 0.0

pub = pyzeros.Pub(Sphere, "sphere")
pub.publish(Sphere(radius=42.0))
```

> [!IMPORTANT]
> For ROS 2 interop, the `typename` and field names **must** match the ROS message definition exactly.

#### Defining a service:

```python
from dataclasses import dataclass
from ros2_pyterfaces.cyclone import idl
from ros2_pyterfaces.cyclone.idl import IdlStruct, make_idl_service

@dataclass
class AddThreeIntsRequest(IdlStruct, typename="tutorial_interfaces/srv/AddThreeInts_Request"):
    a: idl.types.int64 = 0
    b: idl.types.int64 = 0
    c: idl.types.int64 = 0

@dataclass
class AddThreeIntsResponse(IdlStruct, typename="tutorial_interfaces/srv/AddThreeInts_Response"):
    sum: idl.types.int64 = 0

AddThreeInts = make_idl_service(AddThreeIntsRequest, AddThreeIntsResponse)
```

See [`ros2_pyterfaces`](https://github.com/2lian/ros2_pyterfaces) for the full type system.

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
    # Gets the task group of this afor scope
    tg = afor.Scope.current().task_group
    # adds concurent tasks to the group
    tg.create_task(publisher())
    tg.create_task(listener())
    tg.create_task(serve_trigger())
    # block indefinitely to not exit the scope
    await asyncio.Future()

@afor.scoped
async def publisher():
    # publisher declared, will be destroyed on scope exit, so at the end of this coroutine
    pub = pyzeros.Pub(String, "heartbeat")
    counter = 0
    # Timer executing at 1 Hz
    async for _ in afor.Rate(1).listen():
        pub.publish(String(data=f"alive #{counter}"))
        counter += 1

async def listener():
    # Subscriber declared, will be destroyed on scope exit, so when main() finishes
    sub = pyzeros.Sub(String, "commands")
    # Iterates every time a message arrives
    async for msg in sub.listen_reliable():
        print(f"Command: {msg.data}")

async def serve_trigger():
    # Service server declared, will be destroyed on scope exit, so when main() finishes
    server = pyzeros.Server(Trigger, "reset")
    # Iterates every time a request arrives
    async for responder in server.listen_reliable():
        print("Reset triggered!")
        # responder object hold the request, and the response to fill out
        responder.response.success = True
        responder.response.message = "done"
        # Sends the reply
        responder.send()

if __name__ == "__main__":
    # `my_robot` node is created and set as default
    with pyzeros.auto_context(node="my_robot", namespace="/robot"):
        with suppress(KeyboardInterrupt):
            # event loop starts
            asyncio.run(main())
```

## Examples

> [!NOTE]
> Those examples can run directly from your installation without our pixi environment. However running them from inside this repo using `pixi run <command>` will ensure you are using the right RMW, ROS_DOMAIN_ID, zenoh config ...

Examples live under `pyzeros.examples.*`:

| Example                                             | Run                                               | Description                  |
| --------------------------------------------------- | ------------------------------------------------- | ---------------------------- |
| [example.py](./pyzeros/examples/example.py)         | `pixi run python -m pyzeros.examples.example`                                | Minimal subscriber           |
| [basic_usage.py](./pyzeros/examples/basic_usage.py) | `pixi run python -m pyzeros.examples.basic_usage` | Repeater with custom message |
| [demo.py](./pyzeros/examples/demo.py)               | `pixi run python -m pyzeros.examples.demo`                                   | Ring of async tasks          |
| [custom_msgs.py](./pyzeros/examples/custom_msgs.py) | `pixi run python -m pyzeros.examples.custom_msgs` | Python-defined JointState    |

Inspect from the ROS 2 side (using our pixi):

```bash
pixi run -e jazzy ros2 topic list
pixi run -e jazzy ros2 topic echo /demo/chatter std_msgs/msg/String
```
