# PyZeROS2

`pyzeros` is a Python only interface to ROS 2, built on top of [`ros-z`](https://github.com/ZettaScaleLabs/ros-z), [`asyncio-for-robotics`](https://github.com/2lian/asyncio-for-robotics), and [`ros2_pyterfaces`](https://github.com/2lian/ros2_pyterfaces). The final goal is to be installable with `pip`.

*PyZeROS2* lets you write ROS 2-compatible Python code: without `rclpy`, without a ROS 2 installation, without even compiling messages. Replacing the callback-based ROS executor, *PyZeROS2* execution model is simply `asyncio`, ensuring thread safety and first class integration with the Python ecosystem.

Features:
- Write Asyncio Python code to sub/sub to topics: [`asyncio-for-robotics`](https://github.com/2lian/asyncio-for-robotics)
- Use and **CREATE!** ROS 2 messages in python (`.msg`, `.srv`): [`ros2_pyterfaces`](https://github.com/2lian/ros2-pyterfaces)
- Access all other ROS 2 features: [`ros-z`](https://github.com/ZettaScaleLabs/ros-z)
- No ROS 2 dependencies.

> [!Note]
> This project is still experimental and may change a lot.
> 
> Upcomming features:
> - Services Client/Servers (soon)
> - Actions (later)
> - Shared Memory + Zero Copy (looking for help)
> - Many ROS 2 introspection tools (topic list, node list...) are available in `ros-z-py`, missing ones needs to be implemented there and not here.

## Installation from source

[Pixi is required](https://pixi.prefix.dev/latest/installation/) and manages the dev environments for this repo. `ros-z` will be downloaded and build from source.

```bash
git clone https://github.com/2lian/pyzeros2
cd pyzeros2

pixi install
```

> [!Note]
> This is a developpement environment only working with `pixi`. Installation with `pip install` will be available once the project is stable enough.


## Quick Start

Start the local Zenoh router (always required to be running):

```bash
pixi run router
```

In another terminal, run the simple Python listener:

```bash
pixi run example
```

From the ROS 2 side, publish with the normal CLI:

```bash
pixi run -e ros ros2 topic pub "/chatter" std_msgs/msg/String "{data: "Hello_World"}"
```

Our PyZeROS2 Python process receives the message from a standard ROS 2 publisher. You can also see the node and topic using the standard ROS 2 cli.

## Basic Usage

```python
import asyncio
from dataclasses import dataclass

import asyncio_for_robotics as afor
from ros2_pyterfaces.cyclone.all_msgs import String
from ros2_pyterfaces.cyclone.idl import IdlStruct

from pyzeros.pub import ZPublisher
from pyzeros.sub import Sub


@dataclass
class MyCustomString(IdlStruct, typename="std_msgs/msg/String"):
    """ros2_pyterfaces allow us to re-create any ROS message.
    We could have also used 
        `from ros2_pyterfaces.cyclone.all_msgs import String`
    """
    data: str = ""


async def repeat_task():
    """Listen on a topic, repeats on another"""
    # PyZeROS2 will translate the ros2_pyterfaces message into ros-z format,
    # and declare publisher / subscriber .
    sub = Sub(MyCustomString, "/listener")
    pub = ZPublisher(MyCustomString, "/repeater")
    # asyncio_for_robotics allows us to use ayncio syntax.
    # here we iterate every incomming messages
    async for msg in sub.listen_reliable():
        pub.publish(MyCustomString(data=f"repeating: {msg.data}"))


async def pub_task():
    """Just a publisher publishing at a constant rate"""
    pub = ZPublisher(MyCustomString, "/listener")
    # asyncio_for_robotics constant rate, like a ros timer
    async for t_ns in afor.Rate(1).listen():
        pub.publish(MyCustomString("Hello World!"))


async def main():
    # asyncio can register several tasks that execute concurently
    async with asyncio.TaskGroup() as tg:
        tg.create_task(pub_task())
        tg.create_task(repeat_task())
        print("Listen to me using: ")
        print("""pixi run -e ros ros2 topic echo "/repeater" std_msgs/msg/String""")


asyncio.run(main())
```

- To learn about the `asyncio` usage and subscriber refer to: [`asyncio-for-robotics`](https://github.com/2lian/asyncio-for-robotics)
- To use and create ros2 messages refer to: [`ros2_pyterfaces`](https://github.com/2lian/ros2-pyterfaces).
- To use the wider ROS 2 echosystem refer to: [`ros-z`](https://github.com/ZettaScaleLabs/ros-z)
- For usage of this library, refer to the docstring of the source code and the examples.

## Examples

### Ring demo [pyzeros.demo](./pyzeros/demo.py)

```bash
pixi run demo
```

Starts a ring of asyncio pub/sub tasks. Each participant subscribes to one topic, appends one character, and republishes to the next participant. It is mostly a concurrency demo showing how to run many small subscribers and publishers.

You can inspect the resulting topics from the ROS 2 side:

```bash
pixi run -e ros ros2 topic list
```

You can additionally have fun, and run `pixi run demo -n 1000 -s 0`.

### Python-defined message example [pyzeros.custom_msgs](./pyzeros/custom_msgs.py)

You can re-create ROS types using the `ros2_pyterfaces` library. If your python class definition corresponds exactly to the type of ROS 2, then communication will work! If not, messages will not be delivered.

You need:
- Same `typename` as ROS (here `sensor_msgs/msg/JointState`).
- Same attribute names (here `header`, `name` ...)
- Same attribute type annotations (here `all_msgs.Header`, `idl.types.sequence[str]` ...)

Here we define a ROS 2 [sensor_msgs/msg/JointState](https://docs.ros2.org/foxy/api/sensor_msgs/msg/JointState.html) message type fully from python at runtime (no colcon compilation, no ros package).

```python
from dataclasses import dataclass, field
from ros2_pyterfaces.cyclone import all_msgs, idl

@dataclass
class MyCustomType(idl.IdlStruct, typename="sensor_msgs/msg/JointState"):
    header: all_msgs.Header = field(default_factory=all_msgs.Header)
    name: idl.types.sequence[str] = field(default_factory=list)
    position: idl.types.sequence[idl.types.float64] = field(default_factory=list)
    velocity: idl.types.sequence[idl.types.float64] = field(default_factory=list)
    effort: idl.types.sequence[idl.types.float64] = field(default_factory=list)
```

> [!NOTE]
> `ros2_pyterfaces` has 2 codecs implemented: CycloneDDS and CYDR. 
> - CycloneDDS is compatible with any message/service, purely written in python.
> - CYDR is much more performant leveraging JIT compilation, but not compatible with everything.

Run:

```bash
pixi run python -m pyzeros.custom_msgs
```

This example defines the message class in Python and still interoperates with ordinary ROS 2 tooling.

```bash
pixi run -e ros ros2 topic echo /custom_msg
```

## ROS 2 Network Interoperability

Use `pixi shell` for the Python environment and `pixi shell -e ros` for the ROS 2 Jazzy CLI environment.

The repo defines two isolated environments:

- `default`: `pyzeros`, its python dependencies, plus `ros-z` built from source with `cargo`+`maturin`
- `ros`: ROS 2 Jazzy CLI with `rmw_zenoh_cpp`.

The following environment variables keep communications local and make the ROS 2 talk through the same Zenoh network:

```toml
RMW_IMPLEMENTATION="rmw_zenoh_cpp"
ROS_DOMAIN_ID="0"
ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST"
ZENOH_ROUTER_CONFIG_URI="./router.json5"
ZENOH_SESSION_CONFIG_URI="./client.json5"
ROS_LOCALHOST_ONLY=""
```

For more background on the Zenoh configuration for ROS 2 refer to the RMW documentation:
https://github.com/ros2/rmw_zenoh

For more background on ROS 2 interoperability, see the `ros-z` documentation:
https://zettascalelabs.github.io/ros-z/chapters/interop.html
