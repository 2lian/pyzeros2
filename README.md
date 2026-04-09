# PyZeROS2

`pyzeros` is a Python only interface to ROS 2, built on top of [Zenoh](https://zenoh.io/), [`asyncio-for-robotics`](https://github.com/2lian/asyncio-for-robotics), and [`ros2_pyterfaces`](https://github.com/2lian/ros2_pyterfaces). The final goal is to be installable with `pip`.

_PyZeROS2_ lets you write ROS 2-compatible Python code: without `rclpy`, without a ROS 2 installation, without even compiling messages. Replacing the callback-based ROS executor, _PyZeROS2_ execution model is simply `asyncio`, ensuring thread safety and first class integration with the Python ecosystem.

Features:

- Write Asyncio Python code to use topics and services.
- Use and **CREATE!** ROS 2 messages in python.
- No ROS 2 dependencies.
- Interoperability with ROS 2 using `RMW_IMPLEMENTATION="rmw_zenoh_cpp"`

> [!NOTE]
> This project is still experimental and may change a lot.
>
> Planned work:
>
> - Actions
> - Shared memory / zero-copy improvements

## Installation from source

[Pixi is required](https://pixi.prefix.dev/latest/installation/) and manages the development environments for this repo.

```bash
git clone https://github.com/2lian/pyzeros2
cd pyzeros2

pixi install
```

> [!NOTE]
> This is currently a development setup. A regular `pip install` flow will come later.

## Quick Start

Start the local Zenoh router:

```bash
pixi run router
```

In another terminal, run the minimal Python subscriber:

```bash
pixi run example
```

From the ROS 2 side, publish with the normal CLI:

```bash
pixi run -e ros ros2 topic pub "/pyzeros/chatter" std_msgs/msg/String '{data: "Hello_World"}'
```

## Basic Usage

```python
import asyncio
from dataclasses import dataclass

import asyncio_for_robotics as afor
from ros2_pyterfaces.cyclone.idl import IdlStruct

from pyzeros.node import Node


@dataclass
class MyCustomString(IdlStruct, typename="std_msgs/msg/String"):
    data: str = ""


async def repeat_task(node: Node):
    sub = node.create_subscriber(MyCustomString, "listener")
    pub = node.create_publisher(MyCustomString, "repeater")
    async for msg in sub.listen_reliable():
        pub.publish(MyCustomString(data=f"repeating: {msg.data}"))


async def pub_task(node: Node):
    pub = node.create_publisher(MyCustomString, "listener")
    counter = 0
    async for _ in afor.Rate(1).listen():
        pub.publish(MyCustomString(f"Hello World! #{counter}"))
        counter += 1



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
    async with asyncio.TaskGroup() as tg:
        node = Node(name="my_node", namespace="/pyzeros")
        tg.create_task(pub_task(node))
        tg.create_task(repeat_task(node))
        print('pixi run -e ros ros2 topic echo "/pyzeros/repeater" std_msgs/msg/String')


if __name__ == "__main__":
    asyncio.run(main())
```

- For the async subscription model, see [`asyncio-for-robotics`](https://github.com/2lian/asyncio-for-robotics).
- For Python-defined ROS message classes, see [`ros2_pyterfaces`](https://github.com/2lian/ros2_pyterfaces).
- For `pyzeros` itself, read the source docstrings and the examples in [`pyzeros/examples`](./pyzeros/examples).

## Examples

Example modules live under `pyzeros.examples.*`.

### Minimal subscriber [pyzeros.examples.example](./pyzeros/examples/example.py)

```bash
pixi run example
```

Creates one node and listens on `/pyzeros/chatter` using the current `Node` / `Sub` API.

### Ring demo [pyzeros.examples.demo](./pyzeros/examples/demo.py)

```bash
pixi run demo
```

Starts a ring of asyncio tasks. Each participant subscribes to one topic, appends one character, and republishes to the next participant.

You can inspect the resulting topics from the ROS 2 side:

```bash
pixi run -e ros ros2 topic list
```

### Python-defined message example [pyzeros.examples.custom_msgs](./pyzeros/examples/custom_msgs.py)

This example defines a ROS-compatible message class in Python and uses it with the same `Node` / `Pub` / `Sub` workflow as built-in message types.

```bash
pixi run python -m pyzeros.examples.custom_msgs
```

Then inspect it from the ROS 2 side:

```bash
pixi run -e ros ros2 topic echo /pyzeros/custom_msg
```

## ROS 2 network interoperability

### For yourself

This project is only compatible with ROS 2 using `RMW_IMPLEMENTATION="rmw_zenoh_cpp"`, see the docs for the RMW here: https://github.com/ros2/rmw_zenoh .

### In this repo

This repo gives an example of Pyzeros/ROS2 interoperability. Use `pixi shell` for the Python environment and `pixi shell -e ros` for the ROS 2 Jazzy environment.

- `default`: `pyzeros` and its Python dependencies
- `ros`: ROS 2 Jazzy CLI with `rmw_zenoh_cpp`

These environment variables keep communication local and make ROS 2 and `pyzeros` talk through the same Zenoh network:

```toml
RMW_IMPLEMENTATION="rmw_zenoh_cpp"
ROS_DOMAIN_ID="0"
ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST"
ZENOH_ROUTER_CONFIG_URI="./router.json5"
ZENOH_SESSION_CONFIG_URI="./client.json5"
ROS_LOCALHOST_ONLY=""
```
