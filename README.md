# PyZeROS2

`pyzeros` is a small Python layer on top of [`ros-z`](https://github.com/ZettaScaleLabs/ros-z) and [`asyncio-for-robotics`](https://github.com/2lian/asyncio-for-robotics).

We let you write ROS 2-compatible Python code without `rclpy`, and even without needing a ROS 2 installation. Our execution model -- replacing the ROS executor -- is simply `asyncio`, ensuring thread safety and first class integration with the Python ecosystem.

Features:
- Write Asyncio Python code to subscribe to topics: [`asyncio-for-robotics`](https://github.com/2lian/asyncio-for-robotics)
- Use and **CREATE!** ROS 2 messages: [`ros2_pyterfaces`](https://github.com/2lian/ros2-pyterfaces)
- Access all other ROS 2 features: [`ros-z`](https://github.com/ZettaScaleLabs/ros-z)
- No ROS 2 depenencies.
- Zero-copy.

> [!Note]
> This project is still experimental and may change a lot.

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

Start the local Zenoh router:

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
from ros2_pyterfaces import all_msgs
from pyzeros.pub import ZPublisher
from pyzeros.sub import Sub

async def main():
    pub = ZPublisher(all_msgs.String, "/repeater")
    sub = Sub(all_msgs.String, "/listener")
    async for msg in sub.listen()
        pub(all_msgs.String(data=f"repeating: {msg.data}"))

asyncio.run(main())
```

- To learn about the `asyncio` usage and subscriber refer to: [`asyncio-for-robotics`](https://github.com/2lian/asyncio-for-robotics)
- To use and create ros2 messages refer to: [`ros2_pyterfaces`](https://github.com/2lian/ros2-pyterfaces).
- To use the wider ROS 2 echosystem refer to: [`ros-z`](https://github.com/ZettaScaleLabs/ros-z)
- For usage of this library, refer to the docstring of the source code and the examples.

## Examples

### Simple listener [pyzeros.example](./pyzeros/example.py)

Minimal example showing a Python node receiving messages from the ROS 2 CLI.

### Ring demo [pyzeros.demo](./pyzeros/demo.py)

Run:

```bash
pixi run demo
```

This starts a ring of asyncio tasks. Each participant subscribes to one topic, appends one character, and republishes to the next participant. It is mostly a concurrency demo showing how to run many small subscribers and publishers.

You can inspect the resulting topics from the ROS 2 side:

```bash
pixi run -e ros ros2 topic list
```

You can additionally have fun, and run `pixi run demo -n 1000 -s 0`.

### Python-defined message example [pyzeros.custom_msgs](./pyzeros/custom_msgs.py)

You can re-create ROS types using the CycloneDDs based `ros2_pyterfaces` library. If your python class definition corresponds exactly to the type of ROS 2, then communication will work! If not, messages will not be delivered.

You need:
- Same `typename` as ROS (here `sensor_msgs/msg/JointState`).
- Same attribute names (here `header`, `name` ...)
- Same attribute type annotations (here `all_msgs.Header`, `idl.types.sequence[str]` ...)

```python
from dataclasses import dataclass, field
from ros2_pyterfaces import all_msgs, idl

@dataclass
class MyCustomType(idl.IdlStruct, typename="sensor_msgs/msg/JointState"):
    header: all_msgs.Header = field(default_factory=all_msgs.Header)
    name: idl.types.sequence[str] = field(default_factory=list)
    position: idl.types.sequence[idl.types.float64] = field(default_factory=list)
    velocity: idl.types.sequence[idl.types.float64] = field(default_factory=list)
    effort: idl.types.sequence[idl.types.float64] = field(default_factory=list)
```

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

- `default`: `pyzeros`, it python dependencies, plus `ros-z` built from source,
- `ros`: ROS 2 Jazzy CLI with `rmw_zenoh_cpp`.

The following environment variables keep communication local and make the ROS 2 side talk through the same Zenoh network:

```toml
RMW_IMPLEMENTATION="rmw_zenoh_cpp"
ROS_DOMAIN_ID="0"
ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST"
ZENOH_ROUTER_CONFIG_URI="./router.json5"
ZENOH_SESSION_CONFIG_URI="./client.json5"
ROS_LOCALHOST_ONLY=""
```

For more background on the ROS 2 side of the setup, see the `ros-z` interoperability documentation:
https://zettascalelabs.github.io/ros-z/chapters/interop.html
