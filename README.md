# PyZeROS2

Python demo for interfacing [`asyncio-for-robotics`](https://github.com/2lian/asyncio-for-robotics) with [`ros-z`](https://github.com/ZettaScaleLabs/ros-z). This exposes ROS 2 capabilities in python without installing ROS 2 nor using `rclpy`.

> [!WARNING]
> This project is an early proof of concept, not a polished or stable package.
> Using it in your own project will likely require manual setup and experimentation.
> The project name (`PyZeROS2`) is also tentative and may change.

## Installation from source

[Pixi is required](https://pixi.prefix.dev/latest/installation/) and will handle our environments and installation.

`ros-z` is not published yet, so this project builds it from source.

```bash
git clone https://github.com/2lian/pyzeros2 -b ros-z
cd pyzeros2

# clone the patched ros-z repo inside this repository
git clone https://github.com/ZettaScaleLabs/ros-z ros-z

# install all environments and builds
pixi install -a
```

---

- [Simple listener pyzeros.example](#simple-listener-pyzerosexample)
- [Ring demo pyzeros.demo](#ring-demo-pyzerosdemo)
  - [What the demo does](#what-the-demo-does)
- [Custom Messages Hack pyzeros.custom_msgs](#custom-messages-hack-pyzeroscustom_msgs)
- [ROS 2 Environment Setup for Interoperability](#ros-2-environment-setup-for-interoperability)

## Simple listener [pyzeros.example](./pyzeros/example.py)

Start the router:

```bash
pixi run router
```

In another terminal, start the Python node:

```bash
pixi run example
```

It is exposed as a normal ROS node, so you can publish from the ROS 2 CLI:

```bash
pixi run -e ros ros2 topic pub "/chatter" std_msgs/msg/String "{data: "Hello_World"}"
```

## Ring demo [pyzeros.demo](./pyzeros/demo.py)

Start the router:

```bash
pixi run router
```

In another terminal, run the demo:

```bash
pixi run demo
# To see available parameters: `pixi run demo -h`
```

Use standard ROS 2 cli:

```bash
pixi run -e ros ros2 topic list
```

### What the demo does

The demo creates a ring of participants:

```
participant_0 -> participant_1 -> ... -> participant_N-1 -> participant_0
```

Each participant:
- subscribes to one topic,
- receives a string,
- appends one character to it,
- publishes to the next participant.

This is a small concurrency demo showing:
- many async tasks running together,
- one subscriber and one publisher per participant,
- non-blocking delays between receive and send,
- everything running in a single thread.

You can change the number of participants, delay, initial payload, and target string.

## Custom Messages Hack [pyzeros.custom_msgs](./pyzeros/custom_msgs.py)

Start the router:

```bash
pixi run router
```

In another terminal, run the pub/sub:

```bash
pixi run python -m pyzeros.custom_msgs
```

Use standard ROS 2 cli to listen to a `KeyValue` message that is not part of native `ros-z` and that we [re-created in python](./pyzeros/custom_msgs.py).

```bash
pixi run -e ros ros2 topic echo /custom_key_val
```

## ROS 2 Environment Setup for Interoperability

Use `pixi shell` to enter the PyZeROS2 environment. Use `pixi shell -e ros` to enter the ROS 2 Jazzy environment.

Refer to the [ros-z documentation](https://zettascalelabs.github.io/ros-z/chapters/interop.html) for complete explanation.

The `pixi.toml` provides two isolated environments
- `default`: `PyZeROS2` python project and `ros-z` installed from source.
- `ros`: Only ROS 2 Jazzy CLI with `rmw-zenoh-cpp`

As well as environment variable to ensure communications between Zenoh and ROS 2 (purposefully limited to localhost):
```toml
RMW_IMPLEMENTATION="rmw_zenoh_cpp"
ROS_DOMAIN_ID="0"
ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST"
ZENOH_ROUTER_CONFIG_URI="./router.json5"
ZENOH_SESSION_CONFIG_URI="./client.json5"
ROS_LOCALHOST_ONLY=""
```
