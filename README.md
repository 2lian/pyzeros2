# PyZeROS2

Python prototype for interfacing [`asyncio-for-robotics`](https://github.com/2lian/asyncio-for-robotics) with [`ros-z`](https://github.com/ZettaScaleLabs/ros-z). This exposes ROS 2 capabilities in python without installing ROS 2 nor using `rclpy`.

This is an early proof of concept, not a polished package.

## Installation

[Pixi is required](https://pixi.prefix.dev/latest/) and will handle our environments and installation.

`ros-z` is not published yet, so this project builds it from source.

```bash
git clone https://github.com/2lian/pyzeros2
cd pyzeros2

# clone the patched ros-z repo inside this repository
git clone https://github.com/2lian/ros-z ros-z

# install all environments and builds
pixi install -a
```

## [Simple listened](./pyzeros/example.py)

Start the router:

```bash
pixi run router
```

In another terminal, start the Python node:

```bash
pixi run main
```

It is exposed as a normal ROS node, so you can publish from the ROS 2 CLI:

```bash
pixi run -e ros ros2 topic pub "/chatter" std_msgs/msg/String "{data: "Hello_World"}"
```

## [Ring demo](./pyzeros/demo.py)

Start the router:

```bash
pixi run router
```

In another terminal, run the demo:

```bash
pixi run demo
# To see available parameters: `pixi run demo -h`
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
