# Pyzeros

A simple toy example of a ROS 2 node written only with python and zenoh. No ROS 2 dependencies.

Only made to work with ROS 2 Jazzy + RMW_zenoh for now. This is very very much for fun and a proof of concept, made in less than a day. Many things in the code are not ideal.

## Run the node

Pixi is required and highly recommanded to run the commands. Evironment is setup in the pixi.toml.

Start the node:

```python
# terminal 1
pixi run main
```

Start the zenoh router:

```python
# terminal 2
pixi run router
```

## Explore

Simply use normal ros2 cli to interact with the node, pub and sub.
