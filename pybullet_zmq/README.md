# PyBullet ZMQ

PyBullet Simulator with a ZMQ communication interface.

## Usage instructions

First, make sure to clone [aica-technology/docker-images](https://github.com/aica-technology/docker-images) and install
the scripts before you start, as the instructions below rely on those scripts.

Second, build the image with `bash build.sh`.

Finally, you have two options to run the container, either as a [runtime container](#run-container-in-interactive-mode)
or as an [SSH server](#ssh-server-for-remote-development) for remote development.

### Run container in interactive mode

Spin up the container with

```console
aica-docker interactive aica-technology/zmq-simulator -u ros2
```

If you want to connect to the image to have a more terminals, run

```console
aica-docker connect aica-technology-zmq-simulator-runtime -u ros2
```

If you create files within the container that are relevant (for example RViz config files), copy them to the host with

```console
docker cp aica-technology-zmq-simulator-runtime:/home/<user>/path/within/container/ /host/path/target
```

### SSH server for remote development

To start up an SSH server in the background for remote development, run

```console
aica-docker server aica-technology/zmq-simulator -p 7770 -u ros2
```

## Running instructions

Once inside the container, start the simulation with

```console
python3 pybullet_zmq/simulator.py
```

The simulation can be configured by setting the values in [franka_config.yaml](pybullet_zmq/config/franka_config.yaml)
to the desired ones.

### Pycharm setup for integrated Python development

See [ros-pycharm-example](https://github.com/domire8/ros-pycharm-example) for help on configuring a Python interpreter
with an SSH host.
