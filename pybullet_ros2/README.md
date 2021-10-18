# PyBullet ROS2

PyBullet Simulator with a ROS2 communication interface.

## Usage instructions

Make sure to clone [aica-technology/docker-images](https://github.com/aica-technology/docker-images) and install the
scripts before you start, as the instructions below rely on those scripts.

### Build the image

To build the image,

```console
bash build-server.sh
```

### Run image interactively

```console
aica-docker interactive aica-technology/ros2-simulator:galactic -u ros2
# inside the container
ros2 launch pybullet_ros2 franka.launch.py
```

If you create files within the container that are relevant (for example RViz config files), copy them to the host with

```console
docker cp aica-technology-ros2-simulator-galactic-runtime:/home/<user>/path/within/container/ /host/path/target
```

### SSH server for remote development

To start up an SSH server with port number 7771 in the background for remote development, run

```console
bash build-server.sh -s
```

#### Pycharm setup for integrated Python development

See [ros-pycharm-example](https://github.com/domire8/ros-pycharm-example) for help on configuring a Python interpreter
with an SSH host.
