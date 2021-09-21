# PyBullet ROS2

## Usage instructions

Make sure to clone [aica-technology/docker-images](https://github.com/aica-technology/docker-images) and install the
scripts before you start, as the instructions below rely on those scripts.

### Build the image

To build the image,

```bash
bash build.sh
```

### Run image interactively

```bash
aica-docker interactive aica-technology/ros2-simulator:galactic -u ros2
```

If you want to connect to the image to have a more terminals, run

```bash
aica-docker connect aica-technology-ros2-simulator-galactic-runtime -u ros2
```

If you create files within the container that are relevant (for example RViz config files), copy them to the host with

```bash
docker cp aica-technology-ros2-simulator-galactic-runtime:/home/<user>/path/within/container/ /host/path/target
```

### SSH server for remote development

Run the image as server with port number 7771

```bash
aica-docker server aica-technology/ros2-simulator:galactic -p 7771 -u ros2
```

If you want to connect to the image to have a more terminals, run

```bash
aica-docker connect aica-technology-ros2-simulator-galactic-ssh -u ros2
```

If you create files within the container that are relevant (for example RViz config files), copy them to the host with

```bash
docker cp aica-technology-ros2-simulator-galactic-ssh:/home/<user>/path/within/container/ /host/path/target
```

### Pycharm setup for integrated Python development

TODO when [ros-pycharm-example](https://github.com/domire8/ros-pycharm-example) PRs are merged.
