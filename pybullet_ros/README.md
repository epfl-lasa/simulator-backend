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
aica-docker interactive aica-technology/ros-simulator:noetic -u ros
```

If you want to connect to the image to have a more terminals, run

```bash
aica-docker connect aica-technology-ros-simulator-noetic-runtime -u ros
```

If you create files within the container that are relevant (for example RViz config files), copy them to the host with

```bash
docker cp aica-technology-ros-simulator-noetic-runtime:/home/<user>/path/within/container/ /host/path/target
```

### SSH server for remote development

Run the image as server with port number 7772

```bash
aica-docker server aica-technology/ros-simulator:noetic -p 7772 -u ros
```

If you want to connect to the image to have a more terminals, run

```bash
aica-docker connect aica-technology-ros-simulator-noetic-ssh -u ros
```

If you create files within the container that are relevant (for example RViz config files), copy them to the host with

```bash
docker cp aica-technology-ros-simulator-noetic-ssh:/home/<user>/path/within/container/ /host/path/target
```

### Pycharm setup for integrated Python development

TODO when [ros-pycharm-example](https://github.com/domire8/ros-pycharm-example) PRs are merged.
