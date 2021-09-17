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
aica-docker interactive aica-technology/zmq-simulator -u ros2
```

If you want to connect to the image to have a more terminals, run

```bash
aica-docker connect aica-technology-zmq-simulator-runtime -u ros2
```

If you create files within the container that are relevant (for example RViz config files), copy them to the host with

```bash
docker cp aica-technology-zmq-simulator-runtime:/home/<user>/path/within/container/ /host/path/target
```

### SSH server for remote development

Run the image as server with port number 7770

```bash
aica-docker server aica-technology/zmq-simulator -p 7770 -u ros2
```

If you want to connect to the image to have a more terminals, run

```bash
aica-docker connect aica-technology-zmq-simulator-ssh -u ros2
```

If you create files within the container that are relevant (for example RViz config files), copy them to the host with

```bash
docker cp aica-technology-zmq-simulator-ssh:/home/<user>/path/within/container/ /host/path/target
```

## Running instructions

To run the simulator, run

```console
python3 pybullet_zmq/simulator.py
```

### Pycharm setup for integrated Python development

See [ros-pycharm-example](https://github.com/domire8/ros-pycharm-example).
