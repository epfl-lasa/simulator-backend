# simulator-backend

## Usage instructions

Make sure to clone [aica-technology/docker-images](https://github.com/aica-technology/docker-images) before you start,
as the instructions below rely on scripts from this repository. In either case, use the commands below to set everything
up correctly.

### Build the image

To build the image,

```bash
bash build.sh
```

### Run image interactively

```bash
cd path/to/aica-technology/docker-images/scripts
bash run_interactive.sh aica-technology/simulator-backend -u ros2
```

If you want to connect to the image to have a more terminals, run

```bash
cd path/to/aica-technology/docker-images/scripts
bash connect.sh aica-technology-simulator-backend-runtime -u ros2
```

If you create files within the container that are relevant (for example RViz config files), copy them to the host with

```bash
docker cp aica-technology-simulator-backend-runtime:/home/<user>/path/within/container/ /host/path/target
```

### SSH server for remote development

Run the image as server with port number 7777

```bash
cd path/to/aica-technology/docker-images/scripts
bash server.sh aica-technology/simulator-backend -p 7777 -u ros2
```

If you want to connect to the image to have a more terminals, run

```bash
cd path/to/aica-technology/docker-images/scripts
bash connect.sh aica-technology-simulator-backend-ssh -u ros2
```

If you create files within the container that are relevant (for example RViz config files), copy them to the host with

```bash
docker cp aica-technology-simulator-backend-ssh:/home/<user>/path/within/container/ /host/path/target
```

### Pycharm setup for integrated Python development

TODO when [ros-pycharm-example](https://github.com/domire8/ros-pycharm-example) PRs are merged.

## Run the first example

After building the image, spin up an interactive runtime container:

```bash
cd path/to/aica-technology/docker-images/scripts
bash run_interactive.sh aica-technology/simulator-backend -u ros2
```

Inside the container, run

```bash
ros2 launch pybullet_ros2 visualize_franka.launch.py
```

You should see a RViz window with the Franka as well as the Joint State Publisher GUI that you can use to change the
joint configuration of the Franka.
