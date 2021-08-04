# simulator-backend

## Usage instructions

There are two options to use and/or develop the packages present in this repository. Either you build and run
a [runtime image](#runtime-image) or you spin up a container in the background
for [remote development](#ssh-server-for-remote-development).

Make sure to clone [aica-technology/docker-images](https://github.com/aica-technology/docker-images) before you start,
as the instructions below rely on scripts from this repository. In either case, use the commands below to set everything
up correctly.

### Runtime image

To build a runtime image,

```bash
bash build.sh --target runtime
```

and to run the image interactively

```bash
cd path/to/aica-technology/docker-images/scripts
bash run_interactive.sh aica-technology/simulator-backend:runtime -u ros2
```

If you want to connect to the image to have a more terminals, run

```bash
cd path/to/aica-technology/docker-images/scripts
bash connect.sh aica-technology-simulator-backend-runtime-runtime -u ros2
```

If you create files within the container that are relevant (for example RViz config files), copy them to the host with

```bash
docker cp aica-technology-simulator-backend-runtime-runtime:/home/<user>/path/within/container/ /host/path/target
```

### SSH server for remote development

To build the image for remote development,

```bash
bash build.sh --target workspace
```

and to run the image as server with port number 7777

```bash
cd path/to/aica-technology/docker-images/scripts
bash server.sh aica-technology/simulator-backend:workspace -p 7777 -u ros2
```

If you want to connect to the image to have a more terminals, run

```bash
cd path/to/aica-technology/docker-images/scripts
bash connect.sh aica-technology-simulator-backend-workspace-ssh -u ros2
```

If you create files within the container that are relevant (for example RViz config files), copy them to the host with

```bash
docker cp aica-technology-simulator-backend-workspace-ssh:/home/<user>/path/within/container/ /host/path/target
```

### Pycharm setup for integrated Python development

TODO when [ros-pycharm-example](https://github.com/domire8/ros-pycharm-example) PRs are merged.
