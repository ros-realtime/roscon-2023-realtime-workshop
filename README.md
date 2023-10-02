Real-time programming with ROS 2
================================

This is the repository for the `Real-time programming with ROS 2` workshop for
ROScon 2023.

Workshop setup for laptops
--------------------------

### System requirements

The workshop code **only supports Linux**. Officially, we only test **Ubuntu
22.04** as the laptop/development operating system. However, with the Docker
setup described below, it may be possible to run this on other Linux variants,
although this is untested.

**A real-time kernel is not necessary**, although you may see decreased maximum
latency numbers if you have a real-time kernel. For reference, the workshop code
is developed on regular Ubuntu kernels.

**Docker for Mac and Docker for Windows are not supported. Please try to bring a
Linux laptop to the workshop for best results.**. It is possible that a virtual
machine running on Linux may work for you, although this is also not supported.

**Podman is not supported**. It might work, but we have not tested the Docker
setup below with Podman.

The code is tested on both `amd64` and `arm64` (via the Raspberry Pi 4). No
tests are made against the M1/M2 Macbooks.

### Using Docker

The simplest way to run the workshop code is to use the workshop-specific Docker
image. To get the Docker image, there are two options: (1) getting the image
before the workshop **(highly recommended)**, or (2) getting the image during
the workshop (we cannot guarantee this experience will be good due to
constraints of networking during a conference).

Before getting the image, you must install Docker Engine on your Linux machine.
Please follow the [Docker Engine installation instructions][docker-install].
**Ensure you do not use Docker Desktop as that is not supported due to its use
of a virtual machine**. At the end, you should confirm that you see the
following output when running the following commands:

```console
$ docker version | grep -A2 Client
Client: Docker Engine - Community
 Version:           24.0.6
 API version:       1.43

$ docker version | grep -A2 Server
Server: Docker Engine - Community
 Engine:
  Version:          24.0.6
```

The exact version of Docker may be different than the above output.

[docker-install]: https://docs.docker.com/engine/install/ubuntu/

### Getting the image BEFORE the workshop (highly recommended)

1. Download the image file from [the latest release](https://github.com/ros-realtime/roscon-2023-realtime-workshop/releases/latest). The file is called `image.tar.gz`.
2. Clone [roscon-2023-realtime-workshop](https://github.com/ros-realtime/roscon-2023-realtime-workshop) repository.
3. `cd` into the `roscon-2023-realtime-workshop` repository.
4. `docker/import path/to/downloaded/image.tar.gz`.

This should import the Docker image with a name of `roscon-2023-realtime-workshop`. You can check with `docker image list`

### Getting the image DURING the workshop (as a last resort)

If you didn't get the image before the workshop, we will offer a way to get the
image during the workshop without access to internet. **However, try to do the
step above as the network bandwidth during the workshop is limited!**.

1. Connect your laptop to Raspberry Pi via Ethernet.
2. Ensure the Raspberry Pi is powered on.
3. You should be able to `ping 192.168.10.1` once the Pi is powered on and it is connected to the ethernet.
4. `cd` into this repository.
5. Run `docker/import`. This should automatically download and import the Docker image from the Ethernet-connected Raspberry Pi.

### Starting the Docker container

After importing the Docker image, you can start the Docker container via the
special shell file [`docker/start`](docker/start):

1. `cd` into this repository.
2. `docker/start`

This should start the Docker container and it wll mount this repository into the
`/code` directory inside the container. As a result, changes to the repository
in the host will be reflected in the container.

This should start the Docker container with all of the appropriate privileges to
run real-time-scheduled applications. It also setup the container with
everything needed to run GUI programs.

### Log into the Docker container

After starting the Docker container, you can login to the Docker container using
the special [`docker/shell`](docker/shell) script:

1. `cd` into this repository.
2. `docker/shell`

You should be greeted with something like the following:

```
To run a command as administrator (user "root"), use "sudo <command>".
See "man sudo_root" for details.

user@6896a5d8dd84:/code$
```

This script logs you into an user named `user` which has the same UID and GID as
your host user. This eliminates some of the permission errors you may encounter
with using Docker. It also allows you to run GUI programs.

### Checking everything works

After logging into the Docker container, you should check everything works:

**Check the trace viewer tools are available**

1. After starting the Docker container, open a browser.
2. Navigate to http://localhost:3100.
3. Ensure a webpage like the following screenshot loads up.

![](imgs/perfetto.png)

**Make sure all exercises compile and exercise 1 runs**

After logging into the Docker container:

```console
$ git submodule init
$ git submodule update
```

```console
$ cd /code/exercise1 && colcon build
$ cd /code/exercise2 && colcon build
$ cd /code/exercise3 && colcon build
$ cd /code/exercise4 && colcon build
```

Make sure these all build. Then make sure at least exercise 1 runs (which takes
10 seconds):

```console
$ cd /code/exercise1
$ install/latency_tester/bin/latency_tester
Testing latency for 10 seconds with 2 threads...
Latency testing complete. Trace file available at: exercise1.perfetto
```

**Known issues - docker proxy**

In case the above commands like `git submodule update` failed because of missing internet connection, you might need to configure manually the proxy for the docker container, especially if you are in a corporate network. See this [docker documentation](https://docs.docker.com/network/proxy/) or contact your local IT service to setup the proxy correctly.

Alternativly, you could execute those `git submodule` commands in the `roscon-2023-realtime-workshop` repository folder on host machine and not in docker container. Because this repository folder is mounted in the docker container, this would also work.

**Check rviz2 is working**

After logging into the Docker container, run:

```console
$ rviz2
```

The usual rviz2 GUI window should show up if all is well.

### Using Visual Studio Code

If you like to use VS code for development, we recommend the [Remote Development Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack), which enables the dev container system.

Specifically, this repository contains a [`.devcontainer`](.devcontainer) setup.
However, this setup relies on the image imported above using the `docker/import`
script. So if you haven't performed the above and launched VS code in this
repository with the dev container, it will result in an error.

To use VS code with dev container, perform the Docker importing steps above and
then simply open VS code with dev container in this repository. Everything
should just work after that.

Workshop setup for the Raspberry Pi 4
-------------------------------------

We are still finalizing whether or not we can bring a limited number of
Raspberry Pi 4s to the workshop to loan out to people. So if you want to ensure
you can follow the tutorial on a Raspberry Pi 4, it may be best to bring your
own. We recommend the following hardware:

- A Raspberry Pi 4 (4GB+ is recommended, 2GB may be OK)
- An Ethernet cable and any necessary USB Ethernet adapters to connect the Raspberry Pi directly to your laptop
- A microSD card with greater than 8GB
- The Raspberry Pi 4 power supply with USA power plugs

### Before the workshop: flashing the image

TBD

### Connecting to the Raspberry Pi 4

To connect to the Raspberry Pi:

1. Connect the Ethernet to the Raspberry Pi 4's only Ethernet port.
2. Connect the other end of the Ethernet to your laptop. You may have to do this via an USB Ethernet adapter.
3. Wait briefly for the Raspberry Pi 4 to boot up and your laptop to connect to the network. This may take a few minutes on first boot.
4. Once connected, the Raspberry Pi 4 is accessible at the IP address `192.168.10.1`.
5. You can `ssh ubuntu@192.168.10.1`. The password is **`ubuntu`**.
6. This repository is located in `~/repo`.

You can also connect to the Raspberry Pi 4's HTTP server which hosts the trace
viewer, the Docker image, a tarball of the repository. This is available at:
http://192.168.10.1
