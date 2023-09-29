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
2. Clone this repository.
3. `cd` into this repository.
4. `docker/import path/to/downloaded/image.tar.gz`.

This should import the Docker image with a name of `roscon-2023-realtime-workshop`.

### Getting the image DURING the workshop (as a last resort)

If you didn't get the image before the workshop, we will offer a way to get the
image during the workshop without access to internet. **However, try to do the
step above as the network bandwidth during the workshop is limited!**.

TBD

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
2. `docker shell`

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

**Check rviz2 is working**

After logging into the Docker container, run:

```console
$ rviz2
```

The usual rviz2 GUI window should show up if all is well.

Workshop setup for the Raspberry Pi 4
-------------------------------------
