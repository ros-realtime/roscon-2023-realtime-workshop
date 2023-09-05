ARG ROS_VERSION
FROM osrf/ros:${ROS_VERSION}-desktop

ARG ROS_VERSION
ENV ROS_DISTRO=${ROS_VERSION}
ARG WORKSPACE
ARG USERNAME=user

RUN set -xe; \
    adduser --gecos "" --disabled-password $USERNAME; \
    usermod -a -G sudo $USERNAME; \
    usermod -a -G video $USERNAME; \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudoers;

RUN set -xe; \
    export DEBIAN_FRONTEND=noninteractive; \
    apt-get update; \
    apt-get install -y \
        clang-tidy \
        clang-tools \
        clang-format-12 \
        cmake \
        gdb \
        less \
        libbenchmark-dev \
        libgtest-dev \
        mesa-utils \
        nano \
        protobuf-compiler \
        python3-pip \
        ros-${ROS_VERSION}-joint-state-publisher-gui \
        ros-${ROS_VERSION}-rmw-cyclonedds-cpp \
        wget \
        xdg-utils \
    ; \
    echo "export CONTAINER_WORKSPACE_FOLDER=${WORKSPACE}" > /etc/default/container-workspace-folder; \
    chsh -s /bin/bash $USERNAME

COPY ros-activate.sh /etc/profile.d/ros-activate.sh

CMD ["sh", "-c", "tail -f /dev/null"]
