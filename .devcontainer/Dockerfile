FROM roscon-2023-realtime-workshop:latest

ARG WORKSPACE
ARG USERNAME=user

RUN set -xe; \
    adduser --gecos "" --disabled-password $USERNAME; \
    usermod -a -G sudo $USERNAME; \
    usermod -a -G video $USERNAME; \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudoers;

RUN set -xe; \
    echo "export CONTAINER_WORKSPACE_FOLDER=${WORKSPACE}" > /etc/default/container-workspace-folder; \
    chsh -s /bin/bash $USERNAME

COPY ros-activate.sh /etc/profile.d/ros-activate.sh
