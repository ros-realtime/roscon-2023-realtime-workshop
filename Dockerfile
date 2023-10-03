FROM ros:humble-ros-base

ENV ROS_LOCALHOST_ONLY=1
ENV ROS_DOMAIN_ID=21

RUN set -xe; \
    export DEBIAN_FRONTEND=noninteractive; \
    apt-get update; \
    apt-get install -y \
      # ROS dependencies
      ros-humble-rviz2 \
      ros-humble-joint-state-publisher-gui \
      # cactus_rt dependencies
      protobuf-compiler \
      # Utility tools
      sudo \
      git \
      stress-ng \
    ; \
    apt-get clean; \
    rm -rf /var/lib/apt/lists/*;

COPY docker/bin /opt/bin
COPY docker/profile.d/custom.sh /etc/profile.d/custom.sh
COPY prebuilts/perfetto /opt/perfetto

ENTRYPOINT ["/opt/bin/entrypoint"]
