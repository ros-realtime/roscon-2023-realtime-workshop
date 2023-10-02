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
      ros-humble-rmw \
      ros-humble-lifecycle-msgs \
      ros-humble-rclcpp \
      ros-humble-rcl-lifecycle \
      ros-humble-rosidl-typesupport-cpp \
      ros-humble-ament-cmake-gtest \
      ros-humble-ament-lint-auto \
      ros-humble-ament-lint-common \
      ros-humble-mimick-vendor \
      ros-humble-performance-test-fixture \
      ros-humble-rcpputils \
      ros-humble-rcutils \
      ros-humble-test-msgs \
      ros-humble-ament-cmake-ros \
      ros-humble-ament-index-cpp \
      ros-humble-builtin-interfaces \
      ros-humble-rcl-interfaces \
      ros-humble-rosgraph-msgs \
      ros-humble-rosidl-runtime-cpp \
      ros-humble-rosidl-typesupport-c \
      ros-humble-libstatistics-collector \
      ros-humble-rcl \
      ros-humble-rcl-yaml-param-parser \
      ros-humble-statistics-msgs \
      ros-humble-tracetools \
      ros-humble-ament-cmake-gmock \
      ros-humble-ament-cmake-google-benchmark \
      ros-humble-rmw-implementation-cmake \
      ros-humble-rosidl-default-generators \
      ros-humble-ament-cmake-gen-version-h \
      python3-dev \
      ros-humble-std-msgs \
      ros-humble-ament-cmake \
      ros-humble-class-loader \
      ros-humble-composition-interfaces \
      ros-humble-launch-testing \
      ros-humble-rosidl-runtime-c \
      ros-humble-action-msgs \
      ros-humble-rcl-action \
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
