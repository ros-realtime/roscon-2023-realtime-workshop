#!/bin/bash
source install/setup.bash

if [ -d /opt/ros2-rt-rpi4/ ]; then
  ros2 launch inverted_pendulum_example_2_2 exercise2-2.launch.py
else
  ros2 launch inverted_pendulum_example_2_2 exercise2-2.launch.py rviz:=true
fi
