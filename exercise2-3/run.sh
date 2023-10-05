#!/bin/bash
source install/setup.bash

if [ -d /opt/ros2-rt-rpi4/ ]; then
  ros2 launch inverted_pendulum_example_2_3 exercise2-3.launch.py
else
  ros2 launch inverted_pendulum_example_2_3 exercise2-3.launch.py rviz:=true
fi
