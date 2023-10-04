#!/bin/bash

if [ "$(basename $(pwd))" != "exercise3-2" ]; then
  echo "ERROR: please make sure you run this command in the exercise workspace" >&2
  exit 1
fi

if [ ! -f install/setup.bash ]; then
  echo 'ERROR: colcon build has not finished running yet. Please run `colcon build` first.' >&2
  exit 1
fi

if [ -n "$COLCON_PREFIX_PATH" ]; then
  echo 'ERROR: You have already sourced install/setup.bash. To avoid potential conflict, please create a new terminal and use ./run.sh instead!' >&2
  exit 1
fi

source install/setup.bash
exec ./install/camera_demo_3_2/bin/camera_demo
