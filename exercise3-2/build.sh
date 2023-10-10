#!/bin/bash

if [ ! -d build ]; then
  tar xzf /opt/exercise3-2-build-output.tar.gz
fi

colcon build
