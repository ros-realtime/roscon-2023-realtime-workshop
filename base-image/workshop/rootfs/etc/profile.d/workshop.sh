source /opt/ros/humble/setup.bash

colcon() {
  if [ "$(pwd)" == "/code" ]; then
    echo "ERROR: Please do not run colcon commands in /code." >&2
    echo "       Instead, run it in one of the exercise directories." >&2
    return 1
  fi

  /usr/bin/colcon "$@"

  # Make the attendees' lives easier?? Or maybe worse.
  # if [ "$1" == "build" ]; then
  #   if [ -f install/setup.bash ]; then
  #     source install/setup.bash
  #   fi
  # fi
}

export ROS_DOMAIN_ID=21
cd /code

