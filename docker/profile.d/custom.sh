source /opt/ros/humble/setup.bash

if [ -d /code ]; then
  cd /code
fi

colcon() {
  if [ "$(pwd)" == "/code" ]; then
    echo "ERROR: Please do not run colcon commands in /code." >&2
    echo "       Instead, run it in one of the exercise directories." >&2
    return 1
  fi

  if [ "$(pwd)" == "$CONTAINER_WORKSPACE_FOLDER" ]; then
    echo "ERROR: Please do not run colcon commands in $CONTAINER_WORKSPACE_FOLDER." >&2
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
