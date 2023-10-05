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

  /usr/bin/colcon "$@"

  # Make the attendees' lives easier?? Or maybe worse.
  # if [ "$1" == "build" ]; then
  #   if [ -f install/setup.bash ]; then
  #     source install/setup.bash
  #   fi
  # fi
}

upload-to-pi() {
  pushd /code
  rsync \
    --rsh "/usr/bin/sshpass -p ubuntu ssh -o StrictHostKeyChecking=no -l ubuntu" \
    -avr \
    --no-t \
    --checksum \
    --ignore-times \
    --exclude '.git' \
    --exclude 'docker' \
    --exclude 'vendor' \
    --exclude 'base-image' \
    --exclude '*/build/' \
    --exclude '*/log/' \
    --exclude '*/install/' \
    --exclude 'imgs' \
    ./ \
    ubuntu@192.168.10.1:/code
  popd
}
