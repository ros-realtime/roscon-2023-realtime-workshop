#!/bin/bash

set -e

cd ..

if [ ! -f docker/image.tar.gz ]; then
  docker/build.sh
else
  echo "WARNING: NOT REBUILDING DOCKER IMAGE"
  echo "WARNING: NOT REBUILDING DOCKER IMAGE"
  echo "WARNING: NOT REBUILDING DOCKER IMAGE"
  echo "WARNING: NOT REBUILDING DOCKER IMAGE"
  echo "WARNING: NOT REBUILDING DOCKER IMAGE"
fi

mkdir -p base-image/cache

set -x

rm -rf base-image/workshop/rootfs/var/www/html/data/
mkdir base-image/workshop/rootfs/var/www/html/data
ln docker/image.tar.gz base-image/workshop/rootfs/var/www/html/data/image.tar.gz

rm -rf base-image/workshop/rootfs/var/www/html/perfetto
cp -ar prebuilts/perfetto base-image/workshop/rootfs/var/www/html/perfetto

cd base-image
sudo ../vendor/ros-realtime-rpi4-image/ros-rt-img build jammy-rt jammy-rt-humble workshop
