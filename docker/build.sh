#!/bin/bash

DOCKER_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
IMAGE_NAME=roscon-2023-realtime-workshop

set -xe

cd $DOCKER_DIR
chmod 0644 rpi_ssh.conf # Otherwise ssh will freak out. Also Git doesn't record very granular permissions.

cd $DOCKER_DIR/..
docker build -t $IMAGE_NAME .

docker save $IMAGE_NAME | pv | gzip >docker/docker-image.tar.gz
