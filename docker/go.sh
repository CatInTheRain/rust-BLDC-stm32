#!/bin/bash

xhost +local:docker > /dev/null

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth-n
touch $XAUTH
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

username=lattedrone
current_dir=`pwd -P`
script_dir="$( cd "$(dirname "$0")" ; pwd -P )"
# container_id=`cat "${script_dir}/docker_id"`

# last lines https://github.com/jacknlliu/ros-docker-images/issues/7

docker run -it --rm \
    --network host \
    --privileged \
    --gpus all \
    --volume=$XSOCK:$XSOCK:rw \
    --volume=$XAUTH:$XAUTH:rw \
    -v /dev:/dev \
    -v "$script_dir"/../:/home/rust_stm/rust-BLDC-stm32 \
    --env TERM=xterm-256color \
    --env="XAUTHORITY=${XAUTH}" \
    --env DISPLAY=$DISPLAY \
    --env NVIDIA_VISIBLE_DEVICES=all \
    --env NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    --name rust_container \
    rust_stm32f7:ubuntu22
