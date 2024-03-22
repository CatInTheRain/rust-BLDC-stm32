#!/bin/bash

script_dir="$( cd "$(dirname "$0")" ; pwd -P )"

docker build \
    --build-arg USER_ID=1000 \
    --build-arg GROUP_ID=1000 \
    -f $script_dir/Dockerfile \
    -t rust_stm32f3:ubuntu22 \
    .
