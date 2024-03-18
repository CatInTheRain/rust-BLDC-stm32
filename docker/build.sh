#!/bin/bash

docker build \
    --build-arg USER_ID=1000 \
    --build-arg GROUP_ID=1000 \
    -f ./Dockerfile \
    -t rust_stm32f7:ubuntu22 \
    .
