#!/bin/bash

qemu-system-arm \
  -cpu cortex-m4 \
  -machine wa \
  -nographic \
  -semihosting-config enable=on,target=native \
  -gdb tcp::3333 \
  -S \
  -kernel $1

