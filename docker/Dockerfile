FROM ubuntu:22.04
# FROM nvidia/cuda:12.3.0-runtime-ubuntu22.04
ARG USER_ID
ARG GROUP_ID

#  install dependencies
RUN apt-get update && apt-get upgrade -y && \
  apt-get install -y \
    sudo \
    git \
    wget \
    build-essential \
    cmake \
    pkg-config \
    libssl-dev \
    libudev-dev \
    curl \
    vim \
    tmux \
    tmuxinator \
    bash-completion \
    && apt-get -y autoremove \
	&& apt-get clean


# Set Clock Time
ENV TZ=Europe/Rome
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata

ARG USER=rust_stm
# set user as default so install scripts look the same for local host install
RUN addgroup --gid $GROUP_ID ${USER} && \
    adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID ${USER} && \
    adduser ${USER} sudo && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER ${USER}

WORKDIR /home/${USER}

# Get Rust
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y && \
    echo 'source $HOME/.cargo/env' >> $HOME/.bashrc

SHELL ["/bin/bash", "-c"]
ENV HOME /home/${USER}
ENV PATH "$HOME/.cargo/bin:$PATH"

# Rust Dependencies for stm32 development
RUN rustup update && \
    rustup component add llvm-tools-preview && \
    cargo install \
      cargo-binutils \
      cargo-expand \
      cargo-generate

RUN cargo install probe-rs --features cli

# Linux General Dependencies for stm32 development
RUN sudo apt-get update && \
    sudo apt-get install -y \
      gdb-multiarch \
      openocd \
      qemu-system-arm

# For Cortex-M4
RUN rustup target add thumbv7em-none-eabihf

ENV USER ${USER}

WORKDIR /home/${USER}/rust-BLDC-stm32

# ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# vim: set et fenc=utf-8 ff=unix ft=dockerfile sts=0 sw=2 ts=2 :
