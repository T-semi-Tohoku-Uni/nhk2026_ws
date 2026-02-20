FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Base build/test tools for ROS2 colcon workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Extra system libs required by this workspace (from README and package dependencies)
RUN apt-get update && apt-get install -y --no-install-recommends \
    gfortran \
    libboost-dev \
    libdw-dev \
    liblapack-dev \
    libnanoflann-dev \
    libopencv-dev \
    libqt5svg5-dev \
    libudev-dev \
    libzmq3-dev \
    pkg-config \
    qtbase5-dev \
    swig \
    && rm -rf /var/lib/apt/lists/*

# Prepare rosdep in container image
RUN if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update

# Install BehaviorTree.CPP 4.7.0 required by yasarobo2025_26
RUN git clone --depth 1 --branch 4.7.0 https://github.com/BehaviorTree/BehaviorTree.CPP.git /tmp/BehaviorTree.CPP && \
    cmake -S /tmp/BehaviorTree.CPP -B /tmp/BehaviorTree.CPP/build && \
    cmake --build /tmp/BehaviorTree.CPP/build -j"$(nproc)" && \
    cmake --install /tmp/BehaviorTree.CPP/build && \
    rm -rf /tmp/BehaviorTree.CPP
