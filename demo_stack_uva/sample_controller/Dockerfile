# This a Dockerfile for building a demo stack to drive at IMS in Indy Autonomous Challenge

#This base image is quite comprehensive, but there's still a few more things we need
FROM osrf/ros:humble-desktop AS example_driving_stack_base

SHELL ["/bin/bash", "-c"]

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*


# install ros2 packages
RUN apt update && apt install -y --no-install-recommends \
    clang \
    libpcl-dev \
    libopencv-dev \
    libgeographic-dev \
    libusb-1.0-0-dev \
    libflann-dev \
    libqhull-dev \
    libalglib-dev \
    libarmadillo-dev \
    libgl1-mesa-dev \
    libpcap-dev \
    libboost-all-dev \
    python3-setuptools \
    python3-pip \
    python3-numpy \
    python3-yaml \
    python3-pillow \
    python3-tqdm \
    python3-matplotlib \
    python3-pandas \
    python3-scipy \
    python3-transforms3d \
    python3-shapely \
    libeigen3-dev \
    ninja-build \
    dos2unix \
    ros-humble-can-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN sudo update-alternatives --config cc
RUN sudo update-alternatives --config c++

RUN mkdir -p /root/.colcon
ADD demo_stack_uva/sample_controller/defaults.yaml /root/.colcon/defaults.yaml

RUN apt-get update && \
    apt-get install -y --no-install-recommends can-utils iproute2 kmod strace

RUN mkdir -p /root/ros_ws_aux
COPY ros_ws_aux /root/ros_ws_aux

RUN source /opt/ros/humble/local_setup.bash && \
    rosdep install -i --from-path /root/ros_ws_aux/src --rosdistro humble -y && \
    colcon build --symlink-install --base-paths /root/ros_ws_aux --build-base /root/ros_ws_aux/build --install-base /root/ros_ws_aux/install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    echo 'source /root/ros_ws_aux/install/local_setup.bash' >> /root/.bashrc

FROM example_driving_stack_base AS example_driving_stack_dev

RUN mkdir -p /root/runtime_scripts && \
    mkdir -p /root/record_log
COPY demo_stack_uva/sample_controller/runtime_scripts /root/runtime_scripts
RUN chmod +x /root/runtime_scripts/*
WORKDIR /root/runtime_scripts

ENTRYPOINT ["tail", "-f", "/dev/null"]

FROM example_driving_stack_dev AS example_driving_stack

RUN mkdir -p /root/ros_ws
COPY demo_stack_uva/sample_controller/ros_ws /root/ros_ws
WORKDIR /root/ros_ws
RUN source /opt/ros/humble/local_setup.bash && \
    source /root/ros_ws_aux/install/local_setup.bash && \
    rosdep install -i --from-path /root/ros_ws/src --rosdistro humble -y && \
    colcon build --symlink-install --cmake-clean-first --base-paths /root/ros_ws/ --build-base /root/ros_ws/build --install-base /root/ros_ws/install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    echo 'source /root/ros_ws/install/local_setup.bash' >> /root/.bashrc

ENTRYPOINT ["sh", "-c", "/root/runtime_scripts/entrypoint.sh"]
