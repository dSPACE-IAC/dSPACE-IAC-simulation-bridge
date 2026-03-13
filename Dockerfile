# syntax=docker/dockerfile:1.4
ARG BASE_IMAGE
FROM --platform=linux/amd64 $BASE_IMAGE AS dspace_ros_base
# Adds all dspace specific dependencies to a ros base image.
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["tail", "-f", "/dev/null"]

ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/VESI/lib
ENV SIM_CLOCK_MODE=false
ENV ENABLE_LOG=false
ENV ROS_DISTRO=jazzy
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

RUN apt-get update && \
    apt-get install -y --no-install-recommends openssh-server xauth build-essential libboost-all-dev python3-colcon-common-extensions git cmake zip g++ software-properties-common gdb wget python3-pip debconf python3 python3-setuptools ros-$ROS_DISTRO-rmw-cyclonedds-cpp ros-$ROS_DISTRO-tf2 ros-$ROS_DISTRO-foxglove-msgs

RUN rosdep update --include-eol-distros && \
    echo 'source /opt/ros/$ROS_DISTRO/local_setup.bash' >> /root/.bashrc

RUN mkdir -p /etc/cyclonedds && \
    echo 'export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml' >> /root/.bashrc
COPY cyclonedds.xml /etc/cyclonedds/cyclonedds.xml

RUN ldconfig

# Build bridge message definitions to enable dev working environment
RUN mkdir -p /root/ros_ws_aux
COPY ros_ws_aux /root/ros_ws_aux

# Add novatel + vectornav message packages ONLY (now included in ros_ws_aux)
# RUN --mount=type=ssh cd /root/ros_ws_aux/src && \
#     git clone --depth 1 git@github.com:airacingtech/novatel_oem7_driver.git && \
#     mv novatel_oem7_driver/src/novatel_oem7_msgs . && \
#     rm -rf novatel_oem7_driver && \
#     git clone --depth 1 git@github.com:airacingtech/vectornav-rtcm.git && \
#     mv vectornav-rtcm/vectornav_msgs . && \
#     rm -rf vectornav-rtcm

RUN source /opt/ros/$ROS_DISTRO/local_setup.bash && \
    rosdep install -i --from-path /root/ros_ws_aux/src --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install --base-paths /root/ros_ws_aux --build-base /root/ros_ws_aux/build --install-base /root/ros_ws_aux/install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    echo 'source /root/ros_ws_aux/install/local_setup.bash' >> /root/.bashrc

FROM dspace_ros_base AS dspace_bridge_base
# Add V-ESI API to enable message transfer
RUN mkdir -p /opt/VESI/lib 
COPY V-ESI-API/linux/libVESIAPI.so /opt/VESI/lib/

FROM dspace_bridge_base AS dspace_bridge_dev
# Prepares image to mount and build source code into the container during runtime (development usecase)
RUN mkdir -p /root/runtime_scripts && \
    mkdir -p /root/record_log
COPY runtime_scripts /root/runtime_scripts
RUN chmod +x /root/runtime_scripts/*

WORKDIR /root/runtime_scripts

ENTRYPOINT [ "tail", "-f", "/dev/null" ]

FROM dspace_bridge_dev AS asm_ros2_bridge_simphera
ENV BRIDGE_TYPE="ASM_ROS2"
# Build bridge node and enable autostart for headless execution
COPY dspace_bridge_ws/src/asm_ros2_bridge /root/dspace_bridge_ws/src/asm_ros2_bridge
RUN mkdir -p /root/record_log && \
    source /opt/ros/$ROS_DISTRO/local_setup.bash && \
    source /root/ros_ws_aux/install/local_setup.bash && \
    source /opt/race_common/install/setup.bash && \
    rosdep install -i --from-path /root/dspace_bridge_ws/src --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install --cmake-clean-first --base-paths /root/dspace_bridge_ws/ --build-base /root/dspace_bridge_ws/build --install-base /root/dspace_bridge_ws/install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo 'source /root/dspace_bridge_ws/install/local_setup.bash' >> /root/.bashrc

ENTRYPOINT ["sh", "-c", "/root/runtime_scripts/entrypoint.sh"]
# --ros-args -p use_sim_time:=$SIM_CLOCK_MODE (For later)


FROM dspace_bridge_dev AS asm_socketcan_bridge_simphera
ENV BRIDGE_TYPE="ASM_CAN"
# Build bridge node and enable autostart for headless execution
RUN apt-get update && \
    apt-get install -y --no-install-recommends can-utils iproute2 kmod
COPY dspace_bridge_ws/src/asm_socketcan_bridge /root/dspace_bridge_ws/src/asm_socketcan_bridge
RUN mkdir -p /root/record_log && \
    source /opt/ros/$ROS_DISTRO/local_setup.bash && \
    source /root/ros_ws_aux/install/local_setup.bash && \
    rosdep install -i --from-path /root/dspace_bridge_ws/src --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install --cmake-clean-first --base-paths /root/dspace_bridge_ws/ --build-base /root/dspace_bridge_ws/build --install-base /root/dspace_bridge_ws/install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo 'source /root/dspace_bridge_ws/install/local_setup.bash' >> /root/.bashrc

ENTRYPOINT ["sh", "-c", "/root/runtime_scripts/entrypoint.sh"]

FROM dspace_bridge_dev AS aurelion_ros2_bridge_simphera
ENV BRIDGE_TYPE="AURELION_ROS2"
# Build bridge node and enable autostart for headless execution
COPY dspace_bridge_ws/src/aurelion_ros2_bridge /root/dspace_bridge_ws/src/aurelion_ros2_bridge
RUN mkdir -p /root/record_log && \
    source /opt/ros/$ROS_DISTRO/local_setup.bash && \
    source /root/ros_ws_aux/install/local_setup.bash && \
    rosdep install -i --from-path /root/dspace_bridge_ws/src --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install --cmake-clean-first --base-paths /root/dspace_bridge_ws/ --build-base /root/dspace_bridge_ws/build --install-base /root/dspace_bridge_ws/install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    echo 'source /root/dspace_bridge_ws/install/local_setup.bash' >> /root/.bashrc

ENTRYPOINT ["sh", "-c", "/root/runtime_scripts/entrypoint.sh"]

FROM dspace_bridge_dev AS dspace_foxglove_bridge
ENV BRIDGE_TYPE="FOXGLOVE"
# Install Foxglove bridge 
RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-$ROS_DISTRO-foxglove-bridge

ENTRYPOINT ["sh", "-c", "/root/runtime_scripts/entrypoint.sh"]
