ARG BASE_IMAGE=scratch
FROM $BASE_IMAGE AS dspace_ros_base
# Adds all dspace specific dependencies to a ros base image.
SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=jazzy
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

RUN mkdir -p /etc/cyclonedds && \
    echo 'export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml' >> /root/.bashrc
COPY cyclonedds.xml /etc/cyclonedds/cyclonedds.xml

RUN apt-get update && \
    apt-get install -y --no-install-recommends openssh-server xauth build-essential libboost-all-dev python3-colcon-common-extensions git cmake zip software-properties-common gdb wget python3-pip python3-setuptools ros-$ROS_DISTRO-rmw-cyclonedds-cpp ros-$ROS_DISTRO-tf2 ros-$ROS_DISTRO-foxglove-msgs

RUN rosdep update --include-eol-distros && \
    echo 'source /opt/ros/$ROS_DISTRO/local_setup.bash' >> /root/.bashrc


COPY ros_ws_aux /root/ros_ws_aux

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

FROM dspace_bridge_dev AS socketcan_dbw_bridge
ENV BRIDGE_TYPE="CAN_DBW"
# Build bridge node and enable autostart for headless execution
COPY ros_dbw_ws/src /root/ros_dbw_ws/src
# Add dependencies
RUN cd /root/ros_dbw_ws/src && git clone https://github.com/autowarefoundation/ros2_socketcan.git
# Build ros
# NOTE: /opt/race_common must be provided by BASE_IMAGE
RUN source /opt/ros/$ROS_DISTRO/local_setup.bash && \
    source /root/ros_ws_aux/install/local_setup.bash && \
    source /opt/race_common/install/setup.bash && \
    rosdep install -i --from-path /root/ros_dbw_ws/src --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install --cmake-clean-first --base-paths /root/ros_dbw_ws/ --build-base /root/ros_dbw_ws/build --install-base /root/ros_dbw_ws/install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo 'source /root/ros_dbw_ws/install/local_setup.bash' >> /root/.bashrc

ENTRYPOINT ["sh", "-c", "/root/runtime_scripts/entrypoint.sh"]

FROM dspace_bridge_dev AS asm_socketcan_bridge
ENV BRIDGE_TYPE="ASM_CAN"
# Build bridge node and enable autostart for headless execution
RUN apt-get update && \
    apt-get install -y --no-install-recommends can-utils iproute2 kmod
COPY dspace_bridge_ws/src/asm_socketcan_bridge /root/dspace_bridge_ws/src/asm_socketcan_bridge
RUN source /opt/ros/$ROS_DISTRO/local_setup.bash && \
    source /root/ros_ws_aux/install/local_setup.bash && \
    rosdep install -i --from-path /root/dspace_bridge_ws/src --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install --cmake-clean-first --base-paths /root/dspace_bridge_ws/ --build-base /root/dspace_bridge_ws/build --install-base /root/dspace_bridge_ws/install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo 'source /root/dspace_bridge_ws/install/local_setup.bash' >> /root/.bashrc

ENTRYPOINT ["sh", "-c", "/root/runtime_scripts/entrypoint.sh"]

FROM dspace_bridge_dev AS aurelion_ros2_bridge
ENV BRIDGE_TYPE="AURELION_ROS2"
# Build bridge node and enable autostart for headless execution
COPY dspace_bridge_ws/src/aurelion_ros2_bridge /root/dspace_bridge_ws/src/aurelion_ros2_bridge
RUN source /opt/ros/$ROS_DISTRO/local_setup.bash && \
    source /root/ros_ws_aux/install/local_setup.bash && \
    rosdep install -i --from-path /root/dspace_bridge_ws/src --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install --cmake-clean-first --base-paths /root/dspace_bridge_ws/ --build-base /root/dspace_bridge_ws/build --install-base /root/dspace_bridge_ws/install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    echo 'source /root/dspace_bridge_ws/install/local_setup.bash' >> /root/.bashrc

ENTRYPOINT ["sh", "-c", "/root/runtime_scripts/entrypoint.sh"]

FROM dspace_bridge_dev AS ros2_foxglove_bridge
ENV BRIDGE_TYPE="FOXGLOVE"
# Install Foxglove bridge 
RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-$ROS_DISTRO-foxglove-bridge

ENTRYPOINT ["sh", "-c", "/root/runtime_scripts/entrypoint.sh"]