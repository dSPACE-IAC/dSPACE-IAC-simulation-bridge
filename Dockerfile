FROM ros:humble-ros-base AS dspace_ros_base
# Adds all dspace specific dependencies to a ros humble base image.
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["tail", "-f", "/dev/null"]

RUN apt-get update && \
    apt-get install -y --no-install-recommends openssh-server xauth build-essential libboost-all-dev python3-colcon-common-extensions git cmake zip g++ software-properties-common gdb wget python3-pip debconf python3 python3-setuptools ros-humble-rmw-cyclonedds-cpp ros-humble-tf2

RUN rosdep update && \
    echo 'source /opt/ros/humble/local_setup.bash' >> /root/.bashrc

# Build custom message definitions 
RUN mkdir -p /root/ros_ws_aux
COPY dSPACE-IAC-simulation-bridge/ros_ws_aux /root/ros_ws_aux

RUN source /opt/ros/humble/local_setup.bash && \
    rosdep install -i --from-path /root/ros_ws_aux/src --rosdistro humble -y && \
    colcon build --symlink-install --base-paths /root/ros_ws_aux --build-base /root/ros_ws_aux/build --install-base /root/ros_ws_aux/install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    echo 'source /root/ros_ws_aux/install/local_setup.bash' >> /root/.bashrc

FROM dspace_ros_base AS dspace_bridge_base
# Add V-ESI API to enable message transfer
RUN mkdir -p /opt/VESI/lib 
RUN mkdir -p /opt/VESI/include
COPY dSPACE-IAC-simulation-bridge/V-ESI-API/linux/libVESIAPI.so /opt/VESI/lib/
COPY dSPACE-IAC-simulation-bridge/V-ESI-API/include /opt/VESI/include/

FROM dspace_bridge_base AS dspace_bridge_dev
# Prepares image to mount and build source code into the container during runtime (development usecase)
RUN mkdir -p /root/runtime_scripts && \
    mkdir -p /root/record_log
COPY dSPACE-IAC-simulation-bridge/runtime_scripts /root/runtime_scripts
RUN chmod +x /root/runtime_scripts/*

WORKDIR /root/runtime_scripts

ENTRYPOINT [ "tail", "-f", "/dev/null" ]

FROM dspace_bridge_dev AS socketcan_dbw_bridge
ENV BRIDGE_TYPE="CAN_DBW"
# Build bridge node and enable autostart for headless execution
COPY dSPACE-IAC-simulation-bridge/ros_dbw_ws/src /root/ros_dbw_ws/src
# Build ros
RUN mkdir -p /root/record_log && \
    source /opt/ros/humble/local_setup.bash && \
    source /root/ros_ws_aux/install/local_setup.bash && \
    rosdep install -i --from-path /root/ros_dbw_ws/src --rosdistro humble -y && \
    colcon build --symlink-install --cmake-clean-first --base-paths /root/ros_dbw_ws/ --build-base /root/ros_dbw_ws/build --install-base /root/ros_dbw_ws/install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo 'source /root/ros_dbw_ws/install/local_setup.bash' >> /root/.bashrc

ENTRYPOINT ["sh", "-c", "/root/runtime_scripts/entrypoint.sh"]

FROM dspace_bridge_dev AS asm_socketcan_bridge
ENV BRIDGE_TYPE="ASM_CAN"
# Build bridge node and enable autostart for headless execution
RUN apt-get update && \
    apt-get install -y --no-install-recommends can-utils iproute2 kmod
COPY dSPACE-IAC-simulation-bridge/dspace_bridge_ws/src/asm_socketcan_bridge /root/dspace_bridge_ws/src/asm_socketcan_bridge
RUN mkdir -p /root/record_log && \
    source /opt/ros/humble/local_setup.bash && \
    source /root/ros_ws_aux/install/local_setup.bash && \
    rosdep install -i --from-path /root/dspace_bridge_ws/src --rosdistro humble -y && \
    colcon build --symlink-install --cmake-clean-first --base-paths /root/dspace_bridge_ws/ --build-base /root/dspace_bridge_ws/build --install-base /root/dspace_bridge_ws/install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo 'source /root/dspace_bridge_ws/install/local_setup.bash' >> /root/.bashrc

ENTRYPOINT ["sh", "-c", "/root/runtime_scripts/entrypoint.sh"]

FROM dspace_bridge_dev AS aurelion_ros2_bridge
ENV BRIDGE_TYPE="AURELION_ROS2"
# Build bridge node and enable autostart for headless execution
COPY dSPACE-IAC-simulation-bridge/dspace_bridge_ws/src/aurelion_ros2_bridge /root/dspace_bridge_ws/src/aurelion_ros2_bridge
RUN mkdir -p /root/record_log && \
    source /opt/ros/humble/local_setup.bash && \
    source /root/ros_ws_aux/install/local_setup.bash && \
    rosdep install -i --from-path /root/dspace_bridge_ws/src --rosdistro humble -y && \
    colcon build --symlink-install --cmake-clean-first --base-paths /root/dspace_bridge_ws/ --build-base /root/dspace_bridge_ws/build --install-base /root/dspace_bridge_ws/install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    echo 'source /root/dspace_bridge_ws/install/local_setup.bash' >> /root/.bashrc

ENTRYPOINT ["sh", "-c", "/root/runtime_scripts/entrypoint.sh"]

FROM dspace_bridge_dev AS ros2_foxglove_bridge
ENV BRIDGE_TYPE="FOXGLOVE"
# Install Foxglove bridge 
RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-humble-foxglove-bridge

ENTRYPOINT ["sh", "-c", "/root/runtime_scripts/entrypoint.sh"]
