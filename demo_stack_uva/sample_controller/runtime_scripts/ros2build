#!/bin/bash
source /opt/ros/humble/local_setup.sh
source /root/ros_ws_aux/install/local_setup.bash
rosdep install -i --from-path /root/ros_ws/src --rosdistro humble -y
colcon build --symlink-install --cmake-clean-first --base-paths /root/ros_ws/ --build-base /root/ros_ws/build --install-base /root/ros_ws/install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo 
