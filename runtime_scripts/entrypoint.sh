#!/bin/bash

echo "[INFO] Sourcing ROS environments..."
source /opt/ros/humble/local_setup.sh
source /root/ros_ws_aux/install/local_setup.sh
source /root/dspace_bridge_ws/install/local_setup.sh

if [ $BRIDGE_TYPE = "ASM_CAN" ]; then
    echo "[INFO] Running CAN interface setup..."
    echo "[INFO] Checking for privileged mode..."

    # Try to create socket to test for privileges
    ip link add dev $CAN_INTERFACE type vcan 2>/dev/null

    if [ $? -eq 0 ]; then
        echo "[INFO] Privileged mode detected. Setting up vcan interfaces..."
        ip link add dev $CAN_INTERFACE type vcan
        ip link set up $CAN_INTERFACE
    else
        echo "[WARN] Not running in privileged mode or vcan module not available. Skipping automatic can interface setup. Setup of the interfaces needs to be done on the host and container needs to be started in host network."
    fi

    if [ $? -ne 0 ]; then
        echo "[ERROR] CAN interface setup failed."
        exit 1
    fi

    echo "[INFO] Starting AsmSocketcanBridgeNode..."
    # exec /root/dspace_bridge_ws/install/asm_socketcan_bridge/lib/asm_socketcan_bridge/AsmSocketCanBridgeNode
    exec ros2 run asm_socketcan_bridge AsmSocketCanBridgeNode --ros-args -p use_sim_time:=$SIM_CLOCK_MODE

elif  [ $BRIDGE_TYPE = "ASM_ROS2" ]; then
    echo "[INFO] Starting AsmRos2BridgeNode..."
    # exec /root/dspace_bridge_ws/install/asm_socketcan_bridge/lib/asm_ros2_bridge/AsmRos2BridgeNode
    exec ros2 run asm_ros2_bridge AsmRos2BridgeNode --ros-args -p use_sim_time:=$SIM_CLOCK_MODE

elif  [ $BRIDGE_TYPE = "AURELION_ROS2" ]; then
    echo "[INFO] Starting AurelionRos2BridgeNode..."
    # exec /root/dspace_bridge_ws/install/asm_socketcan_bridge/lib/asm_ros2_bridge/AurelionRos2BridgeNode
    exec ros2 run aurelion_ros2_bridge AurelionRos2BridgeNode --ros-args -p use_sim_time:=$SIM_CLOCK_MODE

elif  [ $BRIDGE_TYPE = "FOXGLOVE" ]; then
    echo "[INFO] Starting Foxglove bridge..."
    exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml
else
    echo "[ERROR] Unknown bridge type"
    exit 1
fi
