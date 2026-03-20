#!/bin/bash

echo "[INFO] Sourcing ROS environments..."
source /opt/ros/humble/local_setup.sh
source /root/ros_ws_aux/install/local_setup.sh

if [ $BRIDGE_TYPE = "ASM_CAN" ]; then
    echo "[INFO] Running CAN interface setup..."

    source /root/dspace_bridge_ws/install/local_setup.sh
    PARAMS_FILE="${ASM_SOCKETCAN_PARAMS_FILE:-$(ros2 pkg prefix asm_socketcan_bridge)/share/asm_socketcan_bridge/config/asm_socketcan_bridge.yaml}"

    if [ ! -f "$PARAMS_FILE" ]; then
        echo "[ERROR] Parameter file not found: $PARAMS_FILE"
        exit 1
    fi

    if [ -z "$CAN_INTERFACE" ]; then
        CAN_INTERFACE=$(awk -F':' '/^[[:space:]]*can\.interface:/ {
            value=$2
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
            gsub(/"/, "", value)
            print value
            exit
        }' "$PARAMS_FILE")
    fi

    if [ -z "$CAN_INTERFACE" ]; then
        echo "[ERROR] No CAN interface configured. Set can.interface in: $PARAMS_FILE"
        sleep 1h
        exit 1
    fi
    echo "[INFO] Using CAN interface: $CAN_INTERFACE"

    if [ $? -ne 0 ]; then
        echo "[ERROR] CAN interface setup failed."
        exit 1
    fi

    echo "[INFO] Starting AsmSocketCanBridgeNode via launch file..."
    exec ros2 launch asm_socketcan_bridge asm_socketcan_bridge.launch.py params_file:="$PARAMS_FILE"

elif  [ $BRIDGE_TYPE = "AURELION_ROS2" ]; then
    echo "[INFO] Starting AurelionRos2BridgeNode..."
    source /root/dspace_bridge_ws/install/local_setup.sh
    exec ros2 run aurelion_ros2_bridge AurelionRos2BridgeNode --ros-args -p use_sim_time:=$SIM_CLOCK_MODE

elif  [ $BRIDGE_TYPE = "CAN_DBW" ]; then
    source /opt/ros/humble/local_setup.bash
    source /root/ros_dbw_ws/install/local_setup.sh
    cd /root/ros_dbw_ws/
    ros2 launch raptor_dbw_can raptor_dbw_can_launch_entire.py

elif  [ $BRIDGE_TYPE = "FOXGLOVE" ]; then
    echo "[INFO] Starting Foxglove bridge..."
    exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml
else
    echo "[ERROR] Unknown bridge type"
    exit 1
fi
