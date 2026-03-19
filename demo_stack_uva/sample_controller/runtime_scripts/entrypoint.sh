#! /bin/bash
echo "[INFO] Sourcing ROS environments..."
source /opt/ros/humble/local_setup.sh
source /root/ros_ws_aux/install/local_setup.sh
source /root/ros_ws/install/setup.bash

echo "[INFO] Running CAN interface setup..."
echo "[INFO] Checking for privileged mode..."

PARAMS_FILE="${NPC_CONTROLLER_PARAMS_FILE:-$(ros2 pkg prefix npc_controller)/share/npc_controller/config/ims.param.yaml}"

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

echo "[INFO] Starting NpcController via launch file..."
ros2 launch npc_controller dspace.launch.py car_ip:=10.6.0.5 car_port:=60221
