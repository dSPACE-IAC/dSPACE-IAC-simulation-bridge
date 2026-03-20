#!/bin/bash

# define general parameters
TEAM="dspace"
DATE=$(date '+%Y-%m-%d')

# define name of the image
NAME_BUILD=$TEAM/iac_dspace_bridge_build:$DATE
NAME_DEV=$TEAM/iac_dspace_bridge_dev:$DATE
NAME_ASM_SOCKETCAN=$TEAM/iac_asm_socketcan_bridge:$DATE
NAME_IAC_AURELION_ROS2=$TEAM/iac_aurelion_ros2_bridge:$DATE
NAME_IAC_SOCKETCAN_DBW=$TEAM/iac_socketcan_dbw_bridge:$DATE
NAME_FOXGLOVE=$TEAM/iac_ros2_foxglove_bridge:$DATE

# build image
echo "---------- build general dspace_bridge_dev ----------"
docker build -t $NAME_DEV --target=dspace_bridge_dev --platform=linux/amd64 -f Dockerfile ..
echo "---------- build asm_socketcan_bridge ----------"
docker build -t $NAME_ASM_SOCKETCAN --target=asm_socketcan_bridge --platform=linux/amd64 -f Dockerfile ..
echo "---------- build iac_aurelion_ros2_bridge ----------"
docker build -t $NAME_IAC_AURELION_ROS2 --target=aurelion_ros2_bridge --platform=linux/amd64 -f Dockerfile ..
echo "---------- build iac_socketcan_dbw_bridge ----------"
docker build -t $NAME_IAC_SOCKETCAN_DBW --target=socketcan_dbw_bridge --platform=linux/amd64 -f Dockerfile ..
echo "---------- build ros2_foxglove_bridge ----------"
docker build -t $NAME_FOXGLOVE --target=ros2_foxglove_bridge --platform=linux/amd64 -f Dockerfile ..
