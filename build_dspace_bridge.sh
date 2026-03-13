#!/bin/bash

# define general parameters
REGISTRY="722180079256.dkr.ecr.eu-central-1.amazonaws.com"
TEAM="airacingtech"
DATE=$(date '+%Y-%m-%d')
BASE_IMAGE="ghcr.io/airacingtech/art_ros_jazzy_cpu-built-dspace:stable"

# define name of the image
NAME_BUILD=$TEAM/iac_dspace_bridge_build:$DATE
NAME_DEV=$TEAM/iac_dspace_bridge_dev:$DATE
NAME_ASM_ROS2_SIMPHERA=$TEAM/iac_asm_ros2_bridge:$DATE
NAME_ASM_SOCKETCAN_SIMPHERA=$TEAM/iac_asm_socketcan_bridge:$DATE
NAME_IAC_AURELION_ROS2_SIMPHERA=$TEAM/iac_aurelion_ros2_bridge:$DATE
NAME_FOXGLOVE=$TEAM/iac_dspace_bridge_foxglove:$DATE

# build image
echo "---------- build general dspace_bridge dev version ----------"
docker build -t $NAME_DEV --target=dspace_bridge_dev --build-arg BASE_IMAGE=$BASE_IMAGE -f Dockerfile .
echo "---------- build asm_ros2_bridge simphera version ----------"
docker build -t $NAME_ASM_ROS2_SIMPHERA --target=asm_ros2_bridge_simphera --build-arg BASE_IMAGE=$BASE_IMAGE -f Dockerfile .
echo "---------- build asm_socketcan_bridge simphera version ----------"
docker build -t $NAME_ASM_SOCKETCAN_SIMPHERA --target=asm_socketcan_bridge_simphera --build-arg BASE_IMAGE=$BASE_IMAGE -f Dockerfile .
echo "---------- build iac_aurelion_ros2_bridge simphera version ----------"
docker build -t $NAME_IAC_AURELION_ROS2_SIMPHERA --target=aurelion_ros2_bridge_simphera --build-arg BASE_IMAGE=$BASE_IMAGE -f Dockerfile .
echo "---------- build dspace_bridge foxglove version ----------"
docker build -t $NAME_FOXGLOVE --target=dspace_foxglove_bridge --build-arg BASE_IMAGE=$BASE_IMAGE -f Dockerfile .
