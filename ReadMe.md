# dSPACE Indy Autonomous Challenge Simulator

This repository contains the current state of the different bridge implementations for the dSPACE IAC simulation environment.
The asm_ros2_bridge and asm_socketcan_bridge enable the data exchange between the dSPACE car and environment model simulated using the Automotive Simulation Models (ASM) and the SUT of the IAC team.
The asm_ros2_bridge creates a complete ROS2 message based interface, while the asm_socketcan_bridge uses the virtual socketcan interface in Linux for parts of the communication in order to have a closer match of the real car.
The aurelion_ros2_bridge communicates sensor data (camera, lidar, radar) between the dSPACE AURELION sensor simulation and the IAC team SUT.
The simulation data is published as ROS2 messages.
The implementation of all bridges is in form of ROS2 nodes.
There is also a foxglove bridge, which is used to make the messages published in ROS2 observable in Foxglove.

For further information please check the latest simulation package version.
This repository is meant to be provide further insides in the implementation and enables you to adapt the basic bridge implementations to the requirements of your SUT, in order to make full use of the dSPACE Indy Autonomous Challenge simulator.

## Prerequisites
You need to have the following tools installed:
- Docker (e.g. Docker Desktop)
- Docker Compose (already installed if you use Docker Desktop)

To run the complete simulation setup, you need to have access to the following instances:
- dSPACE IAC license server
- SIMPHERA AWS docker registry

## Structure of this repository
The following section provides an overview of the repository content, in order to speed up the process of finding what you are looking for and provide an understanding, where to add things when contributing.

### Dockerfile
This Dockerfile is used to create all bridge versions (asm_ros2, asm_socketcan, aurelion_ros2 and foxglove).
The images are differentiated by selecting the target of the docker build command (asm_ros2_bridge_simphera, asm_socketcan_bridge_simphera, aurelion_ros2_bridge_simphera, dspace_foxglove_bridge).
There is also a dev version (dspace_bridge_dev), which only contains the dependencies required by the bridge versions (e.g. ROS2, custom message definitions, socketcan packages etc) and a neutral entrypoint, so that could be used to quickly test new developments on the bridge.
The simphera version contains the respective compiled bridge node and an entrypoint for automatic startup of that node.
The simphera version is also recommended to be used for local testing, when working on the SUT code to check whether the system is capable to run with automatic startup procedure used in the cloud.
The foxglove version contains the foxglove-bridge application and an entrypoint to start the corresponding node.

### build_dspace_bridge.sh
This script could be used to quickly build all versions of the dSPACE bridges.

### runtime_scripts
This directory contains some scripts, which should make the work with the dev version of the bridges easier.

### ros_ws_aux
This auxilary ros workspace contains all ros packages, that are required by the several bridge versions.
Currently this contains all custom message definitions used in the system.
If you want to make your custom messages available in foxglove, the easiest way would be to add your definitions to this auxilary workspace and rebuild the foxglove bridge.
There is no need to add your custom messages to the repository, if they should only be available in foxglove.
If they should also be used by the simphera bridge please push them to a seperate branch and create a pull request.

### dspace_bridge_ws
This ros workspace contains the source code of the different bridge versions in seperate packages.
Usually there is only one bridge type per Docker image, so the respective package is copied into the image during build.
In case of the dev image, the suggested approach is to mount the dspace_bridge_ws directory into the running container.
If you want to understand how the connection to the simulator is implemented and how the interface is designed, have a look here.

## How to
The following instructions assume an execution in a Linux environment, either on a Linux host system or in WSL.
This means that all given commands and scripts are written for Linux.
However the execution also works for Windows and Mac, you just need to adapt the commands and scripts slightly for your preferred OS.

### Execute
Example workflow for sut-te-bridge:
1. Start Docker Desktop
2. Navigate into the folder, where the Docker compose is located.
3. Open a terminal and execute `docker compose -f docker-compose_sut_te_bridge_foxglove.yml up`
4. Start Foxglove
    1. Open Foxglove for visualisation `https://simphera-iac.dspace-dev.com/foxglove/`
    2. Click on *Open connection* and connect to the default address *ws://localhost:8765*
    3. Load layout from ApplCSH\SW\Foxglove\iac-layout-basic.json. CLick View->Import layout from file->Select json file
5. Open a second terminal, attach to the running sut_te_bridge container, build the solution and start the execution. In our example that means the following steps:
    1. Switch the sut_te_bridge image to the dev version in the docker compose and mount the directories including the required sources into the container
        1. Image name: `722180079256.dkr.ecr.eu-central-1.amazonaws.com/dspace/iac_sut_te_bridge_dev`
        2. volumes:
            - ./dSPACE-IAC-sut-te-bridge/ros2_bridge_ws:/root/ros2_bridge_ws
            - ./dSPACE-IAC-sut-te-bridge/runtime_scripts:/root/runtime_scripts
    2. `docker exec -it sut_te_bridge bash`
    3. `./sut-te-bridge-ros2build`
    4. `./sut-te-bridge-ros2run`
6. Open a third terminal and start your stack. In our example that includes attaching to the driving_stack container and executing the start script:
    1. `docker exec -it driving_stack bash`
    2. `./ros2build`
    3. `./ros2run`
7. To shut down the simulation, open another terminal and execute `docker compose -f docker-compose_sut_te_bridge_foxglove.yml down --remove-orphans`

### Contribute
In general the bridge is maintained by dSPACE, so if you find any missing features or bugs it would be great if you make use of the github issue feature to share them with us.
If you would like to package the simulation data in ROS publishers/subscribers, that are currently not supported e.g. CAN or dbw_raptor messages, the following section should provide some guidance how to easily achieve that.
In case that these additional interfaces might be useful to other teams, it would be great, if you could push your changes to a seperate branch and create a pull request, so that they become available for the rest of the community.
1. Add declaration of the publishers/subscribers to the [bridge.h](ros2_bridge_ws/src/sut_te_bridge/include/bridge.h) header
2. Initialize publishers similar to lines 127-162 in [bridge.cpp](ros2_bridge_ws/src/sut_te_bridge/src/bridge.cpp#L127-L162)
3. Initialize subscribers similar to lines 164-167 in [bridge.cpp](ros2_bridge_ws/src/sut_te_bridge/src/bridge.cpp#L164-L167)
4. Implement publisher function which accesses the data stored in this->CanBus object, creates ROS messages from it and publishes it using your publisher from step 2
5. Add call to the publisher function to the [publishSimulationState function](ros2_bridge_ws/src/sut_te_bridge/src/bridge.cpp#L363-L408)
6. Implement callback functions for your subscribers from step 3, which read the corresponding ROS messages and write the data to this->feedbackCmd object
