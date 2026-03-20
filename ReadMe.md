# dSPACE Indy Autonomous Challenge Simulator

This repository contains the current state of the different bridge implementations for the dSPACE IAC simulation environment.
Each bridge lives in its own ROS2 package inside `dspace_bridge_ws/src`.
The asm_socketcan_bridge enables the data exchange between the dSPACE car and environment model simulated using the Automotive Simulation Models (ASM) and the SUT of the IAC team.
The asm_ros2_bridge is deprecated and has been removed from this repository. It will not receive updates anymore from dSPACE.
The socketcan package provides per-message timers, configurable publish rates and a dedicated CAN decoding layer that mirrors the race car CAN network.
The aurelion_ros2_bridge communicates sensor data (camera, lidar, radar) between the dSPACE AURELION sensor simulation and the IAC team SUT.
The simulation data is published as ROS2 messages.
The implementation of all bridges is in form of ROS2 nodes.
There is also a foxglove bridge, which is used to make the messages published in ROS2 observable in Foxglove or Lichtblick.

For further information please check the latest simulation package version.
This repository is meant to provide further insights into the implementation and enables you to adapt the basic bridge implementations to the requirements of your SUT, in order to make full use of the dSPACE Indy Autonomous Challenge simulator.

## Prerequisites
You need to have the following tools installed:
- Docker (e.g. Docker Desktop)
- Docker Compose (already installed if you use Docker Desktop)

To run the complete simulation setup, you need to have access to the following instances:
- dSPACE IAC license server
- dSPACE AWS docker registry

For asm_socketcan_bridge:
- socketcan kernel module (not available in default wsl ubuntu images -> use native install or VM)
- can-utils (optional for debugging)

## Structure of this repository
The following section provides an overview of the repository content, in order to speed up the process of finding what you are looking for and provide an understanding, where to add things when contributing.

### Dockerfile
This Dockerfile is used to create all bridge versions (asm_socketcan, aurelion_ros2 and foxglove).
The images are differentiated by selecting the target of the docker build command (asm_socketcan_bridge, aurelion_ros2_bridge, ros2_foxglove_bridge).
There is also a dev version (dspace_bridge_dev), which only contains the dependencies required by the bridge versions (e.g. ROS2, custom message definitions, socketcan packages etc) and a neutral entrypoint, so that could be used to quickly test new developments on the bridge.
The standard version contains the respective compiled bridge node and an entrypoint for automatic startup of that node.
The standard version is also recommended to be used for local testing, when working on the SUT code to check whether the system is capable to run with automatic startup procedure used for headless testing in the cloud.
The foxglove version contains the foxglove-bridge application and an entrypoint to start the corresponding node. This can be connected to Lichtblick as well.

### build_dspace_bridge.sh
This script could be used to quickly build all versions of the dSPACE bridges.

### docker_compose_example.yml
This compose file bundles VEOS, V-ESI, CTUN, the asm_socketcan bridge and Foxglove for local execution.
There is also the uva_example_driving_stack added to the compose, which can be used for initial commissioning and should be replaced by your stack (note: the uva example stack is only working on IMS).
Use it as a template for your own setup by adjusting image tags, license settings and mounted configuration files.
To easily adapt the configuration of the asm_socketcan_bridge node, the example includes a (commented out) mount of `asm_socketcan_bridge_override.yaml` into the container.
This enables quick configuration without rebuilding the image.

### runtime_scripts
This directory contains some scripts, which should make the work with the dev version of the bridges easier.
It also contains the entrypoint.sh script which is used to startup all bridge types.

### ros_ws_aux
This auxilary ros workspace contains all ros packages, that are required by the several bridge versions.
Currently this contains all custom message definitions used in the system.
If you want to make your custom messages available in foxglove, the easiest way would be to add your definitions to this auxilary workspace and rebuild the foxglove bridge.
There is no need to add your custom messages to the repository, if they should only be available in foxglove.
If they should also be used by the official bridge please push them to a seperate branch and create a pull request.

### dspace_bridge_ws
This ros workspace contains the source code of the different bridge versions in seperate packages.
Usually there is only one bridge type per Docker image, so the respective package is copied into the image during build.
In case of the dev image, the suggested approach is to mount the dspace_bridge_ws directory into the running container.
This should be your starting point, if you want to understand how the connection between your stack and the simulator is implemented and how the interface is designed.
The `src` directory is split into the ROS2 packages `asm_socketcan_bridge` and `aurelion_ros2_bridge`.
The asm_socketcan_bridge package contains the full socketcan implementation:
- `config/asm_socketcan_bridge.yaml` holds all runtime parameters. Every publish function has a dedicated timer interval parameter, so you can throttle or burst individual CAN messages by editing this file and rebuilding the image or by mounting an override.
- `config/CAN1-INDY-V23.dbc` provides the CAN signal definitions that feed the decoder lookup tables. Extend or exchange them if your car uses a different CAN layout. After modifying the DBC, run `config/generate_dbc_c_code.py` to regenerate the C++ utility code.
- `launch/asm_socketcan_bridge.launch.py` binds parameters, brings up the multi threaded executor and wires the timers to the publishers.
Mount the `asm_socketcan_bridge_override.yaml` from the repository root into your container whenever you want to apply custom parameters without touching the default file in the install space.

### dspace_bridge_logs
Runtime log files written by the bridge containers are stored here by default.
Clean the directory regularly when iterating locally to keep disk usage in check.

### demo_stack_uva
Implementation of the demo controller working with the CAN bridge.

### ASM_Maneuver.py
Script that can be executed in the VEOS container to send flags manually. In the container the script is located in `/home/dspace/scripts`.


## How to
The following instructions assume an execution in a Linux environment, either on a Linux host system or in WSL.
This means that all given commands and scripts are written for Linux.
However the execution also works for Windows and Mac, you just need to adapt the commands and scripts slightly for your preferred OS.

### Execute
Example workflow for asm_socketcan_bridge including Foxglove:
1. Start Docker Desktop
2. Navigate into the folder, where the Docker compose is located.
3. Copy `docker-compose_example.yml` and rename to `docker-compose.yml`.
4. Adjust parameters in `docker-compose.yml` to match your registry tags and license server. Update the `dspace_bridge` service to point to the bridge variant you want to launch (socketcan, ros2, aurelion or dev).
5. Provide your custom bridge parameters by editing `asm_socketcan_bridge_override.yaml` and removing the comment in the volume mount for the bridge. E.g. adjust the `publish_intervals.*` values whenever you want to slow down or speed up individual CAN and ROS2 message publishers.
6. Open a terminal and execute `docker compose up`.
7. Start Lichtblick
    1. Open Lichtblick for visualisation either the local container `localhost:8080` or from the Lichtblick suite `https://lichtblick-suite.github.io/lichtblick/`
    2. Click on *Open connection* and connect to the default address *ws://localhost:8765*
    3. Load layout by clicking View->Import layout from file->Select json file (example layout can be found under `foxglove_bridge/iac-layout-basic.json`)
8. Attach to the running bridge container if you want to iterate on the code:
    1. Switch the image to `dspace/iac_dspace_bridge_dev` in the compose file and mount `./dspace_bridge_ws` into `/root/dspace_bridge_ws`
    2. `docker exec -it dspace_bridge_dev bash`
    3. `./dspace_bridge_build`
    4. `export BRIDGE_TYPE="ASM_CAN"`
    5. `./entrypoint.sh`
9. To shut down the simulation, open another terminal and execute `docker compose down --remove-orphans`

### Iterate
To create an updated simulator image after touching the bridge sources, use `build_dspace_bridge.sh`.
The script can build the dev image as well as the asm_socketcan, aurelion and foxglove/Lichtblick variants.
After succesful build, only update the tags of the bridge images inside your custom `docker-compose.yml` to the current date and restart the compose stack.

### Contribute
In general the bridge is maintained by dSPACE, so if you find any missing features or bugs it would be great if you make use of the Github issue feature to share them with us.
Also feel free to directly implement missing features by simply creating your own branch or fork of the main repository.
In case that your enhancements might be useful to other teams, it would be great, if you could create a pull request, so that they become available for the rest of the community.
