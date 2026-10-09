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
6. Set `use_sim_time: true` in the YAML files of the bridge (`asm_socketcan_bridge_override.yaml`), the demo controller (`demo_stack_uva/base.param_override.yaml`) and the Raptor DBW node (`raptor_dbw_override.yaml`) when deterministic simulation stepping is required. These YAML files are the only source of this setting; there is no environment variable override, so all three must agree.
7. Open a terminal and execute `docker compose up`.
8. Start Lichtblick
    1. Open Lichtblick for visualisation either the local container `localhost:8080` or from the Lichtblick suite `https://lichtblick-suite.github.io/lichtblick/`
    2. Click on *Open connection* and connect to the default address *ws://localhost:8765*
    3. Load layout by clicking View->Import layout from file->Select json file (example layout can be found under `foxglove_bridge/iac-layout-basic.json`)
9. Attach to the running bridge container if you want to iterate on the code:
    1. Switch the image to `dspace/iac_dspace_bridge_dev` in the compose file and mount `./dspace_bridge_ws` into `/root/dspace_bridge_ws`
    2. `docker exec -it dspace_bridge_dev bash`
    3. `./dspace_bridge_build`
    4. `export BRIDGE_TYPE="ASM_CAN"`
    5. `./entrypoint.sh`
10. To shut down the simulation, open another terminal and execute `docker compose down --remove-orphans`

### Simulation-time setup
For simulation-time mode, set `use_sim_time: true` in the bridge, stack and (when that path is selected) `raptor_dbw` YAML files, see step 6 above. Set `DS_CUSTOM_DATA_NUM_SAMPLES=1`. Without this value, the first `requestCustomData()` call can block inside the 1 ms V-ESI sub-step loop.

The bridge emits a `SIM_OBS ... summary=1` line at shutdown in sim mode; sampled per-second `SIM_OBS` counters are logged only when `logging.sim_observability: true`. The `demo_stack_uva/ims.param_override.yaml` override exposes `connection.useRaptorDbwNode` so direct CAN and Raptor DBW runs can be selected without rebuilding the controller image.

### Run modes and simulation time
The `asm_socketcan_bridge` supports two run modes:

| Mode | Selection | Time base | Step progression |
|------|-----------|-----------|------------------|
| Real-time | `use_sim_time: false` (default) | Wall clock, no `/clock` | Free-running bridge; publisher timers run on wall time. |
| Sim time | `use_sim_time: true` | `/clock` published by the bridge | The bridge steps VEOS/V-ESI and waits for the stack's commands. |

`use_sim_time` is set in the bridge YAML.

#### How to set up your team stack (sim time)
The stack needs no simulator-specific code, topics or parameters:

- It might be required to add a `use_sim_time=true` parameter to every stack node (take a look at the example stack, set it in `demo_stack_uva/base.param_override.yaml`). The bridge publishes standard `/clock` (reliable, transient local). Use ROS-time timers, as `steady_clock`, `system_clock` and wall timers do not follow `/clock`, and a ROS-time timer fires at most once per `/clock` jump.
- The stack reads the sensor topics and CAN report frames it uses on the car and sends its usual DBW commands. The bridge observes the command frames on `can0`, so all of these variants work:
  - **V1:** the stack writes CAN frames directly.
  - **V2:** a team DBW node writes CAN frames.
  - **V3:** the stack publishes Raptor command topics and the `raptor_dbw` default node from this repository writes the CAN frames (set `use_sim_time` as well in `raptor_dbw_override.yaml`).
- Every command ID the stack sends per cycle must be a scheduled ID (see adaptation below). A frame with an unscheduled ID that arrives after its step closed is dropped.
- The bridge emits a marker frame `0x7FF` (uint64 step counter) before each `/clock` for capture tooling, which might be used by your stack for synchronization (it will also work when you ignore this). Set `sim.step_marker.enabled: false` to disable this behavior.

For each step the bridge latches one command snapshot, runs the number (given by `sim.step_size_ms`) of 1 ms V-ESI sub-steps with that snapshot, publishes all due outputs, the marker and finally `/clock`, then waits until every due command ID has a fresh frame or a wall timeout expires. On timeout it logs a warning and continues; missing IDs keep their last accepted value. Simulation time is independent of the wall time spent, so a run may be faster or slower than real time.

#### Environment outputs and their intervals
All environment outputs use `publish_intervals.*_ms` in `asm_socketcan_bridge.yaml` (or your override file) in every run mode. Valid values are `1` to `1000` (publication interval in ms) and `0` (output disabled: no timer, no message construction). Other values are rejected with a warning and replaced by 10 ms. The shipped values are examples which you should adjust to what your stack needs. It is recommended to disable (set to `0`) any outputs that your stack does not use to improve simulation performance.

In sim mode each enabled output is published from the step scheduler with a drift-free rule: at the first step with `t >= next_due`, then `next_due += interval`. All outputs are built from the final ASM data of the step and stamped with the step time.

#### Adaptation period
With `adaptation.mode: on` (default) the bridge learns the stack's command schedule at the start of every run, while the car is usually stationary or leaving the pit lane:

1. For the first `adaptation.duration_s` (default 3.0) simulated seconds, time advances in 1 ms steps, wall-paced at `adaptation.realtime_factor` (default 1.0). Nothing is awaited.
2. Per command ID the bridge measures period, phase and reaction latency. An ID seen fewer than three times or with irregular spacing is unscheduled: applied when present, never awaited.
3. Step duration = GCD of the scheduled command periods and the enabled output intervals, clamped to 1-100 ms. The bridge then switches to the normal protocol at the next multiple of the step.

The detected schedule is written to `adaptation.txt` in `logging.path` for inspection. Currently there is no mechanism to read that back, so every run adapts from zero. If you use adaptation mode, make sure, that all stack commands are sent at their regular intervals during the adaptation period in order to get deterministic results. Adjust the `adaptation.duration_s` if needed. 
Alternatively use `adaptation.mode: off` to provide a static schedule instead (see `sim.static.*` parameters below). 

#### Parameters
| Parameter | Default | Meaning |
|-----------|---------|---------|
| `sim.step_marker.enabled` | `true` | Emit marker `0x7FF` before each `/clock`. |
| `sim.readiness.min_clock_subscribers` | `1` | Matched `/clock` subscriptions (excluding the bridge) required before stepping. |
| `sim.readiness.settle_ms` | `1000` | Wall settle time after the subscriber condition holds. |
| `adaptation.mode` | `on` | `on` or `off`. |
| `adaptation.duration_s` | `3.0` | Adaptation window in simulated seconds. |
| `adaptation.realtime_factor` | `1.0` | Wall pacing during adaptation only. |
| `sim.timeout_ms` | `20` | Wall-time limit in ms (1-10000) to wait for the stack's commands, counted from the `/clock` release. Applies to all steps after adaptation. |
| `sim.static.step_ms` | `10` | Feedback cycle time in ms with `adaptation.mode: off`. |
| `sim.static.required_command_ids` | `[1400, 1401, 1402, 1403, 1404]` | Awaited CAN IDs with `adaptation.mode: off`. |
| `sim.static.command_periods_ms` | `[10, 10, 10, 10, 500]` | Period per ID above. |
| `sim.static.command_first_due_ms` | `[10, 10, 10, 10, 510]` | Simulation time of the first due step per ID. |
| `logging.real_time_factor` | `false` | Log real-time factor every 30 simulated seconds and in the summary. |
| `logging.sim_observability` | `false` | Log the `SIM_OBS` step and timeout counters once per second (debugging). The shutdown summary is always logged. |
| `logging.sim_steps` | `false` | Write `sim_steps.csv` (step records) and `sim_commands.csv` (per-step commands) to `logging.path`. |
| `sim.replay_file` | `""` | Open-loop replay of a `sim_commands.csv`; needs no stack. |

### Iterate
To create an updated simulator image after touching the bridge sources, use `build_dspace_bridge.sh`.
The script can build the dev image as well as the asm_socketcan, aurelion and foxglove/Lichtblick variants.
After succesful build, only update the tags of the bridge images inside your custom `docker-compose.yml` to the current date and restart the compose stack.

### Contribute
In general the bridge is maintained by dSPACE, so if you find any missing features or bugs it would be great if you make use of the Github issue feature to share them with us.
Also feel free to directly implement missing features by simply creating your own branch or fork of the main repository.
In case that your enhancements might be useful to other teams, it would be great, if you could create a pull request, so that they become available for the rest of the community.
