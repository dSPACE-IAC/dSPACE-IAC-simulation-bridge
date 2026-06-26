# Copyright 2020-2021, The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch file for IAC vehicle."""

import os
import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler)

from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.conditions import IfCondition


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def read_track_name_from_params(param_file, default_track='ims'):
    try:
        with open(param_file, 'r', encoding='utf-8') as yaml_file:
            for line in yaml_file:
                stripped = line.strip()
                if stripped.startswith('track_name:'):
                    value = stripped.split(':', 1)[1].strip().strip('"').strip("'")
                    if value:
                        return value
    except OSError:
        pass
    return default_track


def generate_launch_description():
    """Launch all packages for the vehicle in IAC."""

    node_arguments = []
    include_arguments = []

    car_ip = 'x.x.x.x' # Define sim host IP here or with ip:=x.x.x.x argument when launching
    car_port = 'xxxx' # Define sim host port here or with port:=xxxx argument when launching
    bs_ip = 'x.x.x.x' # Define sim host IP here or with ip:=x.x.x.x argument when launching
    bs_port = 'xxxx' # Define sim host port here or with port:=xxxx argument when launching
    namespace = ''
    for arg in sys.argv:
        if arg.startswith("car_ip:="):
            socket_ip = str(arg.split(":=")[1])
        elif arg.startswith("car_port:="):
            socket_port = str(arg.split(":=")[1])
        elif arg.startswith("ns:="):
            namespace = str(arg.split(":=")[1])

    src_base_param_file = '/root/ros_ws/src/npc_controller/config/base.param.yaml'
    share_base_param_file = get_share_file(
        package_name='npc_controller', file_name='config/base.param.yaml'
    )
    npc_controller_param_file = os.environ.get('NPC_CONTROLLER_PARAMS_FILE', src_base_param_file)
    if not os.path.exists(npc_controller_param_file):
        npc_controller_param_file = share_base_param_file

    configured_track_name = read_track_name_from_params(npc_controller_param_file)
    track_name_variants = [configured_track_name]
    if '_' in configured_track_name:
        track_name_variants.append(configured_track_name.replace('_', '-'))
    if '-' in configured_track_name:
        track_name_variants.append(configured_track_name.replace('-', '_'))

    src_track_param_file = ''
    share_track_param_file = ''
    for track_name in track_name_variants:
        src_candidate = f'/root/ros_ws/src/npc_controller/config/{track_name}.param.yaml'
        if os.path.exists(src_candidate):
            src_track_param_file = src_candidate
            break

    if not src_track_param_file:
        for track_name in track_name_variants:
            share_candidate = get_share_file(
                package_name='npc_controller', file_name=f'config/{track_name}.param.yaml'
            )
            if os.path.exists(share_candidate):
                share_track_param_file = share_candidate
                break
    npc_controller_track_param_file = os.environ.get('NPC_CONTROLLER_TRACK_PARAMS_FILE', '')
    if not npc_controller_track_param_file:
        if os.path.exists(src_track_param_file):
            npc_controller_track_param_file = src_track_param_file
        elif share_track_param_file:
            npc_controller_track_param_file = share_track_param_file

    npc_controller_param = DeclareLaunchArgument(
        'npc_controller_param_file',
        default_value=npc_controller_param_file,
        description='Path to param file for npc controller (can be overridden via container mounts)'
    )

    parameter_files = [LaunchConfiguration('npc_controller_param_file')]
    if npc_controller_track_param_file and npc_controller_track_param_file != npc_controller_param_file:
        parameter_files.append(npc_controller_track_param_file)

    npc_controller_node = Node(
        package='npc_controller',
        executable='controller_exec',
        output='screen',
        remappings=[
            ('bestpos', 'novatel_top/bestpos'),
            ('wheel_speed_report', 'raptor_dbw_interface/wheel_speed_report'),
            ('pt_report', 'raptor_dbw_interface/pt_report'),
            ('steering_cmd', 'raptor_dbw_interface/steering_cmd'),
            ('accelerator_pedal_cmd', 'raptor_dbw_interface/accelerator_pedal_cmd'),
            ('brake_cmd', 'raptor_dbw_interface/brake_cmd'),
            ('gear_cmd', 'raptor_dbw_interface/gear_cmd'),
        ],
        parameters=parameter_files
    )

    include_arguments.append(npc_controller_param)
    node_arguments.append(npc_controller_node)
    
    # node_arguments.append(can_parser_node)
    clock_setting='false'
    if "SIM_CLOCK_MODE" in os.environ:
        clock_setting = os.environ['SIM_CLOCK_MODE']
    
    return LaunchDescription([SetParameter(name='use_sim_time', value=clock_setting)] + include_arguments+node_arguments)
