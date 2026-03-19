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


def generate_launch_description():
    """Launch all packages for the vehicle in IAC."""

    node_arguments = []
    include_arguments = []

    car_ip = 'x.x.x.x' # Define sim host IP here or with ip:=x.x.x.x argument when launching
    car_port = 'xxxx' # Define sim host port here or with port:=xxxx argument when launching
    bs_ip = 'x.x.x.x' # Define sim host IP here or with ip:=x.x.x.x argument when launching
    bs_port = 'xxxx' # Define sim host port here or with port:=xxxx argument when launching
    namespace = ''
    track_name = ''
    for arg in sys.argv:
        if arg.startswith("car_ip:="):
            socket_ip = str(arg.split(":=")[1])
        elif arg.startswith("car_port:="):
            socket_port = str(arg.split(":=")[1])
        elif arg.startswith("ns:="):
            namespace = str(arg.split(":=")[1])

    npc_controller_param_file = get_share_file(
        package_name='npc_controller', file_name='config/ims.param.yaml'
    )

    npc_controller_param = DeclareLaunchArgument(
        'npc_controller_param_file',
        default_value=npc_controller_param_file,
        description='Path to param file for lap state machine'
    )

    # TODO: For the time being, these are the relevant raptor topics for the NPC controller.
    #       For every team's system, the full set of topics should be output.
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
        parameters=[LaunchConfiguration('npc_controller_param_file')]
    )

    include_arguments.append(npc_controller_param)
    node_arguments.append(npc_controller_node)


    # AUTONOMA SIMULATOR BRIDGE
    # TODO: If not using CAN Frame outputs, remove these nodes and make sure all relevant raptor/novatel/vectornav topics are published directly

    # RAPTOR DRIVER
    # raptor_dbc = get_share_file(
    #     package_name='raptor_dbw_can', file_name='launch/CAN1-INDY-V12.dbc'
    # )

    # if namespace == '':
    #     raptor_ns = 'raptor_dbw_interface'
    # else:
    #     raptor_ns = namespace + '/raptor_dbw_interface'

    # raptor_node = Node(
    #     package='raptor_dbw_can',
    #     executable='raptor_dbw_can_node',
    #     output='screen',
    #     namespace= raptor_ns,
    #     parameters=[
    #         {'dbw_dbc_file': raptor_dbc}
    #     ],
    #     remappings=[('can_rx', 'from_can_bus'),
    #                 ('can_tx', 'to_can_bus')]
    # )

    # node_arguments.append(raptor_node)


    # TODO: This node is specific to Autoverse Simulator and should be removed and replaced with another relevant bridge.
    # autoverse_ros2_bridge_node = Node(
    #     package='autoverse_ros2_bridge',
    #     executable='autoverse_ros2_bridge_node',
    #     namespace=namespace,
    #     output='screen',
    #     arguments=['--ip', socket_ip, '--port', socket_port, '--ros-args', '--log-level', 'info'],
    #     remappings=[
    #         ('/novatel_top/bestpos','novatel_top/bestpos'),
    #         ('/novatel_bottom/bestpos','novatel_bottom/bestpos'),
    #         ('/novatel_top/bestvel','novatel_top/bestvel'),
    #         ('/novatel_bottom/bestvel','novatel_bottom/bestvel'),
    #         ('/novatel_top/rawimu','novatel_top/rawimu'),
    #         ('/novatel_bottom/rawimu','novatel_bottom/rawimu'),
    #         ('/novatel_top/bestgnsspos','novatel_top/bestgnsspos'),
    #         ('/novatel_bottom/bestgnsspos','novatel_bottom/bestgnsspos'),
    #         ('/novatel_top/bestgnssvel','novatel_top/bestgnssvel'),
    #         ('/novatel_bottom/bestgnssvel','novatel_bottom/bestgnssvel'),
    #         ('/novatel_top/heading2','novatel_top/heading2'),
    #         ('/novatel_bottom/heading2','novatel_bottom/heading2'),
    #         ('/novatel_top/inspva','novatel_top/inspva'),
    #         ('/novatel_bottom/inspva','novatel_bottom/inspva'),
    #         ('/novatel_top/rawimux','novatel_top/rawimux'),
    #         ('/novatel_bottom/rawimux','novatel_bottom/rawimux'),
    #         ('/vectornav/raw/common','vectornav/raw/common')
    #     ],
    #     parameters=[
    #         {"lon_stdev": 0.01},
    #         {"lat_stdev": 0.01},
    #         {"hgt_stdev": 0.01},
    #         {"heading_stdev": 2.13},
    #         {"linear_acceleration_covariance": 0.0009},
    #         {"angular_velocity_covariance": 0.00035}
    #     ]
    # )

    # node_arguments.append(autoverse_ros2_bridge_node)

    # dbc_file_path = get_share_file(
    #     package_name='can_parser', file_name='dbc/CAN1-INDY-V12.dbc'
    # )

    # can_parser_node = Node(
    #     package='can_parser',
    #     executable='can_parser_node',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[
    #         {'dbc_file': dbc_file_path}
    #     ],
    #     remappings=[
    #         ('powertrain_data', 'autoverse/powertrain_data'),
    #         ('race_control', 'autoverse/race_control'),
    #         ('vehicle_data', 'autoverse/vehicle_data'),
    #         ('vehicle_inputs', 'autoverse/vehicle_inputs'),
    #         ('ct_state', 'autoverse/ct_state'),
    #         ('can_tx', 'raptor_dbw_interface/from_can_bus'),
    #         ('can_rx', 'raptor_dbw_interface/to_can_bus'),
    #     ],
    # )
    
    # node_arguments.append(can_parser_node)
    clock_setting='false'
    if "SIM_CLOCK_MODE" in os.environ:
        clock_setting = os.environ['SIM_CLOCK_MODE']
    
    return LaunchDescription([SetParameter(name='use_sim_time', value=clock_setting)] + include_arguments+node_arguments)
