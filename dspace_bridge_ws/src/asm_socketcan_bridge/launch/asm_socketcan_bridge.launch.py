from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_path = PathJoinSubstitution(
        [FindPackageShare('asm_socketcan_bridge'), 'config', 'asm_socketcan_bridge.yaml']
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_path,
        description='Path to the YAML file with asm_socketcan_bridge parameters.',
    )

    bridge_node = Node(
        package='asm_socketcan_bridge',
        executable='AsmSocketCanBridgeNode',
        name='asm_socketcan_bridge_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
    )

    return LaunchDescription([
        params_file_arg,
        bridge_node,
    ])
