from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    target_ip_arg = DeclareLaunchArgument(
        'target_ip',
        default_value='127.0.0.1',
        description='IP address of the Webots simulation machine'
    )

    target_port_arg = DeclareLaunchArgument(
        'target_port',
        default_value='5504',
        description='UDP port for Webots gripper control'
    )

    initial_state_arg = DeclareLaunchArgument(
        'initial_state',
        default_value='false',
        description='Initial boolean state of the gripper (true: open, false: closed)'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Periodic publication rate in Hz for /gripper/state'
    )

    repeat_count_arg = DeclareLaunchArgument(
        'repeat_count',
        default_value='1',
        description='Number of UDP packets sent on state changes'
    )

    gripper_node = Node(
        package='gripper',
        executable='webots_gripper',
        name='webots_gripper',
        output='screen',
        parameters=[{
            'target_ip': LaunchConfiguration('target_ip'),
            'target_port': LaunchConfiguration('target_port'),
            'initial_state': LaunchConfiguration('initial_state'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'repeat_count': LaunchConfiguration('repeat_count'),
        }]
    )

    return LaunchDescription([
        target_ip_arg,
        target_port_arg,
        initial_state_arg,
        publish_rate_arg,
        repeat_count_arg,
        gripper_node,
    ])
