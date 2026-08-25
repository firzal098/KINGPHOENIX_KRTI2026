import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare configurable parameters with defaults
    server_ip_arg = DeclareLaunchArgument(
        'server_ip',
        default_value='172.24.112.1',
        description='IP address of the Webots host machine'
    )

    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='5599',
        description='UDP port streaming the camera images'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_front_optical_frame',
        description='Optical frame ID for the front camera'
    )

    fov_arg = DeclareLaunchArgument(
        'fov',
        default_value='1.0472',
        description='Horizontal field of view in radians (~60 degrees)'
    )

    # Directly run the camera_publisher node from webots_camera_front
    camera_node = Node(
        package='webots_camera_front',
        executable='camera_publisher',
        name='webots_camera_bridge',
        output='screen',
        parameters=[{
            'server_ip':   LaunchConfiguration('server_ip'),
            'server_port': LaunchConfiguration('server_port'),
            'frame_id':    LaunchConfiguration('frame_id'),
            'fov':         LaunchConfiguration('fov'),
        }]
    )

    # # Foxglove Bridge — allows connecting Foxglove Studio to this ROS instance
    # foxglove_bridge_launch = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('foxglove_bridge'),
    #             'launch',
    #             'foxglove_bridge_launch.xml'
    #         )
    #     )
    # )

    # MAVROS — connects to ArduPilot/PX4 SITL via TCP and forwards GCS via UDP
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mavros'),
                'launch',
                'apm.launch'
            )
        ),
        launch_arguments={
            'fcu_url':       'tcp://172.24.123.183:5760',
            'gcs_url':       'udp://@172.24.112.1:14550',
            'tgt_system':    '1',
            'tgt_component': '1',
        }.items()
    )

    # Set MAVLink message interval for LOCAL_POSITION_NED (msg ID 32) at 50 Hz.
    # Delayed 5 s to give MAVROS time to fully connect to the FCU before calling.
    set_message_interval = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/mavros/set_message_interval',
                    'mavros_msgs/srv/MessageInterval',
                    '{message_id: 32, message_rate: 50.0}',
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        server_ip_arg,
        server_port_arg,
        frame_id_arg,
        fov_arg,
        camera_node,
        # foxglove_bridge_launch,
        mavros_launch,
        set_message_interval,
    ])