from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    return LaunchDescription([
        server_ip_arg,
        server_port_arg,
        frame_id_arg,
        fov_arg,
        camera_node,
    ])