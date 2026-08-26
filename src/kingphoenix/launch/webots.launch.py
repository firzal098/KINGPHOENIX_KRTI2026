import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
        default_value='1.0',
        description='Horizontal field of view in radians (~60 degrees)'
    )

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

    set_stream_rate = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/mavros/set_stream_rate',
                    'mavros_msgs/srv/StreamRate',
                    '{stream_id: 10, message_rate: 50, on_off: true}',
                ],
                output='screen',
            )
        ]
    )

    # FIXED: Changed 'association_max_dist' from int (10) to float (10.0)
    mavros_estimator_node = Node(
        package='mavros_controller',
        executable='mavros_gate_estimator',
        name='mavros_gate_estimator',
        output='screen',
        parameters=[{
            'pnp_vision_sigma':      1.0,
            'drone_pose_sigma':      1.0,
            'gate_prior_sigma':      2.0,
            'association_max_dist':  10.0,
        }],
    )

    controller_node = Node(
        package='mavros_controller',
        executable='controller',
        name='controller',
        output='screen',
    )

    cuda_gate_inference_node = Node(
        package='cuda_gate_inference',
        executable='gate_perception',
        name='cuda_gate_inference',
        output='screen',
    )

    rosbridge_websocket_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'max_message_size': 10000000,
        }],
        arguments=['--ros-args', '--log-level', 'rosbridge_websocket:=WARN']
    )

    # Locate src/gcs directory dynamically
    gcs_candidates = [
        os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'gcs'),
        '/ros_ws/src/gcs',
        '/home/firza/krti2026_final/src/gcs',
    ]
    gcs_dir = next((d for d in gcs_candidates if os.path.exists(os.path.join(d, 'package.json'))), '/ros_ws/src/gcs')

    gcs_frontend_process = ExecuteProcess(
        cmd=['bash', '-c', 'if [ ! -d node_modules ]; then npm install; fi && npm run dev'],
        cwd=gcs_dir,
        output='screen',
    )

    return LaunchDescription([
        server_ip_arg,
        server_port_arg,
        frame_id_arg,
        fov_arg,
        camera_node,
        mavros_launch,
        set_message_interval,
        set_stream_rate,
        mavros_estimator_node,
        cuda_gate_inference_node,
        controller_node,
        rosbridge_websocket_node,
        gcs_frontend_process,
    ])