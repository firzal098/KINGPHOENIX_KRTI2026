import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
        default_value='1.03',
        description='Horizontal field of view in radians (~60 degrees)'
    )

    camera_pitch_deg_arg = DeclareLaunchArgument(
        'camera_pitch_deg',
        default_value='10.0',
        description='Camera mounting pitch angle in degrees (e.g. 15.0 for real drone up-tilt, 0.0 for level sim)'
    )

    estimation_method_arg = DeclareLaunchArgument(
        'estimation_method',
        default_value='pixel_innovation',
        choices=['pixel_innovation', 'pnp'],
        description='Gate estimation method: pixel_innovation (direct image-space EKF) or pnp (legacy 3D pose PnP)'
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

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace='mavros',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('mavros'), 'launch', 'apm_pluginlists.yaml'),
            os.path.join(get_package_share_directory('mavros'), 'launch', 'apm_config.yaml'),
            {
                'use_sim_time':       True,
                'fcu_url':            'tcp://172.24.123.183:5760',
                'gcs_url':            'udp://@172.24.112.1:14550',
                'tgt_system':         1,
                'tgt_component':      1,
                'time.timesync_rate': 0.0,  # Disables timesync spam directly
            }
        ],
        arguments=[
            '--log-level', 'rcl.logging_rosout:=ERROR',
            '--log-level', 'mavros.param:=WARN',
            '--log-level', 'mavros.time:=ERROR',
        ]
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

    use_pixel_ekf = PythonExpression(["'", LaunchConfiguration('estimation_method'), "' == 'pixel_innovation'"])
    use_legacy_pnp = PythonExpression(["'", LaunchConfiguration('estimation_method'), "' == 'pnp'"])

    # 1. Pixel Innovation Pipeline Nodes (Default)
    gate_yolo_node = Node(
        package='cuda_gate_inference',
        executable='gate_yolo_node',
        name='gate_yolo_node',
        output='screen',
        condition=IfCondition(use_pixel_ekf),
        parameters=[{
            'conf_threshold': 0.50,
            'corner_conf_threshold': 0.15,
            'gate_width_m': 1.9,
            'gate_height_m': 2.0,
            'camera_pitch_deg': LaunchConfiguration('camera_pitch_deg'),
            'prior_source': 'defined_constants',
            'publish_pnp_fallback': True,
        }],
    )

    gate_estimator_node = Node(
        package='cuda_gate_inference',
        executable='gate_estimator_node',
        name='gate_estimator_node',
        output='screen',
        condition=IfCondition(use_pixel_ekf),
        parameters=[{
            'estimation_method': 'pixel_innovation',
            'gate_width_m': 1.9,
            'gate_height_m': 2.0,
            'sigma_pixel': 3.0,
            'process_noise_q': 1e-4,
            'p0_sigma': 2.0,
            'camera_pitch_deg': LaunchConfiguration('camera_pitch_deg'),
            'anchor_priors_at_takeoff': True,
            'cond_h_max': 1000.0,
            'max_innovation_px_sanity': 120.0,
            'max_refine_distance_m': 36.0,
            'v7': True,
        }],
    )

    # 2. Legacy PnP Pipeline Nodes (Optional fallback)
    mavros_estimator_node = Node(
        package='mavros_controller',
        executable='mavros_gate_estimator',
        name='mavros_gate_estimator',
        output='screen',
        condition=IfCondition(use_legacy_pnp),
        parameters=[{
            'pnp_vision_sigma':           2.0,
            'drone_pose_sigma':           1.0,
            'gate_prior_sigma':           4.0,
            'association_max_dist':       10.0,
            'max_refine_distance_m':      36.0,
            'blend_gate5_with_mean_1_2':  True,
            'tunnel_blend_gate1_and_2':   True,
            'use_1d_right_axis_offset':   True,
            'enable_gate3_pnp_refinement': True,
            'v7':                         True,
            'camera_pitch_deg':           LaunchConfiguration('camera_pitch_deg'),
        }],
    )

    cuda_gate_inference_node = Node(
        package='cuda_gate_inference',
        executable='gate_perception',
        name='cuda_gate_inference',
        output='screen',
        condition=IfCondition(use_legacy_pnp),
        parameters=[{
            'conf_threshold': 0.50,
            'corner_conf_threshold': 0.15,
        }],
    )

    controller_node = Node(
        package='mavros_controller',
        executable='controller',
        name='controller',
        output='screen',
        parameters=[{
            'triple_gate_pass_method': 3,
            #'max_accel': 5.6638,
            'max_accel': 8.5,
            'v7': True,
            'enable_gate_1_1': True,
            'max_action_magnitude': 16.0,
            'max_yaw_rate_deg': 360.0,
        }],
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
    #    arguments=['--ros-args', '--log-level', 'rosbridge_websocket:=WARN']
    )

    # Locate src/gcs directory dynamically
    gcs_candidates = [
        os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'gcs'),
        '/ros_ws/src/gcs',
        '/home/firza/krti2026_final/src/gcs',
    ]
    gcs_dir = next((d for d in gcs_candidates if os.path.exists(os.path.join(d, 'package.json'))), '/ros_ws/src/gcs')

    gcs_frontend_process = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['npm', 'run', 'dev'],
                cwd=gcs_dir,
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    # Cleanup handler to ensure rosbridge and lingering processes are terminated on launch shutdown
    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=['bash', '-c', 'pkill -9 -f rosbridge_websocket 2>/dev/null || true; fuser -k 5173/tcp 2>/dev/null || true'],
                    output='screen',
                )
            ]
        )
    )

    return LaunchDescription([
        server_ip_arg,
        server_port_arg,
        frame_id_arg,
        fov_arg,
        camera_pitch_deg_arg,
        estimation_method_arg,
        camera_node,
        mavros_node,
        set_message_interval,
        set_stream_rate,
        gate_yolo_node,
        gate_estimator_node,
        mavros_estimator_node,
        cuda_gate_inference_node,
        controller_node,
        rosbridge_websocket_node,
        gcs_frontend_process,
        shutdown_handler,
    ])