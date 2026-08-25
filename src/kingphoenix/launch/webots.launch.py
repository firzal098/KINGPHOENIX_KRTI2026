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
        default_value='1.0',
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
    # Delayed 15 s: gives MAVROS time to connect AND ArduPilot EKF time to initialise
    # before the service call is made (EKF can take 10-30 s after SITL boot).
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

    # Belt-and-suspenders: also enable ArduPilot stream 10 (EXTRA1) at 50 Hz via
    # mavros/set_stream_rate. LOCAL_POSITION_NED is part of this stream group and
    # this call works even if set_message_interval is not supported by the firmware.
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

    # Mavros estimator — fuses MAVROS state with gate detections for position estimation
    mavros_estimator_node = Node(
        package='mavros_controller',
        executable='mavros_gate_estimator',
        name='mavros_gate_estimator',
        output='screen',
        parameters=[{
            'pnp_vision_sigma':      2.0,
            'drone_pose_sigma':      1.0,
            'gate_prior_sigma':      2.0,
            'association_max_dist':  10,
        }],
    )

    # ArduPilot MAVROS Controller — FSM-based takeoff/hover/land state machine
    controller_node = Node(
        package='mavros_controller',
        executable='controller',
        name='controller',
        output='screen',
    )

    # CUDA gate inference — runs the TensorRT/ONNX gate keypoint detector on the GPU
    cuda_gate_inference_node = Node(
        package='cuda_gate_inference',
        executable='gate_perception',
        name='cuda_gate_inference',
        output='screen',
    )

    camera_front_optical_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_optical_tf',
        arguments=[
            '--x', '0.1', '--y', '0.0', '--z', '0.0',
            '--roll', '-1.5707963', '--pitch', '0.0', '--yaw', '-1.5707963',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_front_optical_frame'
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
        set_stream_rate,
        mavros_estimator_node,
        cuda_gate_inference_node,
        controller_node,
        # camera_front_optical_frame
    ])