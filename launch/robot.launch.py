from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Arguments
    enable_motors_arg = DeclareLaunchArgument(
        'enable_motors',
        default_value='true',
        description='Enable motor driver'
    )

    enable_rl_balancer_arg = DeclareLaunchArgument(
        'enable_rl_balancer',
        default_value='false',
        description='Enable RL balancer node'
    )

    rl_model_path_arg = DeclareLaunchArgument(
        'rl_model_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('rl_balancer_node'),
            'policy',
            # 'balancing-robot_model_1000.onnx'
            # 'balancing-robot_model_620.onnx'
            'balancing-robot_model_24_03_2026.onnx'
        ]),
        description='Path to the ONNX policy model file'
    )

    # Launch Configurations
    enable_motors = LaunchConfiguration('enable_motors')
    enable_rl_balancer = LaunchConfiguration('enable_rl_balancer')
    rl_model_path = LaunchConfiguration('rl_model_path')

    # ── Separate processes ──

    tf_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robot_tf_publisher'),
            '/launch/tf_publisher.launch.py'
        ])
    )

    ekf_config = PathJoinSubstitution([
        FindPackageShare('robot_launch'),
        'config',
        'ekf.yaml'
    ])

    tmini_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('tmini_driver_node'),
            '/launch/tmini.launch.py'
        ]),
    )

    foxglove_bridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
             FindPackageShare('foxglove_bridge'),
             '/launch/foxglove_bridge_launch.xml'
        ])
    )

    # ── Composed motion container (single process, multi-threaded, zero-copy) ──

    intra = {'use_intra_process_comms': True}

    motion_container = ComposableNodeContainer(
        name='motion_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='imu_driver',
                plugin='ImuDriverNode',
                name='imu_driver',
                parameters=[{
                    'frame_id': 'imu',
                    'ekf_rate': 100.0,
                    'linear_acceleration_stddev': 0.01,
                    'angular_velocity_stddev': 0.001,
                    'orientation_stddev': 0.001,
                }],
                extra_arguments=[intra],
            ),
            ComposableNode(
                package='robot_launch',
                plugin='FootprintPublisher',
                name='footprint_publisher',
                parameters=[{
                    'odom_frame': 'odom',
                    'base_link_frame': 'base_link',
                    'footprint_frame': 'base_footprint',
                    'publish_rate': 30.0,
                }],
                extra_arguments=[intra],
            ),
        ],
        output='screen',
    )

    load_motor_nodes = LoadComposableNodes(
        target_container='motion_container',
        condition=IfCondition(enable_motors),
        composable_node_descriptions=[
            ComposableNode(
                package='ddsm115_driver',
                plugin='DDSM115Node',
                name='ddsm115_driver_node',
                extra_arguments=[intra],
            ),
            ComposableNode(
                package='odometry_node',
                plugin='OdometryNode',
                name='odometry_node',
                parameters=[{'publish_tf': True}],
                extra_arguments=[intra],
            ),
        ],
    )

    load_rl_node = LoadComposableNodes(
        target_container='motion_container',
        condition=IfCondition(enable_rl_balancer),
        composable_node_descriptions=[
            ComposableNode(
                package='rl_balancer_node',
                plugin='RLBalancerNode',
                name='rl_balancer_node',
                parameters=[{'model_path': rl_model_path}],
                extra_arguments=[intra],
            ),
        ],
    )

    return LaunchDescription([
        enable_motors_arg,
        enable_rl_balancer_arg,
        rl_model_path_arg,
        tf_publisher_launch,
        # ekf_node,
        tmini_launch,
        motion_container,
        load_motor_nodes,
        load_rl_node,
        foxglove_bridge_launch,
    ])
