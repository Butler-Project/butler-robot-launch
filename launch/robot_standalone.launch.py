from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
            'balancing-robot_model_620.onnx'
        ]),
        description='Path to the ONNX policy model file'
    )

    # Launch Configurations
    enable_motors = LaunchConfiguration('enable_motors')
    enable_rl_balancer = LaunchConfiguration('enable_rl_balancer')
    rl_model_path = LaunchConfiguration('rl_model_path')

    # TF Publisher
    tf_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robot_tf_publisher'),
            '/launch/tf_publisher.launch.py'
        ])
    )

    # Robot Localization
    ekf_config = PathJoinSubstitution([
        FindPackageShare('robot_launch'),
        'config',
        'ekf.yaml'
    ])

    # Footprint Publisher
    footprint_publisher_node = Node(
        package='robot_launch',
        executable='footprint_publisher',
        name='footprint_publisher',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'footprint_frame': 'base_footprint',
            'publish_rate': 30.0
        }]
    )

    # IMU Driver
    imu_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('imu_driver'),
            '/launch/imu.launch.py'
        ]),
        launch_arguments={
            'frame_id': 'imu',
            'linear_acceleration_stddev': '0.01',
            'angular_velocity_stddev': '0.001',
            'orientation_stddev': '0.001'
        }.items()
    )

    # YDLidar Driver
    # Tmini Lidar Driver
    tmini_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('tmini_driver_node'),
            '/launch/tmini.launch.py'
        ]),
    )

    # DDSM115 Motor Driver (Conditional)
    ddsm115_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ddsm115_driver'),
            '/launch/ddsm115.launch.py'
        ]),
        condition=IfCondition(enable_motors)
    )

    # Odometry Node (Conditional)
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('odometry_node'),
            '/launch/odometry.launch.py'
        ]),
        condition=IfCondition(enable_motors),
        launch_arguments={'publish_tf': 'true'}.items()
    )

    # RL Balancer Node (Conditional)
    rl_balancer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rl_balancer_node'),
            '/launch/rl_balancer.launch.py'
        ]),
        condition=IfCondition(enable_rl_balancer),
        launch_arguments={'model_path': rl_model_path}.items()
    )

    # Foxglove Bridge
    foxglove_bridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
             FindPackageShare('foxglove_bridge'),
             '/launch/foxglove_bridge_launch.xml'
        ])
    )

    return LaunchDescription([
        enable_motors_arg,
        enable_rl_balancer_arg,
        rl_model_path_arg,
        tf_publisher_launch,
        # ekf_node,
        footprint_publisher_node,
        imu_driver_launch,
        tmini_launch,
        ddsm115_launch,
        odometry_launch,
        rl_balancer_launch,
        foxglove_bridge_launch
    ])
