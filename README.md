# Robot Launch Package

This package orchestrates the startup of all robot nodes.

## Launch

Launch the robot with default settings:
```bash
ros2 launch robot_launch robot.launch.py
```

### Arguments

| Argument | Default | Description |
|---|---|---|
| `enable_motors` | `true` | Enable motor driver (ddsm115) and odometry node |
| `enable_rl_balancer` | `false` | Enable the RL balancing policy node |
| `rl_model_path` | bundled `balancing-robot_model_1000.onnx` | Path to the ONNX policy model file |
| `ydlidar_params_file` | `[ydlidar_ros2_driver]/params/Tmini.yaml` | Path to YDLidar parameter file |

### Examples

Launch without motors (sensors only):
```bash
ros2 launch robot_launch robot.launch.py enable_motors:=false
```

Launch with RL balancer using the bundled model:
```bash
ros2 launch robot_launch robot.launch.py enable_rl_balancer:=true
```

Launch with RL balancer using a custom model:
```bash
ros2 launch robot_launch robot.launch.py enable_rl_balancer:=true rl_model_path:=/path/to/policy.onnx
```

Launch with custom Lidar params:
```bash
ros2 launch robot_launch robot.launch.py ydlidar_params_file:=/path/to/params.yaml
```

## Nodes Started

| Node | Condition | Package |
|---|---|---|
| `robot_state_publisher` | always | `robot_tf_publisher` |
| `ekf_filter_node` | always | `robot_localization` |
| `footprint_publisher` | always | `robot_launch` |
| `imu_driver_node` | always | `imu_driver_node` |
| `ydlidar_ros2_driver_node` | always | `ydlidar_ros2_driver` |
| `foxglove_bridge` | always | `foxglove_bridge` |
| `ddsm115_driver_node` | `enable_motors:=true` | `ddsm115_driver` |
| `odometry_node` | `enable_motors:=true` | `odometry_node` |
| `rl_balancer_node` | `enable_rl_balancer:=true` | `rl_balancer_node` |

## Dependencies
- `robot_tf_publisher`
- `imu_driver_node`
- `ydlidar_ros2_driver`
- `ddsm115_driver`
- `odometry_node`
- `robot_localization`
- `rl_balancer_node`
- `foxglove_bridge`
