# ros2_control

A variant with a `cad/ros2_control.json` has a `<ros2_control>` block in its xacro: `lite_dummy`, `lite_bimanual`, `lite_biped` and `lite_biped_debug`. `lite` and `lite_pro` have a model-only xacro.

## Backends

Two xacro args select the backend, as in the [Universal Robots description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/ros2/urdf/ur.ros2_control.xacro):

| `sim_mujoco` | `use_mock_hardware` | Backend |
|---|---|---|
| `false` | `false` | `humanoid_devices_robstride/RobstrideSystem` |
| `false` | `true` | `mock_components/GenericSystem` |
| `true` | any | `mujoco_ros2_control/MujocoSystem` |

Each backend has one `<ros2_control>` block for each CAN bus, with the same component names. The joint macros always write their hardware params, because each backend ignores the params that it does not know. These parts differ between the backends:

- Mock hardware declares the `safety_level` and `safety_flags` gpio state interfaces of each block. `RobstrideSystem` exports these interfaces itself, and `MujocoSystem` does not have them.
- Only the real backend gets the `can_interface` and `calibration_file` hardware params and the value of `use_linkage`.
- Only MuJoCo and the real backend have the base IMU block of the bipeds.

`sim_mujoco` is not related to the `use_sim_time` node parameter, which controls the clock.

## Args

| Arg | Default | Variants | Function |
|---|---|---|---|
| `use_mock_hardware` | `true` | all four | Selects the backend. |
| `sim_mujoco` | `false` | all four | Selects the backend. |
| `can_interface_left` | `can0`, bipeds `can2` | all four | CAN interface of the left side. |
| `can_interface_right` | `can1`, bipeds `can3` | all four | CAN interface of the right side. |
| `calibration_file` | `""` | all four | Calibration file of `RobstrideSystem`. |
| `imu_port` | `/dev/ttyUSB0` | bipeds | Serial port of the base IMU. |
| `use_linkage` | `true` | bipeds | Converts the ankle pitch between joint and actuator space through the four-bar linkage. Set it to `false` for calibration, which needs raw actuator angles. |

## Base IMU of the bipeds

The `LiteBipedIMU` sensor block gives the `base_imu` sensor:

- With `sim_mujoco:=true`, `MujocoSystem` reads the `base_quat`, `base_gyro` and `base_accel` sensors of the MJCF.
- On the real robot, `humanoid_devices_witmotion/WitImuSensor` reads the IMU on `imu_port`.
- With mock hardware, the block does not exist.

The two blocks have the same interfaces, so `imu_sensor_broadcaster` works in the same way with both.
