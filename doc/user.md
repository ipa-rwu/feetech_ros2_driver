# User Guide

> **⚠️ Migration notice:** The `offset` parameter is deprecated and ignored. The driver now always centers at 2048 (midpoint of the 0–4095 tick range). Use `homing_offset` instead — it writes directly to the servo's EEPROM so the centering happens in hardware. To migrate: `homing_offset = old_offset - 2048`. The driver will log a warning if it detects the old `offset` parameter.

## ros2_control urdf tag

The feetech system interface has a few `ros2_control` urdf tags to customize its behavior.

#### Hardware Parameters

* `usb_port` (required). Example: `<param name="usb_port">/dev/ttyUSB0</param>`.
* `joint_config_file` (optional): Path to a YAML file with per-joint parameters. If omitted, only URDF params are used (backward-compatible). See [YAML Joint Configuration](#yaml-joint-configuration-file) below.

#### Per-joint Parameters

Make sure to look at [Memory table](https://docs.google.com/spreadsheets/d/1GVs7W1VS1PqdhA1nW-abeyAHhTUxKUdR/edit?gid=364516031#gid=364516031) for a detailed explanation of the parameters.

* `id` (**required**, must be in URDF): Servo ID on the bus. The driver uses this to address the servo and to match YAML config entries. Example: `<param name="id">1</param>`.
* `p_coefficient` (optional): Proportional coefficient of the PID controller. Example: `<param name="p_coefficient">8</param>`.
* `i_coefficient` (optional): Integral coefficient of the PID controller. Example: `<param name="i_coefficient">0</param>`.
* `d_coefficient` (optional): Derivative coefficient of the PID controller. Example: `<param name="d_coefficient">32</param>`.
* `homing_offset` (optional): Signed offset written to the servo's EEPROM. The servo firmware applies `Present_Position = Actual_Position - Homing_Offset`, so setting `homing_offset = actual_position - 2048` makes the servo report 2048 (center) at your desired physical center. If migrating from the old `offset` parameter: `homing_offset = old_offset - 2048` (since the old offset was effectively the actual position at center).
* `range_min` (optional): Minimum angle limit (raw ticks, after homing offset is applied).
* `range_max` (optional): Maximum angle limit (raw ticks, after homing offset is applied).
* `drive_mode` (optional): Direction flag for joints whose ROS-space direction is opposite to the raw tick direction.
* `position_mapping` (optional): Controls how the driver converts between raw servo ticks and ROS joint position. Supported values:
  * `servo_angle` (default): Legacy midpoint-based servo angle in radians.
  * `gripper_jaw`: Uses calibrated `range_min`/`range_max` and maps into the ROS joint position range defined by `position_lower`/`position_upper`.
* `position_lower` / `position_upper` (optional): ROS-space lower and upper bounds for `position_mapping=gripper_jaw`. These should match the modeled joint limits, for example the gripper jaw opening range in the URDF.
* `max_torque_limit` (optional): Maximum torque limit.
* `protection_current` (optional): Protection current threshold.
* `overload_torque` (optional): Overload torque threshold.
* `return_delay_time` (optional): Response delay time.
* `acceleration` (optional): Acceleration value.

### Example

Take a look at [ros2_so_arm100](https://github.com/JafarAbdi/ros2_so_arm100/blob/main/so_arm100_description/control/so_arm100.ros2_control.xacro) for an example of how to use the URDF tags.

---

## YAML Joint Configuration File

As an alternative (or addition) to URDF `<param>` tags, joint parameters can be loaded from a YAML file. This is useful for calibration values that change between robots (like `homing_offset`) without modifying the URDF.

The `id` parameter **must** be defined in the URDF (it is the hardware identity used to address the servo on the bus). YAML entries are matched to URDF joints by servo `id`, not by joint name — so the YAML joint name is just a human-friendly label. When both URDF params and YAML are provided for the same `id`, YAML values take precedence.

When `range_min` and `range_max` are provided, the hardware interface can use them for ROS-space conversion:

* Position joints with `position_mapping=servo_angle` keep the legacy midpoint-based radians conversion.
* Position joints with `position_mapping=gripper_jaw` use `range_min`/`range_max` plus `position_lower`/`position_upper` to map between servo ticks and ROS joint position.
* `drive_mode` flips the normalized direction before converting into the ROS joint range.
* Position commands are converted back through the inverse mapping before they are sent to the servo.

If `position_mapping=gripper_jaw` is used, `range_min`, `range_max`, `position_lower`, and `position_upper` must all be present and define increasing ranges.

### Gripper Example

Use the standard `position` interface and declare the gripper semantics explicitly:

```xml
<joint name="${prefix}gripper_joint">
  <param name="id">6</param>
  <param name="position_mapping">gripper_jaw</param>
  <param name="position_lower">0.0</param>
  <param name="position_upper">1.70</param>
  <command_interface name="position" />
  <state_interface name="position" />
  <state_interface name="velocity" />
</joint>
```

With that configuration, a ROS command of `0.0` means "fully closed" and `1.70` means "fully open" in the modeled jaw-angle space, while the driver converts that to the calibrated servo tick range internally.

### Format

```yaml
joints:
  joint_name:
    id: 1
    homing_offset: 530
    range_min: 866
    range_max: 3231
    p_coefficient: 16
    i_coefficient: 0
    d_coefficient: 32
    return_delay_time: 0
    acceleration: 254
```

### URDF Integration

Pass the YAML file path as a hardware parameter:

```xml
<param name="joint_config_file">$(find my_robot_bringup)/config/joints.yaml</param>
```

### Examples

* URDF-only setup: [ros2_so_arm100](https://github.com/JafarAbdi/ros2_so_arm100/blob/main/so_arm100_description/control/so_arm100.ros2_control.xacro)
* YAML config setup: [so101-ros-physical-ai](https://github.com/legalaspro/so101-ros-physical-ai) — see [follower](https://github.com/legalaspro/so101-ros-physical-ai/blob/main/so101_bringup/config/hardware/follower_joints.yaml) and [leader](https://github.com/legalaspro/so101-ros-physical-ai/blob/main/so101_bringup/config/hardware/leader_joints.yaml) arm configs.
