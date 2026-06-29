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

## How Joint State Is Calculated

The driver always reads the servo registers directly, then converts them into ROS state interfaces.

For every servo it reads:

* `Present_Position`
* `Present_Speed`

The exact ROS mapping depends on the joint configuration.

### Arm Position Joints

Typical arm joints such as shoulder, elbow, wrist flex, and wrist roll use the default:

* `position_mapping=servo_angle`
* `command_interface=position`

For these joints, the ROS state is computed as:

* position state: `to_radians(raw_ticks - 2048)`
* velocity state: `to_radians(decoded_speed_ticks)`

Important details:

* `homing_offset` is applied by the servo firmware before `Present_Position` is reported, so the driver sees the already-shifted raw tick value.
* `range_min` and `range_max` are hardware limits for the servo, but they do not rescale the ROS joint state when `position_mapping=servo_angle` is used.
* If RViz and the real mechanism disagree, first check the raw measurement path with `raw_snapshot`, then adjust `homing_offset`, `range_min`, `range_max`, or the URDF joint definition.

### Gripper Jaw Joints

Grippers can be modeled in jaw-opening space instead of raw servo-shaft angle by using:

* `position_mapping=gripper_jaw`
* `command_interface=position`
* `position_lower`
* `position_upper`
* `range_min`
* `range_max`

For these joints, the driver first normalizes the raw servo ticks into the calibrated servo range:

* `normalized = (raw_ticks - range_min) / (range_max - range_min)`
* clamp `normalized` into `[0, 1]`
* if `drive_mode != 0`, flip with `normalized = 1 - normalized`

Then it maps that normalized value into the ROS gripper joint range:

* `ros_position = position_lower + normalized * (position_upper - position_lower)`

The inverse mapping is used for position commands before writing them back to the servo.

This means:

* ROS sees the modeled jaw opening angle or jaw-opening equivalent
* ROS does not see the raw servo shaft angle for that joint
* `0.0` and `position_upper` mean the calibrated closed/open ends of the configured gripper range, not necessarily `0` and `pi` of the servo itself

### Wheel Joints

Wheel joints typically use:

* `command_interface=velocity`
* `state_interface=position`
* `state_interface=velocity`

The command path is velocity-only:

* ROS sends angular velocity commands in `rad/s`
* the driver converts that command into the servo speed register format before writing it

The state path is still direct servo feedback:

* position state: `to_radians(raw_ticks - 2048)`
* velocity state: `to_radians(decoded_speed_ticks)`

So for wheel joints:

* the reported `position` state is the current servo angle around its midpoint
* it is not integrated travel distance or odometry
* if you need mobile-base odometry, compute that at the controller / base layer, not from the raw hardware interface alone

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

---

## `raw_snapshot` utility

The `feetech_driver` package also installs a small CLI utility named `raw_snapshot`. It reads the raw `Present_Position` / `Present_Speed` registers directly from one or more servos and writes the result to JSON.

This is useful when:

* you want to inspect the raw servo ticks without the ROS joint mapping layer
* you are calibrating `homing_offset`, `range_min`, or `range_max`
* you want to compare the driver's raw servo view against another stack such as LeRobot

### Build/install location

After building the workspace, the binary is typically available at:

* `install/feetech_ros2_driver/bin/raw_snapshot`

### Usage

Single snapshot:

```bash
raw_snapshot <port> <comma-separated-ids> <output-json>
```

Example:

```bash
raw_snapshot /dev/ttyACM0 1,2,3,4,5,6 /tmp/arm_snapshot.json
```

Time series capture:

```bash
raw_snapshot <port> <comma-separated-ids> <output-json> <duration-sec> <period-sec>
```

Example:

```bash
raw_snapshot /dev/ttyACM0 1,2,3,4,5,6 /tmp/arm_series.json 2.0 0.05
```

This records samples for `2.0` seconds at approximately `0.05` second intervals.

### Output format

For a single snapshot, the JSON contains one `present` block per servo id:

* `raw_position`: raw tick value reported by the servo
* `centered_ticks`: `raw_position - 2048`
* `radians_from_midpoint`: midpoint-based angle in radians
* `raw_speed`: raw register value
* `decoded_speed_ticks`: signed speed value decoded from the servo register format

For a sampled capture, the JSON contains:

* `duration_sec`
* `period_sec`
* `samples`

Each element in `samples` contains:

* `sample_index`
* `monotonic_time`
* `present`

### Calibration note

`raw_snapshot` is the right tool when you want the hardware truth before ROS-space conversion. In particular, when tuning `homing_offset` or servo tick limits, use `raw_position` as the source of truth and then derive the YAML / URDF calibration values from that measurement.
