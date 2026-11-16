# Gimbal control

`gimbal_control/GimbalController` owns position commands for the camera gimbal
pan and tilt joints. `gimbal_control/MaestroSystem` drives the USB Pololu Maestro
directly. The mast camera is not part of this package; it is still driven via
`/mast_angle` by `gpio_controller/mast_esp`.

## API

Publish `geometry_msgs/msg/Twist` on `/gimbal_controller/cmd_vel`:

| Field | Joint | Units |
| --- | --- | --- |
| `angular.z` | `gimbal_pan_joint` | rad/s |
| `angular.y` | `gimbal_tilt_joint` | rad/s |

Other components are ignored. The controller integrates using the control period,
limits speed to 0.5 rad/s, and clamps joint positions. Invalid angular input stops
both axes. After 0.5 seconds without a command, positions hold.

Publish `std_msgs/msg/Bool` on `/gimbal_controller/reset`. A received `true` homes
both joints to zero at the maximum joint speed; a subsequent `false` does not
cancel homing. Reset takes priority over Twist.

The drive joystick node publishes this API. `gimbal_speed` (default 0.4 rad/s)
sets the joystick pan/tilt speed. The home button resets the gimbal and the mast.

## Calibration

| Joint | Maestro channel | Servo angle at joint zero | Joint limits |
| --- | --- | --- | --- |
| Pan | 1 | 3.10 rad | -3.10 to 3.18 rad |
| Tilt | 0 | 3.75 rad | -3.75 to 2.53 rad |

Pulse calibration is 350–2500 µs over 6.2832 rad. Hardware maps
`servo_angle = center + direction * joint_position`, then converts to Maestro
quarter-microsecond targets. Calibration, channels and the serial port are in
`rover_urdf/urdf/gimbal_urdf.ros2_control.xacro`; position limits must agree with
`gimbal_urdf.xacro` and `config/ros2_controllers.yaml`.

States are the last transmitted targets, not encoder measurements. Deactivation
leaves the Maestro holding its last pulse width. The legacy `servo_pkg` must not
open the same device; core bringup no longer launches it.

## Launch

The gimbal is always part of the robot description and is spawned by
`bringup control.launch.py` (and `rover_urdf ros2_control.launch.py`).

```bash
ros2 topic pub --rate 10 /gimbal_controller/cmd_vel geometry_msgs/msg/Twist \
  '{angular: {z: 0.2}}'
ros2 topic pub --once /gimbal_controller/reset std_msgs/msg/Bool '{data: true}'
```
