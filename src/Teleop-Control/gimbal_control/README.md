# Gimbal control (ROS 2 Humble)

`gimbal_control/GimbalController` owns position commands for pan, tilt, and the
independent mast camera rotation. `gimbal_control/MaestroSystem` drives the
existing USB Pololu Maestro directly. No Python servo node is needed.

## API

Publish `geometry_msgs/msg/Twist` on `/gimbal_controller/cmd_vel`:

| Field | Joint | Units |
| --- | --- | --- |
| `angular.z` | `gimbal_pan_joint` | rad/s |
| `angular.y` | `gimbal_tilt_joint` | rad/s |
| `angular.x` | `mast_camera_joint` | rad/s |

Linear components are ignored. These are joint velocity assignments, not a
Cartesian end-effector twist. The controller integrates using the control period,
limits speed to 0.5 rad/s, and clamps joint positions. Invalid angular input stops
all axes. After 0.5 seconds without a command, positions hold. The timeout uses a
steady clock, including when ROS time is paused or jumps. Nonpositive update
periods or periods longer than the timeout do not move the joints.

Publish `std_msgs/msg/Bool` on `/gimbal_controller/reset`. A received `true` requests
homing to zero at the configured maximum joint speeds; a subsequent `false` does
not cancel homing. Holding `true` holds the centered position. Publish `false` to
release reset and a fresh Twist after homing to resume motion. Reset takes priority
over Twist. Activation discards old commands and starts from the state interfaces;
deactivation holds the last target. Topics use reliable, volatile, depth-one QoS.

The existing drive joystick now publishes this API. `gimbal_speed` (default
0.4 rad/s) replaces per-message `servo_increment`. Home centers all three axes.
Servo center and range settings now belong to the hardware/description rather
than individual joystick profiles.

## Calibration and robot description

The calibration retains the existing USB servo config and joystick home angles:

| Joint | Maestro channel | Servo angle at joint zero | Joint limits |
| --- | --- | --- | --- |
| Pan | 1 | 3.10 rad | -3.10 to 3.18 rad |
| Tilt | 0 | 3.75 rad | -3.75 to 2.53 rad |
| Mast | 3 | 4.20 rad | -4.20 to 2.08 rad |

Pulse calibration is 350–2500 microseconds over 6.2832 radians. Hardware maps
`servo_angle = center + direction * joint_position`, then converts to Maestro
quarter-microsecond targets, sent as `AA device 04 channel low7 high7`. The default
device number is 12. Calibration and channels are in
`rover_urdf/urdf/gimbal_urdf.ros2_control.xacro`. Position limits must agree with
`gimbal_urdf.xacro` and `config/ros2_controllers.yaml`; the description test checks
this. The interface validates calibration, limits, unique channels, and interfaces.
Out-of-range or nonfinite commands fail before any joint commands are transmitted.
Serial writes have a bounded timeout and propagate failures to ros2_control.

**States are last successfully transmitted targets, not encoder measurements.**
The servos have no shaft feedback. Initial hardware activation sends calibrated
center targets; subsequent activation holds the last target. Deactivation and
cleanup leave the Maestro holding its last pulse width. Closing the serial port
does not disable servo power. Use the Maestro USB dual-port command interface.
The legacy `servo_pkg` process must not open the same device; core bringup no
longer launches it. The PDB/I2C backend is not used by this migration.

Pan rotates about +Z, tilt about +Y, and mast rotation about +Z. Camera optical
frames use ROS optical axes. **Mounting translations/rotations default to zero
placeholders**, and the pan/tilt pivot origins are coincident. The repository has
no measured gimbal geometry. Camera boxes, mass, and inertia are illustrative.
Set `gimbal_mount_xyz`, `gimbal_mount_rpy`, `mast_camera_xyz`, and
`mast_camera_rpy` from measured geometry before relying on camera TF for perception;
update the internal pan/tilt pivot separation in the xacro when measured. Software
validation cannot establish correct physical transforms, travel limits, or signs.

## Launch

After building and sourcing the workspace:

```bash
# Standalone simulation using ros2_control GenericSystem (no external clock needed).
ros2 launch rover_urdf gimbal.launch.py

# Standalone USB hardware; opens the existing configured by-id port.
ros2 launch rover_urdf gimbal.launch.py gimbal_mock_hardware:=false
# Optional override: gimbal_serial_port:=/dev/ttyACM0

ros2 topic pub --rate 10 /gimbal_controller/cmd_vel geometry_msgs/msg/Twist \
  '{angular: {z: 0.2}}'
ros2 topic pub --once /gimbal_controller/reset std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /gimbal_controller/reset std_msgs/msg/Bool '{data: false}'
```

Normal `bringup control.launch.py` adds the gimbal to its existing manager and
spawns the controller. It defaults to real gimbal hardware; pass
`gimbal_mock_hardware:=true` for a mock gimbal or `use_gimbal:=false` to omit it.
These flags do not mock the arm or chassis. Do not start the standalone launch
alongside normal control bringup: both own the same manager and joints.

For the separate `rover_urdf launch.py` and `ros2_control.launch.py` launches,
pass `use_gimbal:=false` to **both** to omit the gimbal hardware and controller.
The description launch owns the hardware selection; the controller launch only
selects which controllers to spawn from that description.

## Verification

The repository's general Docker/devcontainer configuration uses Jazzy. This
feature was built and tested separately using Ubuntu 22.04 / ROS 2 Humble. The
whole repository and its existing arm/CTRE dependencies were not rebuilt for
Humble. The changed production `drive.cpp` was compiled against real Humble
messages using the isolated test target below.

In a Humble environment, install dependencies and build from the workspace root:

```bash
sudo apt-get update
sudo apt-get install ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-xacro ros-humble-robot-state-publisher ros-humble-ament-cmake-gtest \
  ros-humble-geographic-msgs liburdfdom-tools
source /opt/ros/humble/setup.bash
colcon build --packages-select interfaces gimbal_control rover_urdf
source install/setup.bash
colcon test --packages-select gimbal_control
colcon test-result --verbose

# Adjust this path if the repository is itself nested under workspace/src.
gimbal_tests=src/Teleop-Control/gimbal_control/test
python3 "$gimbal_tests/test_description.py"
python3 "$gimbal_tests/test_e2e.py"
python3 "$gimbal_tests/test_e2e.py" --serial-emulator
cmake -S "$gimbal_tests/drive" -B /tmp/gimbal-drive-build
cmake --build /tmp/gimbal-drive-build
python3 "$gimbal_tests/test_e2e.py" --serial-emulator \
  --drive-executable /tmp/gimbal-drive-build/drive
```

Run the end-to-end scripts sequentially, without other rover nodes in the same
ROS domain. Each script launches and cleans up its own controller manager.

Verified: five C++ tests (plugin discovery, hardware configuration/failure paths,
serial encoding/limits/lifecycle/disconnect, controller parameter validation,
interface ordering/initial states); 16 description variants; full URDF parsing;
mock and serial-emulator end-to-end motion, reset, timeout, NaN handling, limits,
reactivation; joystick-to-serial integration and malformed joystick handling.
The serial emulator uses a pseudo-terminal and verifies real output bytes; it
cannot verify physical actuation. GenericSystem is a kinematic mock, not Gazebo
or a servo dynamics simulation.

Physical acceptance remains: measure and apply transforms; confirm channel
mapping, signs, centers, and safe travel with the actual rover; verify Twist,
reset, timeout, lifecycle hold, and serial disconnect behavior on that device.
No physical hardware was connected during these tests.

Implementation references: [Humble controller plugins](https://control.ros.org/humble/doc/ros2_controllers/doc/writing_new_controller.html)
and [Humble hardware components](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html).
