# ros_phoenix
This project contains a ROS2 package for interfacing with CTRE motor controllers. It provides components for controlling Victor SPXs, Talon SRXs, and Falcon 500s. It uses the SocketCAN driver for communicating with these devices over a CAN bus. This version takes advantage of ROS2 features such as composition and improved parameter system. There is full support for changing parameters while the nodes are executing including convenient tuning of PID controllers. It supports Linux on x86-64 and 32/64-bit ARM.

This branch primarily targets ROS2 Foxy but is also compatible with Galactic. For ROS1 support see the [melodic](https://github.com/vanderbiltrobotics/ros_phoenix/tree/melodic) branch. Note that the melodic branch has a significantly different implementation and interface than this branch.

The current version of the code should be stable. However, if you find any bugs or other issues please create an issue or pull request.

## Cloning and Building
1. Clone the package into a ROS2 workspace
```
$ cd <ros_ws>/src
$ git clone https://github.com/vanderbiltrobotics/ros_phoenix
$ cd ros_phoenix
$ git checkout foxy
```
2. Build the workspace and source the setup file
```
$ cd <ros_ws>
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
$ source install/setup.bash
```

## Launch File Usage
A launch file is the recommended way to create and configure multiple motor controllers. See [`launch/demo.launch.py`](https://github.com/vanderbiltrobotics/ros_phoenix/blob/foxy/launch/demo.launch.py) for an example. Alternatively, components can be created from the CLI.

### YAML parameter lists
If trying to load parameters in a YAML file into a launch file using ComposableNodes beware of this [known issue in launch_ros](https://github.com/ros2/launch_ros/issues/156). It is still possible to load parameter files, but the file format is slightly different than standard ROS convention. See the issue for details.

## CLI Usage
This package uses ROS2 components to compose nodes into a single process. See the [ROS2 composition tutorial](https://index.ros.org/doc/ros2/Tutorials/Composition/) for a full guide to composition.

1. Start a component container and rename it PhoenixContainer (or any other name)
```
$ ros2 run ros_phoenix phoenix_container --ros-args -r __node:=PhoenixContainer
```
2. Load a ros_phoenix component into the container with the device id number
```
$ ros2 component load /PhoenixContainer ros_phoenix ros_phoenix::TalonSRX --node-name my_talon -p id:=0
```
3. Repeat step 2 for each motor controller

## Phoenix Container
The `phoenix_container` allows motor controller nodes to be dynamically created at runtime. It is responsible for configuring the CAN interface used by all of the motor controllers created inside it. It also feeds the CTRE watchdog. This watchdog is responsible for enabling/disabling all motor controllers on the interface. This global watchdog is separate from the watchdog inside each component which disables an individual motor if it stops receiving updates.

### Parameters
- `period_ms` (50): Period in milliseconds at which watchdog is feed
- `watchdog_ms` (200): Period before motor controllers are disabled if watchdog dies

## Phoenix Components
- `ros_phoenix::VictorSPX`: Victor SPX Component
- `ros_phoenix::TalonSRX`: Talon SRX Component
- `ros_phoenix::TalonFX`: Talon FX / Falcon 500 Component

## Component Interface
### Published Topics
`<node_name>/status` (ros_phoenix/msg/MotorStatus)
- Publishes status information about the motor controller

### Subscribed Topics
`<node_name>/set` (ros_phoenix/msg/MotorControl)
- Sets the control mode and output of the motor controller

### Parameters
- `id` (0): Device CAN bus ID
- `interface` ("can0"): The SocketCAN interface
- `period_ms` (20): Period in milliseconds of status updates
- `watchdog_ms` (100): Watchdog timer. Must be greater than period_ms!
- `follow_id` (-1): If >= to zero, the ID of another device to follow
- `edges_per_rot` (2048): Encoder edges per rotation
- `invert` (false): Invert motor output
- `invert_sensor` (false): Invert sensor direction
- `brake_mode` (true): Enable brake mode
- `analog_input` (false): Use an analog input instead of an encoder
- `max_voltage` (12.0): Voltage which corresponds to a percent output of 1.0
- `max_current` (30.0): Maximum current output
- `sensor_multiplier` (1.0): Factor to convert native device units to meaningful units
- `P` (0.0): kP
- `I` (0.0): kI
- `D` (0.0): kD
- `F` (0.0): kF

## ros2_control Integration
This package additionally contains two implementations of a ros2_control hardware interface
which can be used to control CTRE motor controllers. The `ros_phoenix/PhoenixBridge` acts as
a bridge between phoenix components and the ros2_control interface. It requires that the
components are started separately in a `phoenix_container`. The motor controllers are then
controlled through the topic interfaces described above. To avoid the overhead of using
topics, the `ros_phoenix/PhoenixSystem` can be used standalone without a `phoenix_container`.
The system has the same interface as the bridge but does not publish any status or control
topics.

See examples of how to configure these hardware interfaces in [`ros2_control/`]
(https://github.com/vanderbiltrobotics/ros_phoenix/tree/foxy/ros2_control). These examples
are based off of the ros2_control diffbot example. To see how this configuration can be used
look at the original example in `ros2_control_demos/ros2_control_demo_description`.

### Parameter Interface
The `ros_phoenix/PhoenixBridge` does not provide any interface for configuring the motor
controllers because this configuration is done while launching the PhoenixContainer and
adding components. Whereas, `ros_phoenix/PhoenixSystem` allows the configuration of all
the same parameters through the xml config file. Using the `<param/>` tag, hardware
parameters are given to the `PhoenixManager`. It accepts all the same parameters
as the `PhoenixContainer`. Each motor controller corresponds to one joint. Therefore,
the joint parameters match the available component parameters.

Both hardware interface implementations support dynamic reconfiguration of parameters.