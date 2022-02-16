# turtlebot4
Turtlebot4 common packages.

## Description

The `turtlebot4_description` package contains the URDF description of the robot and the mesh files for each component.

The description can be published with the `robot_state_publisher`:

```bash
ros2 launch turtlebot4_description robot_description.launch.py
```

## Messages

The `turtlebot4_msgs` package contains the custom messages used on the turtlebot4:

* [UserButton](turtlebot4_msgs/msg/UserButton.msg): User Button states.
* [UserLed](turtlebot4_msgs/msg/UserLed.msg): User Led control.

The turtlebot4 can also use all of the messages, actions, and services that the iRobot® Create3® platform supports. See [irobot_create_msgs](https://github.com/iRobotEducation/irobot_create_msgs) for more details.

## Navigation

The `turtlebot4_navigation` packages contains launch and configuration files for using SLAM and navigation on the Turtlebot4.

### SLAM

The Turtlebot4 uses [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) to generate a map with its RPLIDAR A1M8. 

#### Asynchronous

Asynchronous mapping is recommended when running SLAM on the Turtlebot4's Raspberry Pi.

```bash
ros2 launch turtlebot4_navigation slam_async.launch.py
```

#### Synchronous

Synchronous mapping is recommended when running SLAM on a PC or in simulation.

```bash
ros2 launch turtlebot4_navigation slam_sync.launch.py
```

### Nav2

The Turtlebot4 uses [Nav2](https://navigation.ros.org/) to navigate with a map. It can be used simultaneously with SLAM to generate the map while navigating.

```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```

