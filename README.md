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