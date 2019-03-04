# mavros_controllers
[![Build Status](https://travis-ci.org/Jaeyoung-Lim/mavros_controllers.png?branch=master)](https://travis-ci.org/Jaeyoung-Lim/mavros_controllers)

Controllers for controlling MAVs using the [mavros](https://github.com/mavlink/mavros) package in OFFBOARD mode.


## Overview
The repository contains controllers for controlling MAVs using the mavros package. The following packages are included in the repo
- geometric_controller: Trajectory tracking controller based on geometric control
- controller_msgs: custom message definitions
- trajectory_publisher: Node publishing setpoints as states from motion primitives / trajectories for the controller to follow.

[![Hovering done](https://img.youtube.com/vi/FRaPGjX9m-c/0.jpg)](https://youtu.be/FRaPGjX9m-c "Hovering done")

[![Circular trajectory tracking](https://img.youtube.com/vi/IEyocdnlYw0/0.jpg)](https://youtu.be/IEyocdnlYw0 "Circular trajectory tracking")

## Getting Started
### Install PX4 SITL(Only to Simulate)
Follow the instructions as shown in the [PX4 Documentation](http://dev.px4.io/en/simulation/ros_interface.html)
To check if the necessary environment is setup correctly, you can run the gazebo SITL using the following command
```
cd <Firmware_directory>
make posix_sitl_default gazebo
```
To source the PX4 environment, run the following commands
```
source ~/catkin_ws/devel/setup.bash    // (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```
You can run the rest of the roslaunch files in the same terminal

### Building mavros_controllers
mavros_controllers can be built using `catkin build`
```
cd <path_to_catkin_ws>
catkin build mavros_controllers
```

## Running the code
The following launch file enables the geometric controller to follow a circular trajectory
```
roslaunch geometric_controller trajectory_track_circle.launch
```

## Nodes
`mavros_controllers` include the following packages.
### geometric_controller

The geometric controller publishes and subscribes the following topics.
- Parameters
    - /geometric_controller/mavname (default: "iris")
    - /geometric_controller/ctrl_mode (default: MODE_BODYRATE)
    - /geometric_controller/enable_sim (default: true)
    - /geometric_controller/enable_gazebo_state (default: false)
    - /geometric_controller/max_acc (default: 7.0)
    - /geometric_controller/yaw_heading (default: 0.0)
    - /geometric_controller/drag_dx (default: 0.0)
    - /geometric_controller/drag_dy (default: 0.0)
    - /geometric_controller/drag_dz (default: 0.0)
    - /geometric_controller/attctrl_constant (default: 0.2)
    - /geometric_controller/normalizedthrust_constant (default: 0.1)

- Published Topics
	- command/bodyrate_command ( [mavros_msgs/AttitudeTarget](http://docs.ros.org/api/mavros_msgs/html/msg/AttitudeTarget.html) )
	- reference/pose ( [geometry_msgs/PoseStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html) )

- Subscribed Topics
	- reference/setpoint ( [geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html) )
	- /mavros/state ( [mavr0s_msgs/State](http://docs.ros.org/api/mavros_msgs/html/msg/State.html) )
	- /mavros/local_position/pose ( [geometry_msgs/PoseStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html) )
	- /gazebo/model_states( [gazebo_msgs/ModelStates](http://docs.ros.org/kinetic/api/gazebo_msgs/html/msg/ModelState.html) )
	- /mavros/local_position/velocity( [geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html) )

### trajectory_publisher

Trajectory publisher publishes continous trajectories to the trajectory_controller.
- Parameters
    - /trajectory_publisher/initpos_x (default: 0.0)
    - /trajectory_publisher/initpos_y (default: 0.0)
    - /trajectory_publisher/initpos_z (default: 1.0)
    - /trajectory_publisher/updaterate (default: 0.01)
    - /trajectory_publisher/horizon (default: 1.0)
    - /trajectory_publisher/maxjerk (default: 10.0)
    - /trajectory_publisher/trajectory_type (default: 0)
    - /trajectory_publisher/number_of_primitives (default: 7)
    - /trajectory_publisher/shape_radius (default: 1.0)

- Published Topics
	- reference/trajectory ( [nav_msgs/Path](http://docs.ros.org/kinetic/api/nav_msgs/html/msg/Path.html) )
	- reference/setpoint ( [geometry_msgs/TwistStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Twist.html) )

- Subscribed Topics
    - /trajectory_publisher/motionselector ([std_msgs/int32](http://docs.ros.org/api/std_msgs/html/msg/Int32.html));
    - /mavros/local_position/pose ( [geometry_msgs/PoseStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html) )
    - /mavros/local_position/velocity( [geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html) )

## References
[1] Lee, Taeyoung, Melvin Leoky, and N. Harris McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)." Decision and Control (CDC), 2010 49th IEEE Conference on. IEEE, 2010.

[2] Faessler, Matthias, Antonio Franchi, and Davide Scaramuzza. "Differential flatness of quadrotor dynamics subject to rotor drag for accurate tracking of high-speed trajectories." IEEE Robot. Autom. Lett 3.2 (2018): 620-626.

## Contact
Jaeyoung Lim 	jalim@student.ethz.ch