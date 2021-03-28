# geometric_controller

Trajectory trackking controller using [mavros](https://github.com/mavlink/mavros) package in PX4 OFFBOARD mode. This is a implementation of a geometric controller and differentially flat system considering rotor drag

## Overview
Trajectory trackking controller using [mavros](https://github.com/mavlink/mavros) package in PX4 OFFBOARD mode. This is a implementation of a geometric controller and differentially flat system considering rotor drag

[![Hovering done](https://img.youtube.com/vi/FRaPGjX9m-c/0.jpg)](https://youtu.be/FRaPGjX9m-c "Hovering done")

## Parameters
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


## Topics

The geometric controller publishes and subscribes the following topics.

- Published Topics
	- command/bodyrate_command ( [mavros_msgs/AttitudeTarget](http://docs.ros.org/api/mavros_msgs/html/msg/AttitudeTarget.html) )
	- reference/pose ( [geometry_msgs/PoseStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html) )

- Subscribed Topics
	- reference/setpoint ( [geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html) )
	- /mavros/state ( [mavr0s_msgs/State](http://docs.ros.org/api/mavros_msgs/html/msg/State.html) )
	- /mavros/local_position/pose ( [geometry_msgs/PoseStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html) )
	- /gazebo/model_states( [gazebo_msgs/ModelStates](http://docs.ros.org/kinetic/api/gazebo_msgs/html/msg/ModelState.html) )
	- /mavros/local_position/velocity( [geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html) )


## References
[1] Lee, Taeyoung, Melvin Leoky, and N. Harris McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)." Decision and Control (CDC), 2010 49th IEEE Conference on. IEEE, 2010.

[2] Faessler, Matthias, Antonio Franchi, and Davide Scaramuzza. "Differential flatness of quadrotor dynamics subject to rotor drag for accurate tracking of high-speed trajectories." IEEE Robot. Autom. Lett 3.2 (2018): 620-626.


## Contact
Jaeyoung Lim 	jalim@student.ethz.ch