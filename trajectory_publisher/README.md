# trajectory_publisher

## Overview
Trajectory publisher publishes continous trajectories to the geometric controller

## Parameters
- /trajectory_publisher/initpos_x (default: 0.0)
- /trajectory_publisher/initpos_y (default: 0.0)
- /trajectory_publisher/initpos_z (default: 1.0)
- /trajectory_publisher/updaterate (default: 0.01)
- /trajectory_publisher/horizon (default: 1.0)
- /trajectory_publisher/maxjerk (default: 10.0)
- /trajectory_publisher/trajectory_type (default: 0)
- /trajectory_publisher/number_of_primitives (default: 7)
- /trajectory_publisher/shape_radius (default: 1.0)	


## Topics

The geometric controller publishes and subscribes the following topics.

- Published Topics
	- reference/trajectory ( [nav_msgs/Path](http://docs.ros.org/kinetic/api/nav_msgs/html/msg/Path.html) )
	- reference/setpoint ( [geometry_msgs/TwistStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Twist.html) )

- Subscribed Topics
    - /trajectory_publisher/motionselector ([std_msgs/int32](http://docs.ros.org/api/std_msgs/html/msg/Int32.html));
    - /mavros/local_position/pose ( [geometry_msgs/PoseStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html) )
    - /mavros/local_position/velocity( [geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html) )