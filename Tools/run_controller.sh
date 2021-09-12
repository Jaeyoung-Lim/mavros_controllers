#!/bin/bash

set -e

source $HOME/catkin_ws/devel/setup.bash
roslaunch geometric_controller mav_trajectory_track_circle.launch
