cmake_minimum_required(VERSION 2.8.3)
project(trajectory_publisher)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  mavros
  mavros_extras
  mavros_msgs
  mavlink
  controller_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs tf mav_planning_msgs controller_msgs mavros_msgs
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

############
# BINARIES #
############
add_executable(trajectory_publisher
        src/trajectoryPublisher.cpp
        src/trajectoryPublisher_node.cpp
        src/trajectory.cpp
        src/polynomialtrajectory.cpp
        src/shapetrajectory.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
##########
# EXPORT #
##########
#cs_install()
#cs_export()