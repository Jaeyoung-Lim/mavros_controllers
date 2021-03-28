cmake_minimum_required(VERSION 2.8.12)
project(parameter_estimator)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  tf
  mavros
  mavros_extras
  mavros_msgs
  mavlink
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES parameter_estimator
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs sensor_msgs tf
)

#############
# LIBRARIES #
#############
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/parameter_estimator.cpp 
  src/estimator.cpp
)
add_dependencies(parameter_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
add_executable(parameter_estimator_node
  src/parameter_estimator_node.cpp
)
add_dependencies(parameter_estimator_node parameter_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(parameter_estimator_node ${PROJECT_NAME} ${catkin_LIBRARIES})
##########
# EXPORT #
##########
# cs_install()
# cs_export()