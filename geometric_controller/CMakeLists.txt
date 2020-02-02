cmake_minimum_required(VERSION 2.8.12)
project(geometric_controller)

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

generate_dynamic_reconfigure_options(
  cfg/GeometricController.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES geometric_controller
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
  src/geometric_controller.cpp 
)
add_dependencies(geometric_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
add_executable(geometric_controller_node
  src/geometric_controller_node.cpp
)
add_dependencies(geometric_controller_node geometric_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(geometric_controller_node ${PROJECT_NAME} ${catkin_LIBRARIES})
##########
# EXPORT #
##########
# cs_install()
# cs_export()
#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
    # Add gtest based cpp test target and link libraries
    catkin_add_gtest(${PROJECT_NAME}-test test/main.cpp
                                          test/test_example.cpp
                                          test/geometric_controller-test.cpp)

    if(TARGET ${PROJECT_NAME}-test)
        target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
                                                 ${catkin_LIBRARIES}
                                                 ${YAML_CPP_LIBRARIES})
    endif()

endif()
