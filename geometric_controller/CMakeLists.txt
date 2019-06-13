cmake_minimum_required(VERSION 2.8.3)
project(geometric_controller)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
# generate_dynamic_reconfigure_options(
#   cfg/GeometricController.cfg
# )

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/geometric_controller.cpp 
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(geometric_controller_node
  src/geometric_controller_node.cpp
)
target_link_libraries(geometric_controller_node ${PROJECT_NAME} ${catkin_LIBRARIES})
##########
# EXPORT #
##########
cs_install()
cs_export()