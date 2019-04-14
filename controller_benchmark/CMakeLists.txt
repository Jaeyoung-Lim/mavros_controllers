cmake_minimum_required(VERSION 2.8.3)
project(controller_benchmark)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/controller_benchmark.cpp
  src/error_measurer.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(controller_benchmark_node
  src/controller_benchmark_node.cpp
)
target_link_libraries(controller_benchmark_node ${PROJECT_NAME} ${catkin_LIBRARIES})
##########
# EXPORT #
##########
cs_install()
cs_export()