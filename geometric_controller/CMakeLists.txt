cmake_minimum_required(VERSION 2.8.3)
project(geometric_controller)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

############
# BINARIES #
############
cs_add_executable(geometric_controller
        src/geometric_controller.cpp src/geometric_controller_node.cpp
        )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
##########
# EXPORT #
##########
#cs_install()
#cs_export()