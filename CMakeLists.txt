cmake_minimum_required(VERSION 3.1.0)
project(rtt_gazebo_robot_interface)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin QUIET)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(
    include
    ${OROCOS-RTT_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

orocos_component(rtt_gazebo_robot_interface src/rtt_gazebo_robot_interface.cpp)
target_link_libraries(rtt_gazebo_robot_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

orocos_generate_package()