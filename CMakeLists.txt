cmake_minimum_required(VERSION 3.1.0)
project(rtt_gazebo_robot_interface)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin QUIET)

find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
    if(EIGEN3_INCLUDE_DIR)
      unset(EIGEN3_INCLUDE_DIR CACHE)
    endif()
    message(WARNING "Using internal Eigen3 (3.3.90), please consider installing it standalone.")
    set(BUILD_TESTING OFF CACHE BOOL "Disable eigen tests")
    add_subdirectory(external/eigen-3.3.90/eigen3)
    set(ENV{EIGEN3_ROOT} ${PROJECT_SOURCE_DIR}/external/eigen-3.3.90)
    set(EIGEN3_ROOTDIR $ENV{EIGEN3_ROOT})
    set(EIGEN3_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/eigen-3.3.90/eigen3)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


include_directories(${EIGEN3_INCLUDE_DIR})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fext-numeric-literals")
endif()

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
