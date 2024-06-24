cmake_minimum_required(VERSION 2.8.3)
project(cuadc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    message_generation
    rostime
    roscpp
    rosconsole
    roscpp_serialization
    sensor_msgs
    image_transport
    cv_bridge
    actionlib
    tf2_eigen
    tf
        mavros_msgs
        easondrone_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS
    thread
    date_time
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs
)

###########
## Build ##
###########

include_directories(
        include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INUCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
        $ENV{HOME}/EasonDrone/Modules/common/include
)

#add_executable(landing
#        src/autonomous_landing.cpp
#)

add_executable(cuadc_node
        src/cuadc_node.cpp
        src/fsm.cpp
)
target_link_libraries(cuadc_node ${catkin_LIBRARIES})
add_dependencies(cuadc_node ${catkin_EXPORTED_TARGETS})
