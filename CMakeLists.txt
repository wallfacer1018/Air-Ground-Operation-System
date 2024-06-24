cmake_minimum_required(VERSION 2.8.3)
project(cuadc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# Find CUDA
FIND_PACKAGE(CUDA)
if (CUDA_FOUND)
  find_package(CUDA REQUIRED)
  message(STATUS "CUDA Version: ${CUDA_VERSION_STRINGS}")
  message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    -gencode arch=compute_35,code=sm_35
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    -gencode arch=compute_61,code=sm_61
    -gencode arch=compute_62,code=sm_62
  )
  add_definitions(-DGPU)
else()
  list(APPEND LIBRARIES "m")
endif()

# Find X11
MESSAGE ( STATUS "Searching for X11..." )
FIND_PACKAGE ( X11 REQUIRED )
IF ( X11_FOUND )
  INCLUDE_DIRECTORIES ( ${X11_INCLUDE_DIR} )
  LINK_LIBRARIES ( ${X11_LIBRARIES} )
  MESSAGE ( STATUS " X11_INCLUDE_DIR: " ${X11_INCLUDE_DIR} )
  MESSAGE ( STATUS " X11_LIBRARIES: " ${X11_LIBRARIES} )
ENDIF ( X11_FOUND )

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
    easondrone_msgs
    tf2_eigen
    tf
        mavros_msgs
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

add_executable(main_node src/main_node.cpp)
target_link_libraries(main_node ${catkin_LIBRARIES})
add_dependencies(main_node ${catkin_EXPORTED_TARGETS})
