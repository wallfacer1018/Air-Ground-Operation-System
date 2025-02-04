cmake_minimum_required(VERSION 3.0.2)
project(cuadc)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    image_transport
    cv_bridge
    tf2_eigen
    tf
        mavros_msgs
        rospy
)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS
            include
        CATKIN_DEPENDS
            cv_bridge
            roscpp
            rospy
            image_transport
)

include_directories(
        include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(cuadc_node
        src/cuadc_node.cpp
        src/fsm.cpp
)
target_link_libraries(cuadc_node ${catkin_LIBRARIES})
add_dependencies(cuadc_node ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS scripts/pixel2world.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
