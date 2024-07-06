//
// Created by Eason Hua on 6/23/24.
//

#ifndef CUADC_FSM_H
#define CUADC_FSM_H

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <iostream>
#include <cmath>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// opencv头文件
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

// topic 头文件
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <find_object_2d/ObjectsStamped.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsResult.h>

#include <easondrone_msgs/ControlCommand.h>

#define NODE_NAME "cuadc_node"
#define FLIGHT_HEIGHT 0.3
#define REACH_DIST 0.2
#define cx 321.04638671875
#define cy 243.44969177246094
#define fx 369.502083
#define fy 369.502083

using namespace std;

// FSM_EXEC_STATE machine class
class FSM {
private:
// Define the states of the state machine
    enum FSM_EXEC_STATE {
        IDLE,
        TAKE_OFF,
        TO_THROW,
        THROW,
        TO_SEE,
        SEE,
        RETURN,
        LAND
    };

    FSM_EXEC_STATE exec_state_;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;
    bool have_odom_;
    mavros_msgs::State mavros_state;
    geographic_msgs::GeoPointStamped gp_origin;

    //变量声明 - 服务
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    Eigen::Vector3d end_pt_;
    mavros_msgs::PositionTarget pose_cmd;
    //即将发布的command
    easondrone_msgs::ControlCommand easondrone_cmd_;
    bool throw_flag_, land_flag_;
    tf::StampedTransform transform;

    int IMG_W, IMG_H;
    cv::Mat camera_intrinsics_;
    Eigen::Vector2d centerErrorPad_;
    tf::Vector3 pose_pad_vec_;
    Eigen::Vector3d pose_pad_;

    ros::Timer gp_origin_timer_, exec_timer_;
    ros::Subscriber state_sub, odom_sub_, camera_info_sub_, yolo_sub_;
    ros::Publisher gp_origin_pub, local_pos_pub, easondrone_cmd_pub_;
    ros::ServiceClient set_mode_client, arming_client;
    tf::TransformListener tf_listener_;

    inline void changeFSMExecState(FSM_EXEC_STATE new_state){
        exec_state_ = new_state;
    }

    inline void gpOriginCallback(const ros::TimerEvent &e){
        gp_origin_pub.publish(gp_origin);
    }

    void execFSMCallback(const ros::TimerEvent &e);

    inline void state_cb(const mavros_msgs::State::ConstPtr &msg){
        mavros_state = *msg;
    }

    // 保存无人机当前里程计信息，包括位置、速度和姿态
    inline void odometryCallback(const nav_msgs::OdometryConstPtr &msg){
        odom_pos_ << msg->pose.pose.position.x,
                     msg->pose.pose.position.y,
                     msg->pose.pose.position.z;

        odom_vel_ << msg->twist.twist.linear.x,
                     msg->twist.twist.linear.y,
                     msg->twist.twist.linear.z;

        //odom_acc_ = estimateAcc( msg );

        odom_orient_.w() = msg->pose.pose.orientation.w;
        odom_orient_.x() = msg->pose.pose.orientation.x;
        odom_orient_.y() = msg->pose.pose.orientation.y;
        odom_orient_.z() = msg->pose.pose.orientation.z;

        have_odom_ = true;
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
        // TODO: actually, these two parameters should be replaced by cx & cy
        IMG_W = msg->width;
        IMG_H = msg->height;

        // Save the camera calibration parameters
        camera_intrinsics_ = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                camera_intrinsics_.at<double>(i, j) = msg->K[i*3 + j];
            }
        }
    }

    void yoloCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);

public:
    FSM() : exec_state_(IDLE) {}
    ~FSM() {}

    void init(ros::NodeHandle &nh);
};

#endif //CUADC_FSM_H
