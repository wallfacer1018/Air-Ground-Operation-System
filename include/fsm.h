//
// Created by Eason Hua on 6/23/24.
//

#ifndef CUADC_FSM_H
#define CUADC_FSM_H

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
// opencv头文件
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
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <find_object_2d/ObjectsStamped.h>
#include <easondrone_msgs/ControlCommand.h>

#define NODE_NAME "cuadc_node"
#define FLIGHT_HEIGHT 0.3
#define REACH_DIST 0.2
#define IMG_W 640
#define IMG_H 480

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
    bool land_flag_;
    tf::StampedTransform transform;

    Eigen::Vector2d vital_pose_;
    find_object_2d::ObjectsStamped find_object_;
    Eigen::Vector2d object_pixel_;

    ros::Timer gp_origin_timer_, exec_timer_;
    ros::Subscriber state_sub, odom_sub_, vital_sub_, find_object_sub_;
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
        odom_pos_(0) = msg->pose.pose.position.x;
        odom_pos_(1) = msg->pose.pose.position.y;
        odom_pos_(2) = msg->pose.pose.position.z;

        odom_vel_(0) = msg->twist.twist.linear.x;
        odom_vel_(1) = msg->twist.twist.linear.y;
        odom_vel_(2) = msg->twist.twist.linear.z;

        //odom_acc_ = estimateAcc( msg );

        odom_orient_.w() = msg->pose.pose.orientation.w;
        odom_orient_.x() = msg->pose.pose.orientation.x;
        odom_orient_.y() = msg->pose.pose.orientation.y;
        odom_orient_.z() = msg->pose.pose.orientation.z;

        have_odom_ = true;
    }

    inline void vitalCallback(const geometry_msgs::Pose::ConstPtr &msg) {
        if(land_flag_){
            return;
        }

        vital_pose_(0) = msg->position.x;
        vital_pose_(1) = msg->position.y;

        cout << "vital: " << vital_pose_.transpose() << endl;
    }

    inline void findObjectCallback(const find_object_2d::ObjectsStamped::ConstPtr &msg){
        if(msg->objects.data.size()){
            find_object_ = *msg;

            object_pixel_(0) = find_object_.objects.data[10] - IMG_H/2;
            object_pixel_(1) = find_object_.objects.data[9] - IMG_W/2;
            cout << "object: " << object_pixel_.transpose() << endl;
        }
    }

public:
    FSM() : exec_state_(IDLE) {}
    ~FSM() {}

    inline void init(ros::NodeHandle &nh){
        /******** callback ********/
        gp_origin_timer_ = nh.createTimer
                (ros::Duration(0.01), &FSM::gpOriginCallback, this);
        exec_timer_ = nh.createTimer
                (ros::Duration(0.01), &FSM::execFSMCallback, this);

        state_sub = nh.subscribe<mavros_msgs::State>
                ("/mavros/state", 10, &FSM::state_cb, this);
        odom_sub_ = nh.subscribe
                ("/mavros/local_position/odom", 10, &FSM::odometryCallback, this);
        vital_sub_ = nh.subscribe<geometry_msgs::Pose>
                ("/visual_infer/pose", 10, &FSM::vitalCallback, this);
        find_object_sub_ = nh.subscribe<find_object_2d::ObjectsStamped>
                ("/objectsStamped", 10, &FSM::findObjectCallback, this);

        gp_origin_pub = nh.advertise<geographic_msgs::GeoPointStamped>
                ("/mavros/global_position/gp_origin", 10);
        //　【发布】位置/速度/加速度期望值 坐标系 ENU系 本话题要发送至飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp发送), 对应Mavlink消息为SET_POSITION_TARGET_LOCAL_NED (#84), 对应的飞控中的uORB消息为position_setpoint_triplet.msg
        local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
                ("/mavros/setpoint_position/local", 10);
        //　【发布】控制指令
        easondrone_cmd_pub_ = nh.advertise<easondrone_msgs::ControlCommand>
                ("/easondrone/control_command", 10);

        // 【服务】解锁/上锁 本服务通过Mavros功能包 /plugins/command.cpp 实现
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                ("/mavros/cmd/arming");
        // 【服务】修改系统模式 本服务通过Mavros功能包 /plugins/command.cpp 实现
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                ("/mavros/set_mode");

        //监听包装在一个try-catch块中以捕获可能的异常
        try{
            //向侦听器查询特定的转换，(想得到/turtle1到/turtle2的变换)，想要转换的时间ros::Time(0)提供了最新的可用转换。
            tf_listener_.lookupTransform("base_link", "monocular_link", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
        }

        /******* init ********/
        have_odom_ = false;

        gp_origin.header.stamp = ros::Time::now();
        gp_origin.header.frame_id = "world";
        gp_origin.position.latitude = 0;
        gp_origin.position.longitude = 0;
        gp_origin.position.altitude = 0;

        offb_set_mode.request.custom_mode = "OFFBOARD";

        arm_cmd.request.value = true;

        end_pt_(0) = 0; end_pt_(1) = 0; end_pt_(2) = 0;

        pose_cmd.header.stamp = ros::Time::now();
        pose_cmd.header.frame_id = "world";
        pose_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
        //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
        //Bit 10 should set to 0, means is not force sp
        pose_cmd.type_mask =    // mavros_msgs::PositionTarget::IGNORE_PX |
                                // mavros_msgs::PositionTarget::IGNORE_PY |
                                // mavros_msgs::PositionTarget::IGNORE_PZ |
                                mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::FORCE |
                                // mavros_msgs::PositionTarget::IGNORE_YAW |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        // 初始化命令 - Idle模式 电机怠速旋转 等待来自上层的控制指令
        easondrone_cmd_.Mode = easondrone_msgs::ControlCommand::Idle;
        easondrone_cmd_.Command_ID = 0;
        easondrone_cmd_.source = NODE_NAME;
        easondrone_cmd_.Reference_State.Move_mode = easondrone_msgs::PositionReference::XYZ_POS;
        easondrone_cmd_.Reference_State.Move_frame = easondrone_msgs::PositionReference::ENU_FRAME;

        land_flag_ = false;

        ROS_INFO("FSM initialized.");
    };
};

#endif //CUADC_FSM_H
