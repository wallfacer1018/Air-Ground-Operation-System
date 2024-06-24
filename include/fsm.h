//
// Created by hyx020222 on 6/23/24.
//

#ifndef CUADC_FSM_H
#define CUADC_FSM_H

//
// Created by hyx020222 on 6/23/24.
//

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <easondrone_msgs/ControlCommand.h>

#define NODE_NAME "main_node"

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

    //变量声明 - 服务
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    geometry_msgs::PoseStamped pose;

    ros::Timer exec_timer_;
    ros::Subscriber state_sub, odom_sub_;
    ros::Publisher local_pos_pub;
    ros::ServiceClient set_mode_client, arming_client;

    // 保存无人机当前里程计信息，包括位置、速度和姿态
    inline void odometryCallback(const nav_msgs::OdometryConstPtr &msg)
    {
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

    inline void state_cb(const mavros_msgs::State::ConstPtr &msg){
        mavros_state = *msg;
    }

    inline void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call){
        exec_state_ = new_state;
    }

    inline void execFSMCallback(const ros::TimerEvent &e){
        switch (exec_state_) {
            case IDLE: {
                ROS_INFO("FSM_EXEC_STATE: IDLE");

                if (mavros_state.mode != "OFFBOARD") {
                    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                        ROS_INFO("Offboard enabled");
                    }
                } else {
                    if (!mavros_state.armed) {
                        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                            ROS_INFO("Vehicle armed");
                            changeFSMExecState(TAKE_OFF, "FSM");
                        }
                    }
                }
                break;
            }

            case TAKE_OFF: {
                ROS_INFO("FSM_EXEC_STATE: TAKE_OFF");

                if (!have_odom_) {
                    cout << "[fsm] no odom." << endl;
                    return;
                }
                else if (abs(odom_pos_(2) - 2.5) < 0.2) {
                    cout << "[fsm] close to take off height" << endl;
                    changeFSMExecState(TO_THROW, "FSM");
                }
                else {
                    pose.pose.position.x = 0;
                    pose.pose.position.y = 0;
                    pose.pose.position.z = 2.5;

                    local_pos_pub.publish(pose);
                }
                break;
            }

            case TO_THROW:{
                ROS_INFO("FSM_EXEC_STATE: TO_THROW");

                if (!have_odom_) {
                    cout << "[fsm] no odom." << endl;
                    return;
                }
                else if (abs(odom_pos_(0) - 5) < 0.2) {
                    cout << "[fsm] close to take off height" << endl;
                    changeFSMExecState(TO_THROW, "FSM");
                }
                else {
                    pose.pose.position.x = 5;
                    pose.pose.position.y = 0;
                    pose.pose.position.z = 2.5;

                    local_pos_pub.publish(pose);
                }
                break;
            }

            case THROW:
                ROS_INFO("FSM_EXEC_STATE: THROW");

                if (!have_odom_) {
                    cout << "[fsm] no odom." << endl;
                    return;
                }

                break;
            case TO_SEE:
                ROS_INFO("FSM_EXEC_STATE: ERROR");

                if (!have_odom_) {
                    cout << "[fsm] no odom." << endl;
                    return;
                }

                break;
            case SEE:
                ROS_INFO("FSM_EXEC_STATE: ERROR");

                if (!have_odom_) {
                    cout << "[fsm] no odom." << endl;
                    return;
                }

                break;
            case RETURN:
                ROS_INFO("FSM_EXEC_STATE: ERROR");

                if (!have_odom_) {
                    cout << "[fsm] no odom." << endl;
                    return;
                }

                break;
            case LAND:
                ROS_INFO("FSM_EXEC_STATE: ERROR");

                if (!have_odom_) {
                    cout << "[fsm] no odom." << endl;
                    return;
                }

                break;
        }
    }

public:
    FSM() : exec_state_(IDLE) {}
    ~FSM() {}

    inline void init(ros::NodeHandle &nh){
        have_odom_ = false;

        /* callback */
        exec_timer_ = nh.createTimer(ros::Duration(0.01), &FSM::execFSMCallback, this);
        state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &FSM::state_cb, this);
        odom_sub_ = nh.subscribe("/mavros/local_position/odom", 1, &FSM::odometryCallback, this);
        //　【发布】　控制指令
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        // 【服务】解锁/上锁 本服务通过Mavros功能包 /plugins/command.cpp 实现
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        // 【服务】修改系统模式 本服务通过Mavros功能包 /plugins/command.cpp 实现
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        offb_set_mode.request.custom_mode = "OFFBOARD";

        arm_cmd.request.value = true;

        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
    };
};

#endif //CUADC_FSM_H
