//
// Created by hyx020222 on 6/23/24.
//

#include <ros/ros.h>
#include <iostream>
#include <easondrone_msgs/ControlCommand.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/FSM_EXEC_STATE.h>

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

    FSM_EXEC_STATE currentState;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    bool have_odom_;

    //即将发布的command
    easondrone_msgs::ControlCommand Command_to_pub;

    ros::Timer exec_timer_;
    ros::Subscriber odom_sub_;
    ros::Publisher move_pub;

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

    inline void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call){
        if (new_state == exec_state_)
            continously_called_times_++;
        else
            continously_called_times_ = 1;

        static string state_str[8] = {"INIT", "TAKE_OFF", "TO_THROW", "THROW", "TO_SEE", "SEE", "RETURN", "LAND"};
        int pre_s = int(exec_state_);
        exec_state_ = new_state;
        cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
    }

    inline void execFSMCallback(const ros::TimerEvent &e){
        switch (currentState) {
            case IDLE:{
                ROS_INFO("FSM_EXEC_STATE: IDLE");

                if (!have_odom_) {
                    cout << "[fsm] no odom." << endl;
                    return;
                }
                changeFSMExecState(TAKE_OFF, "FSM");
                break;
            }

            case TAKE_OFF: {
                ROS_INFO("FSM_EXEC_STATE: TAKE_OFF");

                if (!have_odom_) {
                    cout << "[fsm] no odom." << endl;
                    return;
                }
                else if (abs(odom_pos_(2) - 2.5) < 0.2) {
                    cout << "[fsm] close to take off height"
                    changeFSMExecState(TO_THROW, "FSM");
                }
                else{
                    Command_to_pub.header.stamp = ros::Time::now();
                    Command_to_pub.Mode = easondrone_msgs::ControlCommand::Takeoff;
                    Command_to_pub.Command_ID += 1;
                    move_pub.publish(Command_to_pub);
                }

                break;
            }

            case TO_THROW:
                ROS_INFO("FSM_EXEC_STATE: TO_THROW");

                if (!have_odom_) {
                    cout << "[fsm] no odom." << endl;
                    return;
                }

                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = easondrone_msgs::ControlCommand::Move;
                Command_to_pub.Command_ID += 1;
                Command_to_pub.Reference_State.position_ref[0] = 5;
                Command_to_pub.Reference_State.position_ref[1] = 0;
                Command_to_pub.Reference_State.position_ref[1] = 2.5;

                break;
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
    FSM() : currentState(IDLE) {}
    ~FSM() {}

    inline void init(ros::NodeHandle &nh){
        have_odom_ = false;

        /* callback */
        exec_timer_ = nh.createTimer(ros::Duration(0.01), &FSM::execFSMCallback, this);
        odom_sub_ = nh.subscribe("/mavros/local_position/odom", 1, &FSM::odometryCallback, this);
        //　【发布】　控制指令
        move_pub = nh.advertise<easondrone_msgs::ControlCommand>("/easondrone/control_command", 10);

        // 初始化命令 - Idle模式 电机怠速旋转 等待来自上层的控制指令
        Command_to_pub.Mode                                = easondrone_msgs::ControlCommand::Idle;
        Command_to_pub.Command_ID                          = 0;
        Command_to_pub.source = NODE_NAME;
        Command_to_pub.Reference_State.Move_mode           = easondrone_msgs::PositionReference::XYZ_POS;
        Command_to_pub.Reference_State.Move_frame          = easondrone_msgs::PositionReference::ENU_FRAME;
        Command_to_pub.Reference_State.position_ref[0]     = 0.0;
        Command_to_pub.Reference_State.position_ref[1]     = 0.0;
        Command_to_pub.Reference_State.position_ref[2]     = 0.0;
        Command_to_pub.Reference_State.velocity_ref[0]     = 0.0;
        Command_to_pub.Reference_State.velocity_ref[1]     = 0.0;
        Command_to_pub.Reference_State.velocity_ref[2]     = 0.0;
        Command_to_pub.Reference_State.acceleration_ref[0] = 0.0;
        Command_to_pub.Reference_State.acceleration_ref[1] = 0.0;
        Command_to_pub.Reference_State.acceleration_ref[2] = 0.0;
        Command_to_pub.Reference_State.yaw_ref             = 0.0;
        Command_to_pub.Reference_State.yaw_rate_ref        = 0.0;
    };
};
