//
// Created by Eason Hua on 6/24/24.
//

#include "fsm.h"

void FSM::execFSMCallback(const ros::TimerEvent &e){
    switch (exec_state_) {
        case IDLE: {
            ROS_INFO("FSM_EXEC_STATE: IDLE");

            if (mavros_state.mode != "OFFBOARD") {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled");
                }
            }
            else {
                if (!mavros_state.armed) {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed");
                        changeFSMExecState(TAKE_OFF);
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

            end_pt_(0) = 0; end_pt_(1) = 0; end_pt_(2) = FLIGHT_HEIGHT;

            if ((odom_pos_ - end_pt_).norm() < 0.2) {
                cout << "[fsm] close to take off height" << endl;
                changeFSMExecState(TO_THROW);
            }
            else {
//                    pose_cmd.header.stamp = ros::Time::now();
//                    pose_cmd.position.x = end_pt_(0);
//                    pose_cmd.position.y = end_pt_(1);
//                    pose_cmd.position.z = end_pt_(2);
//
//                    local_pos_pub.publish(pose_cmd);

                easondrone_cmd_.header.stamp = ros::Time::now();
                easondrone_cmd_.Mode = easondrone_msgs::ControlCommand::Move;
                easondrone_cmd_.Command_ID += 1;
                easondrone_cmd_.Reference_State.position_ref[0] = end_pt_(0);
                easondrone_cmd_.Reference_State.position_ref[1] = end_pt_(1);
                easondrone_cmd_.Reference_State.position_ref[2] = end_pt_(2);
                easondrone_cmd_pub_.publish(easondrone_cmd_);
            }
            break;
        }

        case TO_THROW:{
            ROS_INFO("FSM_EXEC_STATE: TO_THROW");

            if (!have_odom_) {
                cout << "[fsm] no odom." << endl;
                return;
            }

            end_pt_(0) = 32.5; end_pt_(1) = 0; end_pt_(2) = FLIGHT_HEIGHT;

            if ((odom_pos_ - end_pt_).norm() < 0.2) {
                cout << "[fsm] close to throw area" << endl;
                changeFSMExecState(THROW);
            }
            else {
                easondrone_cmd_.header.stamp = ros::Time::now();
                easondrone_cmd_.Mode = easondrone_msgs::ControlCommand::Move;
                easondrone_cmd_.Command_ID += 1;
                easondrone_cmd_.Reference_State.position_ref[0] = end_pt_(0);
                easondrone_cmd_.Reference_State.position_ref[1] = end_pt_(1);
                easondrone_cmd_.Reference_State.position_ref[2] = end_pt_(2);
                easondrone_cmd_pub_.publish(easondrone_cmd_);
            }
            break;
        }

        case THROW: {
            ROS_INFO("FSM_EXEC_STATE: THROW");

            if (!have_odom_) {
                cout << "[fsm] no odom." << endl;
                return;
            }

            break;
        }

        case TO_SEE: {
            ROS_INFO("FSM_EXEC_STATE: TO_SEE");

            if (!have_odom_) {
                cout << "[fsm] no odom." << endl;
                return;
            }

            end_pt_(0) = 57.5; end_pt_(1) = 0; end_pt_(2) = FLIGHT_HEIGHT;

            if ((odom_pos_ - end_pt_).norm() < 0.2) {
                cout << "[fsm] close to see area" << endl;
                changeFSMExecState(SEE);
            }
            else {
                easondrone_cmd_.header.stamp = ros::Time::now();
                easondrone_cmd_.Mode = easondrone_msgs::ControlCommand::Move;
                easondrone_cmd_.Command_ID += 1;
                easondrone_cmd_.Reference_State.position_ref[0] = end_pt_(0);
                easondrone_cmd_.Reference_State.position_ref[1] = end_pt_(1);
                easondrone_cmd_.Reference_State.position_ref[2] = end_pt_(2);
                easondrone_cmd_pub_.publish(easondrone_cmd_);
            }
            break;
        }

        case SEE: {
            ROS_INFO("FSM_EXEC_STATE: ERROR");

            if (!have_odom_) {
                cout << "[fsm] no odom." << endl;
                return;
            }

            break;
        }

        case RETURN: {
            ROS_INFO("FSM_EXEC_STATE: RETURN");

            if (!have_odom_) {
                cout << "[fsm] no odom." << endl;
                return;
            }

            end_pt_(0) = 0; end_pt_(1) = 0; end_pt_(2) = FLIGHT_HEIGHT;

            if ((odom_pos_ - end_pt_).norm() < 0.2) {
                cout << "[fsm] close to land area" << endl;
                changeFSMExecState(LAND);
            }
            else {
                easondrone_cmd_.header.stamp = ros::Time::now();
                easondrone_cmd_.Mode = easondrone_msgs::ControlCommand::Move;
                easondrone_cmd_.Command_ID += 1;
                easondrone_cmd_.Reference_State.position_ref[0] = end_pt_(0);
                easondrone_cmd_.Reference_State.position_ref[1] = end_pt_(1);
                easondrone_cmd_.Reference_State.position_ref[2] = end_pt_(2);
                easondrone_cmd_pub_.publish(easondrone_cmd_);
            }

            break;
        }

        case LAND: {
            ROS_INFO("FSM_EXEC_STATE: LAND");

            if (!have_odom_) {
                cout << "[fsm] no odom." << endl;
                return;
            }

            break;
        }
    }
}
