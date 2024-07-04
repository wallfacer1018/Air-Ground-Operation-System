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
                ROS_ERROR("Odom Lost! Take Off Refused!");
                return;
            }

            end_pt_(0) = 0; end_pt_(1) = 0; end_pt_(2) = FLIGHT_HEIGHT;

            if ((odom_pos_ - end_pt_).norm() < REACH_DIST) {
                ROS_INFO("Close to take off height.");
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

            end_pt_(0) = 32.5; end_pt_(1) = 0; end_pt_(2) = FLIGHT_HEIGHT;

            if ((odom_pos_ - end_pt_).norm() < REACH_DIST) {
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

            throw_flag_ = false;

            break;
        }

        case TO_SEE: {
            ROS_INFO("FSM_EXEC_STATE: TO_SEE");

            end_pt_(0) = 57.5; end_pt_(1) = 0; end_pt_(2) = FLIGHT_HEIGHT;

            if ((odom_pos_ - end_pt_).norm() < REACH_DIST) {
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

            break;
        }

        case RETURN: {
            ROS_INFO("FSM_EXEC_STATE: RETURN");

            end_pt_(0) = 0; end_pt_(1) = 0; end_pt_(2) = FLIGHT_HEIGHT;

            if ((odom_pos_ - end_pt_).norm() < REACH_DIST) {
                cout << "[fsm] close to land area" << endl;
                land_flag_ = true;
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

            end_pt_(0) = odom_pos_(0) + transform.getOrigin().x() + vital_pose_(0);
            end_pt_(1) = odom_pos_(1) + transform.getOrigin().y() - vital_pose_(1);
            end_pt_(2) = 0;

            if ((odom_pos_ - end_pt_).norm() < REACH_DIST) {
                land_flag_ = false;

                end_pt_(0) = odom_pos_(0);
                end_pt_(1) = odom_pos_(1);

                cout<< "CLOSE, land at: " << end_pt_.transpose() << endl;

                easondrone_cmd_.header.stamp = ros::Time::now();
                easondrone_cmd_.Mode = easondrone_msgs::ControlCommand::Land;
                easondrone_cmd_.Command_ID += 1;
                easondrone_cmd_pub_.publish(easondrone_cmd_);
            }
            else {
                cout<< "FAR, land at: " << end_pt_.transpose() << endl;

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
    }
}

void FSM::yoloCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg){
    cout << "------------------ YOLO ------------------" << endl;

    switch (exec_state_) {
        case THROW: {

            break;
        }

        case LAND: {
            double ratio_ = 1.0;

            for (int cnt_ = 0; cnt_ < (msg->bounding_boxes).size(); cnt_++) {
                darknet_ros_msgs::BoundingBox boundingBox = msg->bounding_boxes[cnt_];

                if (boundingBox.Class == "landing_pad") {
                    int x_range_ = boundingBox.xmax - boundingBox.xmin;
                    int y_range_ = boundingBox.ymax - boundingBox.ymin;

                    if (cnt_ == 0){
                        center_pad_(0) = boundingBox.xmin + x_range_/2 - IMG_W/2;
                        center_pad_(1) = boundingBox.ymin + y_range_/2 - IMG_H/2;

                        ratio_ = abs(x_range_ - y_range_)/double(x_range_ + y_range_);

                        cout << "INIT   centerErrorPad: " << center_pad_.transpose() << ", ratio: " << ratio_ << endl;
                    }
                    else if (cnt_ > 0){
                        Eigen::Vector2d center_pad_temp(boundingBox.xmin + x_range_/2 - IMG_W/2,
                                                        boundingBox.ymin + y_range_/2 - IMG_H/2);

                        double ratio_temp = abs(x_range_ - y_range_)/double(x_range_ + y_range_);

                        if (ratio_temp < ratio_){
                            center_pad_ = center_pad_temp;
                            ratio_ = ratio_temp;

                            cout << "BETTER centerErrorPad[" << cnt_ << "]: " << center_pad_.transpose() << ", ratio: " << ratio_ << endl;
                        }
                        else{
                            cout << "WORSE  centerErrorPad[" << cnt_ << "]: " << center_pad_temp.transpose() << ", ratio: " << ratio_temp << endl;
                        }
                    }
                }
                else{
                    cout << "NOT a landing_pad!" << endl;
                }
            }
            cout << "FINAL  centerErrorPad: " << center_pad_.transpose() << ", ratio: " << ratio_ << endl;

            break;
        }

        default:{
            return;
        }
    }
}

void FSM::init(ros::NodeHandle &nh){
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
    yolo_sub_ = nh.subscribe<darknet_ros_msgs::BoundingBoxes>
            ("/darknet_ros/bounding_boxes", 10, &FSM::yoloCallback, this);

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
    //setprecision(n) 设显示小数精度为n位
    cout << setprecision(4);

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

    throw_flag_ = false;
    land_flag_ = false;

    detectedBucket_ = false;
    detectedPad_ = false;

    ROS_INFO("FSM initialized.");
};
