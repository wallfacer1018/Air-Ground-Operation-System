//
// Created by Eason Hua on 6/24/24.
// last updated on 2024.07.22
//

#include "fsm.h"

void FSM::execFSMCallback(const ros::TimerEvent &e){
    switch (exec_state_) {
        case IDLE: {
            ROS_INFO("FSM_EXEC_STATE: IDLE");

            if (current_state.mode != "OFFBOARD") {
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled");
                }
            }
            else {
                if (!current_state.armed) {
                    if (arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed");
                        changeFSMExecState(TAKE_OFF);
                    }
                }
            }

            pos_setpoint.position.x = odom_pos_(0);
            pos_setpoint.position.y = odom_pos_(1);
            pos_setpoint.position.z = odom_pos_(2);
            pos_setpoint.yaw = odom_yaw_;

            break;
        }

        case TAKE_OFF: {
            ROS_INFO("FSM_EXEC_STATE: TAKE_OFF");

            if (!have_odom_) {
                ROS_ERROR("Odom Lost! Takeoff Refused!");
                return;
            }

            end_pt_ << 0.0, 0.0, FLIGHT_HEIGHT;

            if ((odom_pos_ - end_pt_).norm() < REACH_DIST) {
                ROS_INFO("Close to take off height.");

                pos_setpoint.position.x = odom_pos_(0);
                pos_setpoint.position.y = odom_pos_(1);
                pos_setpoint.position.z = odom_pos_(2);
                pos_setpoint.yaw = odom_yaw_;

                changeFSMExecState(TO_THROW);
            }
            else {
                pos_setpoint.position.x = end_pt_(0);
                pos_setpoint.position.y = end_pt_(1);
                pos_setpoint.position.z = end_pt_(2);
                pos_setpoint.yaw = odom_yaw_;
            }

            break;
        }

        case TO_THROW:{
            ROS_INFO("FSM_EXEC_STATE: TO_THROW");

            end_pt_ << 32.5, 0.0, FLIGHT_HEIGHT;

            if ((odom_pos_ - end_pt_).norm() < REACH_DIST) {
                cout << "[fsm] close to throw area" << endl;

                pos_setpoint.position.x = odom_pos_(0);
                pos_setpoint.position.y = odom_pos_(1);
                pos_setpoint.position.z = odom_pos_(2);
                pos_setpoint.yaw = odom_yaw_;

                changeFSMExecState(THROW);
            }
            else {
                pos_setpoint.position.x = end_pt_(0);
                pos_setpoint.position.y = end_pt_(1);
                pos_setpoint.position.z = end_pt_(2);
                pos_setpoint.yaw = odom_yaw_;
            }
            break;
        }

        case THROW: {
            ROS_INFO("FSM_EXEC_STATE: THROW");

            throw_flag_ = false;

            //TODO:

            pos_setpoint.position.x = odom_pos_(0);
            pos_setpoint.position.y = odom_pos_(1);
            pos_setpoint.position.z = odom_pos_(2);
            pos_setpoint.yaw = odom_yaw_;

            break;
        }

        case TO_SEE: {
            ROS_INFO("FSM_EXEC_STATE: TO_SEE");

            end_pt_ << 57.5, 0.0, FLIGHT_HEIGHT;

            if ((odom_pos_ - end_pt_).norm() < REACH_DIST) {
                cout << "[fsm] close to see area" << endl;

                pos_setpoint.position.x = odom_pos_(0);
                pos_setpoint.position.y = odom_pos_(1);
                pos_setpoint.position.z = odom_pos_(2);
                pos_setpoint.yaw = odom_yaw_;

                changeFSMExecState(SEE);
            }
            else {
                pos_setpoint.position.x = end_pt_(0);
                pos_setpoint.position.y = end_pt_(1);
                pos_setpoint.position.z = end_pt_(2);
                pos_setpoint.yaw = odom_yaw_;
            }
            break;
        }

        case SEE: {
            ROS_INFO("FSM_EXEC_STATE: ERROR");

            // TODO:

            pos_setpoint.position.x = odom_pos_(0);
            pos_setpoint.position.y = odom_pos_(1);
            pos_setpoint.position.z = odom_pos_(2);
            pos_setpoint.yaw = odom_yaw_;

            break;
        }

        case RETURN: {
            ROS_INFO("FSM_EXEC_STATE: RETURN");

            end_pt_ << 0.0, 0.0, FLIGHT_HEIGHT;

            if ((odom_pos_ - end_pt_).norm() < REACH_DIST) {
                cout << "[fsm] close to land area" << endl;
                land_flag_ = true;

                pos_setpoint.position.x = odom_pos_(0);
                pos_setpoint.position.y = odom_pos_(1);
                pos_setpoint.position.z = odom_pos_(2);
                pos_setpoint.yaw = odom_yaw_;

                changeFSMExecState(LAND);
            }
            else {
                pos_setpoint.position.x = odom_pos_(0);
                pos_setpoint.position.y = odom_pos_(1);
                pos_setpoint.position.z = odom_pos_(2);
                pos_setpoint.yaw = odom_yaw_;
            }
            break;
        }

        case LAND: {
            ROS_INFO("FSM_EXEC_STATE: LAND");

            end_pt_ = pose_pad_;

            if ((odom_pos_ - end_pt_).norm() < REACH_DIST) {
                land_flag_ = false;

                end_pt_(0) = odom_pos_(0);
                end_pt_(1) = odom_pos_(1);

                cout<< "CLOSE, land at: " << end_pt_.transpose() << endl;

                pos_setpoint.position.x = end_pt_(0);
                pos_setpoint.position.y = end_pt_(1);
                pos_setpoint.position.z = end_pt_(2);
                pos_setpoint.yaw = odom_yaw_;
            }
            else {
                cout<< "FAR, land at: " << end_pt_.transpose() << endl;

                pos_setpoint.position.x = end_pt_(0);
                pos_setpoint.position.y = end_pt_(1);
                pos_setpoint.position.z = end_pt_(2);
                pos_setpoint.yaw = odom_yaw_;
            }
            break;
        }
    }

    pos_setpoint.header.stamp = ros::Time::now();
    setpoint_raw_local_pub.publish(pos_setpoint);
}

void FSM::yoloCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg){
    cout << "------------------ YOLO ------------------" << endl;

    //监听包装在一个try-catch块中以捕获可能的异常
    try{
        //向侦听器查询特定的转换，(想得到/world到/monocular的变换)，想要转换的时间ros::Time(0)提供了最新的可用转换。
        tf_listener_.lookupTransform("world", "monocular_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("%s",ex.what());
    }

    switch (exec_state_) {
        case THROW: {
            // TODO:

            break;
        }

        case LAND: {
            double ratio_ = 1.0;

            for (int cnt_ = 0; cnt_ < (msg->bounding_boxes).size(); cnt_++) {
                darknet_ros_msgs::BoundingBox boundingBox = msg->bounding_boxes[cnt_];

                if (boundingBox.Class == "landing_pad") {
                    Eigen::Vector2d center_pixel_pad_((boundingBox.xmin + boundingBox.xmax)/2,
                                                      (boundingBox.ymin + boundingBox.ymax)/2);
                    Eigen::Vector2d range_pixel_pad_(boundingBox.xmax - boundingBox.xmin,
                                                     boundingBox.ymax - boundingBox.ymin);

                    if (cnt_ == 0){
                        centerErrorPad_ << center_pixel_pad_(0) - cx,
                                           center_pixel_pad_(1) - cy;

                        ratio_ = abs(range_pixel_pad_(0) - range_pixel_pad_(1))/double(range_pixel_pad_(0) + range_pixel_pad_(1));

                        cout << "INIT   centerErrorPad: " << centerErrorPad_.transpose() << ", ratio: " << ratio_ << endl;
                    }
                    else{
                        Eigen::Vector2d center_pad_temp(center_pixel_pad_(0) - cx,
                                                        center_pixel_pad_(1) - cy);

                        double ratio_temp = abs(range_pixel_pad_(0) - range_pixel_pad_(1))/double(range_pixel_pad_(0) + range_pixel_pad_(1));

                        if (ratio_temp < ratio_){
                            centerErrorPad_ = center_pad_temp;
                            ratio_ = ratio_temp;

                            cout << "BETTER centerErrorPad[" << cnt_ << "]: " << centerErrorPad_.transpose() << ", ratio: " << ratio_ << endl;
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

            cout << "FINAL  centerErrorPad: " << centerErrorPad_.transpose() << ", ratio: " << ratio_ << endl;

            // Convert the pixel frame point to world frame
            pose_pad_vec_ = transform * tf::Vector3(centerErrorPad_(0) / cx * odom_pos_(2),
                                                centerErrorPad_(1) / cy * odom_pos_(2),
                                                0.0);
            pose_pad_ << pose_pad_vec_.x()-0.1, pose_pad_.y(), 0.0;

            cout << "pose_pad: " << pose_pad_.transpose() << endl;

            break;
        }

        default:{
            return;
        }
    }
}

void FSM::init(ros::NodeHandle &nh){
    exec_timer_ = nh.createTimer
            (ros::Duration(0.01), &FSM::execFSMCallback, this);

    state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, &FSM::state_cb, this);
    odom_sub_ = nh.subscribe
            ("/mavros/local_position/odom", 10, &FSM::odometryCallback, this);
    yolo_sub_ = nh.subscribe<darknet_ros_msgs::BoundingBoxes>
            ("/darknet_ros/bounding_boxes", 10, &FSM::yoloCallback, this);
    camera_info_sub_ = nh.subscribe
            ("/monocular/camera_info", 1, &FSM::cameraInfoCallback, this);
    depth_sub_ = nh.subscribe
            ("/camera/depth/image_raw", 1, &FSM::depthCallback, this);

    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10);

    // 【服务】解锁/上锁 本服务通过Mavros功能包 /plugins/command.cpp 实现
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    // 【服务】修改系统模式 本服务通过Mavros功能包 /plugins/command.cpp 实现
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    /******* init ********/
    //setprecision(n) 设显示小数精度为n位
    cout << setprecision(4);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    ROS_INFO("Waiting for FCU connection...");

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("FCU connected!");

    have_odom_ = false;

    end_pt_ << 0, 0, 0;

    /*
        uint16 type_mask
        uint16 IGNORE_PX = 1 # Position ignore flags
        uint16 IGNORE_PY = 2
        uint16 IGNORE_PZ = 4
        uint16 IGNORE_VX = 8 # Velocity vector ignore flags
        uint16 IGNORE_VY = 16
        uint16 IGNORE_VZ = 32
        uint16 IGNORE_AFX = 64 # Acceleration/Force vector ignore flags
        uint16 IGNORE_AFY = 128
        uint16 IGNORE_AFZ = 256
        uint16 FORCE = 512 # Force in af vector flag
        uint16 IGNORE_YAW = 1024
        uint16 IGNORE_YAW_RATE = 2048
     */
    pos_setpoint.type_mask = 0b100111111000; // 100 111 111 000  xyz + yaw
    /*
        uint8 coordinate_frame
        uint8 FRAME_LOCAL_NED = 1
        uint8 FRAME_LOCAL_OFFSET_NED = 7
        uint8 FRAME_BODY_NED = 8
        uint8 FRAME_BODY_OFFSET_NED = 9
    */
    pos_setpoint.coordinate_frame = 1;
    pos_setpoint.position.x = odom_pos_(0);
    pos_setpoint.position.y = odom_pos_(1);
    pos_setpoint.position.z = odom_pos_(2);
    pos_setpoint.yaw = odom_yaw_;

    ROS_WARN("Send a few setpoints before starting...");

    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        setpoint_raw_local_pub.publish(pos_setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    offb_set_mode.request.custom_mode = "AUTO.LOITER";

    arm_cmd.request.value = true;

    throw_flag_ = false;
    land_flag_ = false;

    ROS_INFO("FSM initialized.");
};
