//
// Created by zeid on 2/27/20.
//
#include "robot_controller.h"

/**
 * Constructor for the robot
 * Class attributes are initialized in the constructor init list
 * You can instantiate another robot by passing the correct parameter to the constructor
 */
RobotController::RobotController(std::string arm_id) :
robot_controller_nh_("/ariac/"+arm_id),
robot_controller_options("manipulator",
        "/ariac/"+arm_id+"/robot_description",
        robot_controller_nh_),
robot_move_group_(robot_controller_options) {
    ROS_WARN(">>>>> RobotController");

    robot_move_group_.setPlanningTime(20);
    robot_move_group_.setNumPlanningAttempts(10);
    robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
    robot_move_group_.setMaxVelocityScalingFactor(0.8);
    robot_move_group_.setMaxAccelerationScalingFactor(0.8);
    // robot_move_group_.setEndEffector("moveit_ee");
    robot_move_group_.allowReplanning(true);


    //--These are joint positions used for the home position

    // home_joint_pose_ = {-0.15, 3.14, -2.43, -1.63, -0.63, 1.51, 0.88};  //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}
    home_joint_pose_ = {-1.0, 3.14, -2, 2.6, 3.9, 4.7, 0};    //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}   //bin position

    //home_joint_pose_ = {0.8, 3.52, -2.39, -1.76, -0.60, 1.55, 0.88};  
    //home_joint_pose_ = {0.0, 3.1, -1.1, 1.9, 3.9, 4.7, 0};
    //-- offset used for picking up parts
    //-- For the pulley_part, the offset is different since the pulley is thicker
    offset_ = 0.0231;

    //--topic used to get the status of the gripper
    gripper_subscriber_ = gripper_nh_.subscribe(
            "/ariac/"+arm_id+"/gripper/state", 10, &RobotController::GripperCallback, this);

    SendRobotHome();

    robot_tf_listener_.waitForTransform(arm_id+"_linear_arm_actuator", arm_id+"_ee_link",
                                            ros::Time(0), ros::Duration(10));
    robot_tf_listener_.lookupTransform("/"+arm_id+"_linear_arm_actuator", "/"+arm_id+"_ee_link",
                                           ros::Time(0), robot_tf_transform_);


    fixed_orientation_.x = robot_tf_transform_.getRotation().x();
    fixed_orientation_.y = robot_tf_transform_.getRotation().y();
    fixed_orientation_.z = robot_tf_transform_.getRotation().z();
    fixed_orientation_.w = robot_tf_transform_.getRotation().w();

    tf::quaternionMsgToTF(fixed_orientation_,q);
    tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);


    end_position_ = home_joint_pose_;
    end_position_ = {-1.1, 3.14, -2, 2.6, 3.9, 4.7, 0};   //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}
    // end_position_[0] 0.2;      
    // end_position_[1] = 4.5;
    // end_position_[2] = 1.2;


    robot_tf_listener_.waitForTransform("world", arm_id+"_ee_link", ros::Time(0),
                                            ros::Duration(10));
    robot_tf_listener_.lookupTransform("/world", "/"+arm_id+"_ee_link", ros::Time(0),
                                           robot_tf_transform_);

    home_cart_pose_.position.x = robot_tf_transform_.getOrigin().x();
    home_cart_pose_.position.y = robot_tf_transform_.getOrigin().y();
    home_cart_pose_.position.z = robot_tf_transform_.getOrigin().z();
    home_cart_pose_.orientation.x = robot_tf_transform_.getRotation().x();
    home_cart_pose_.orientation.y = robot_tf_transform_.getRotation().y();
    home_cart_pose_.orientation.z = robot_tf_transform_.getRotation().z();
    home_cart_pose_.orientation.w = robot_tf_transform_.getRotation().w();

    agv_tf_listener_.waitForTransform("world", "kit_tray_1",
                                      ros::Time(0), ros::Duration(10));
    agv_tf_listener_.lookupTransform("/world", "/kit_tray_1",
                                     ros::Time(0), agv_tf_transform_);
    agv_position_.position.x = agv_tf_transform_.getOrigin().x();
    agv_position_.position.y = agv_tf_transform_.getOrigin().y();
    agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_;

    gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>(
            "/ariac/arm1/gripper/control");
    counter_ = 0;
    drop_flag_ = false;
}

RobotController::~RobotController() {}

/**
 *
 * @return
 */
bool RobotController::Planner() {
    ROS_INFO_STREAM("Planning started...");
    if (robot_move_group_.plan(robot_planner_) ==
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        plan_success_ = true;
        ROS_INFO_STREAM("Planner succeeded!");
    } else {
        plan_success_ = false;
        ROS_WARN_STREAM("Planner failed!");
    }

    return plan_success_;
}


void RobotController::Execute() {
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.0).sleep();
    }
}

void RobotController::GoToTarget(const geometry_msgs::Pose& pose) {
    ROS_INFO_STREAM("Going to target!");
    target_pose_.orientation = fixed_orientation_;
    target_pose_.position = pose.position;
    ros::AsyncSpinner spinner(4);
    robot_move_group_.setPoseTarget(target_pose_);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.1).sleep();
    }
    else {
        ROS_INFO_STREAM("Retrying...Moving closer");
        auto temp_joint = home_joint_pose_;
        temp_joint = {-1.0, 3.14, -2, 2.6, 3.9, 4.7, 0};
        // if(pose.position.y < home_joint_pose_[0]) temp_joint[0] = -5;
        // else temp_joint[0] +=1;
        // auto temp_pose = pose;
        // temp_pose.position.x = pose.pose.x/2;
        // temp_pose.position.y = pose.pose.y/2;
        
        this->SendRobotToJointValues(temp_joint);
        robot_move_group_.move();
        ros::Duration(5).sleep();
    }
    ROS_INFO_STREAM("Point reached...");
}

void RobotController::GoToTarget(const geometry_msgs::Pose& pose, float delay) {
    ROS_INFO_STREAM("Delayed start...");
    ros::Duration(2.48).sleep();
    ROS_INFO_STREAM("Going to target!");
    target_pose_.orientation = fixed_orientation_;
    target_pose_.position = pose.position;
    ros::AsyncSpinner spinner(4);
    robot_move_group_.setPoseTarget(target_pose_);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO_STREAM("Point reached...");
}

void RobotController::GoToTarget(
        std::initializer_list<geometry_msgs::Pose> list) {
    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::vector<geometry_msgs::Pose> waypoints;
    for (auto i : list) {
        i.orientation.x = fixed_orientation_.x;
        i.orientation.y = fixed_orientation_.y;
        i.orientation.z = fixed_orientation_.z;
        i.orientation.w = fixed_orientation_.w;
        waypoints.emplace_back(i);
    }

    moveit_msgs::RobotTrajectory traj;
    auto fraction =
            robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

    ROS_WARN_STREAM("Fraction: " << fraction * 100);
    ros::Duration(2.0).sleep();

    robot_planner_.trajectory_ = traj;

    //if (fraction >= 0.3) {
    // ROS_INFO_STREAM("Delay");
    robot_move_group_.execute(robot_planner_);
    ros::Duration(3.0).sleep();
//    } else {
//        ROS_ERROR_STREAM("Safe Trajectory not found!");
//    }
}

void RobotController::SendRobotHome() {
    // ros::Duration(2.0).sleep();
    robot_move_group_.setJointValueTarget(home_joint_pose_);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(3.0).sleep();
    }

     ros::Duration(2.0).sleep();
     
}

void RobotController::SendRobotEnd() {
    robot_move_group_.setJointValueTarget(end_position_);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(3.0).sleep();
    }

     ros::Duration(2.0).sleep();

}


void RobotController::SendRobotToJointValues(std::vector<double> joint_values) {
    robot_move_group_.setJointValueTarget(joint_values);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        ROS_INFO_STREAM("Going to joint value...");
        robot_move_group_.move();
        ros::Duration(4.0).sleep();
    }  
}

void RobotController::GripperToggle(const bool& state) {
    gripper_service_.request.enable = state;
    gripper_client_.call(gripper_service_);
    ros::Duration(1.0).sleep();
    // if (gripper_client_.call(gripper_service_)) {
    if (gripper_service_.response.success) {
        ROS_INFO_STREAM("Gripper activated!");
    } else {
        ROS_WARN_STREAM("Gripper activation failed!");
    }
}

// bool RobotController::dropPart(geometry_msgs::Pose part_pose) {
//   counter_++;
//
//   pick = false;
//   drop = true;
//
//   ROS_WARN_STREAM("Dropping the part number: " << counter_);
//
//   // ROS_INFO_STREAM("Moving to end of conveyor...");
//   // robot_move_group_.setJointValueTarget(part_pose);
//   // this->execute();
//   // ros::Duration(1.0).sleep();
//   // this->gripper_state_check(part_pose);
//
//   if (drop == false) {
//     // ROS_INFO_STREAM("I am stuck here..." << object);
//     ros::Duration(2.0).sleep();
//     return drop;
//   }
//   ROS_INFO_STREAM("Dropping on AGV...");
//
//   // agv_position_.position.x -= 0.1;
//   // if (counter_ == 1) {
//   //   agv_position_.position.y -= 0.1;
//   // }
//   // if (counter_ >= 2) {
//   //   agv_position_.position.y += 0.1;
//   //   // agv_position_.position.x +=0.1;
//   // }
//
//   auto temp_pose = part_pose;
//   // auto temp_pose = agv_position_;
//   temp_pose.position.z += 0.35;
//   // temp_pose.position.y += 0.5;
//
//   // this->setTarget(part_pose);
//   // this->execute();
//   // ros::Duration(1.0).sleep();
//   this->goToTarget({temp_pose, part_pose});
//   ros::Duration(1).sleep();
//   ROS_INFO_STREAM("Actuating the gripper...");
//   this->gripperToggle(false);
//
//   // ROS_INFO_STREAM("Moving to end of conveyor...");
//   // robot_move_group_.setJointValueTarget(end_position_);
//   // this->execute();
//   // ros::Duration(1.0).sleep();
//
//   ROS_INFO_STREAM("Going to home...");
//   // this->sendRobotHome();
//   // temp_pose = home_cart_pose_;
//   // temp_pose.position.z -= 0.05;
//   this->goToTarget({temp_pose, home_cart_pose_});
//   return drop;
// }

bool RobotController::DropPart(geometry_msgs::Pose part_pose) {
    // counter_++;

    drop_flag_ = true;
    float delay=1.8;
    ros::spinOnce();
    ROS_INFO_STREAM("Placing phase activated...");

    if (gripper_state_){//--while the part is still attached to the gripper
        //--move the robot to the end of the rail
        ROS_INFO_STREAM("Moving towards AGV1...");
        auto AGV1_drop_position = home_joint_pose_;
        auto bin_position_ = home_joint_pose_;
        
        bin_position_ = {0, 3.14, -2, 2.6, 3.9, 4.7, 0};    //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}
        AGV1_drop_position = {1.00, 1.13, -1.34, 2.0, 4.12, 4.67, -4.27};    //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}

        this->SendRobotToJointValues(AGV1_drop_position);
        this->GoToTarget(part_pose);

        // this->GoToTarget(part_pose);
        ros::Duration(2.0).sleep();
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(false);
        ros::spinOnce();


       if (!gripper_state_) {
            ROS_INFO_STREAM("Going to bin position...");
            // this->SendRobotToJointValues(bin_position_);
            this->SendRobotHome();
       }
    }

    drop_flag_ = false;
    return gripper_state_;
}



bool RobotController::DiscardPart(geometry_msgs::Pose part_pose) {
    // Write for AGV2 
    ROS_INFO_STREAM("Discarding part...");
    bool discard_flag = false;
    if(gripper_state_) {
        ROS_INFO_STREAM("Moving towards AGV...");
        auto AGV1_drop_position = home_joint_pose_;
        AGV1_drop_position = {1.00, 1.13, -1.34, 2.0, 4.12, 4.67, -4.27};    //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}
        this->SendRobotToJointValues(AGV1_drop_position);
        this->GoToTarget(part_pose);
        ros::Duration(2.0).sleep();
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(true);
        ros::spinOnce();
        auto drop_pose = part_pose;
        drop_pose.position.x -= 1;
        drop_pose.position.y -= 1;
        this->GoToTarget(drop_pose);
        ros::Duration(2.0).sleep();
        this->GripperToggle(false);
        ros::spinOnce();
        discard_flag = true;

    }

    ROS_WARN_STREAM("Discarded Part!");
    return discard_flag;
}

void RobotController::GripperCallback(
        const osrf_gear::VacuumGripperState::ConstPtr& grip) {
    gripper_state_ = grip->attached;
}


bool RobotController::PickPart(geometry_msgs::Pose& part_pose, std::string product_type) {
    float delay=1.8;
    ROS_INFO_STREAM("Moving to part...");

    // if(product_type!="gear_part")
    // {
    //     part_pose.position.z += offset_;
    //     part_pose.position.y -= 0.8;
    // }
    // else{
    //     part_pose.position.z += 0.0185;
    // }

    part_pose.position.z += 0.005;
    
    // auto bin_position_ = home_joint_pose_;
    // bin_position_ = {0, 3.14, -2, 2.6, 3.9, 4.7, 0};    //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}
    auto temp_pose_1 = part_pose;
    // temp_pose_1.position.z += 0.75;
    ROS_INFO_STREAM("Actuating the gripper!");
    this->GripperToggle(true);
    ROS_INFO_STREAM("Going to waypoint...");
    
    // std::cout << "Part pose" << part_pose.position.y;
    // if(part_pose.position.y < -0.5) {
    //     bin_position_[0] -= 2;
    //     std::cout << "Here";
    // }
    // else if(part_pose.position.y > 1.0) bin_position_[0] += 1;
    // this->SendRobotToJointValues(bin_position_);
    std::vector<double> bin_position_ = {-1, 3.14, -2, 2.6, 3.9, 4.7, 0};    //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}
    this->SendRobotToJointValues(bin_position_);
    // this->SendRobotEnd();
    // ros::Duration(2.0).sleep();
    // std::cout << "Yes??" ;
    this->GoToTarget(part_pose);
    // if(product_type=="piston_rod_part") this->GoToTarget(part_pose, delay);    // For conveyor belt pick-ups
    // else this->GoToTarget(part_pose);
    ros::spinOnce();
    while (!gripper_state_) {
        part_pose.position.z -= 0.01;
        temp_pose_1.position.z += 0.2;
        this->GoToTarget({temp_pose_1, part_pose});
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(true);
        ros::spinOnce();
    }
    ros::Duration(1.0).sleep();
    this->SendRobotHome();
    ROS_INFO_STREAM("Moving to home position with part!");

    // if(product_type=="piston_rod_part") this->SendRobotToJointValues(home_joint_pose_);
    // else this->SendRobotToJointValues(bin_position_);

    // this->GoToTarget(temp_pose_1);
    ros::Duration(1.5).sleep();



    return gripper_state_;
}