#pragma once

#include <list>
#include <map>
#include <string>
#include <iostream>
#include <deque>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "sensor.h"
#include "robot_controller.h"

class AriacOrderManager {
public:
    AriacOrderManager();
    ~AriacOrderManager();
    void OrderCallback(const osrf_gear::Order::ConstPtr& order_msg);
    bool CheckOrderUpdate(int count, int agv_id);
    void ExecuteOrder();
    std::string GetProductFrame(std::string product_type);
    bool CheckIfPartNeeded(osrf_gear::Product part,std::vector<osrf_gear::Product> product_list);
    bool CanPlaceOnAGV(osrf_gear::Product part, std::deque <osrf_gear::Product> kit);
    osrf_gear::Product GetUpdatedPose(osrf_gear::Product part, std::vector<osrf_gear::Product> product_list);
    bool DiscardPartOnAGV(geometry_msgs::Pose part_pose, int agv_id);
    bool PickPartAtAGV(osrf_gear::Product final_product,geometry_msgs::Pose pickup, int agv_id);
    std::vector<osrf_gear::Product> RemovePartFromList(osrf_gear::Product part, std::vector<osrf_gear::Product> product_list);
    std::map<std::string, std::list<std::pair<std::string,geometry_msgs::Pose>>> GetOrder();
    bool PickAndPlace(std::pair<std::string,geometry_msgs::Pose> object_prop,int agvnum);
    bool PickPartExchange(geometry_msgs::Pose part_pose, std::string product_type, std::string arm1, std::string arm2);
    void SubmitAGV(int num);

private:
    ros::NodeHandle order_manager_nh_;
    ros::Subscriber order_subscriber_;
    std::vector<osrf_gear::Order> received_orders_;
    AriacSensorManager camera_;
    RobotController arm1_, arm2_;
//    RobotController arm2_;
    tf::TransformListener part_tf_listener_;
    std::pair<std::string,geometry_msgs::Pose> product_type_pose_;
    std::string object;
    std::map<std::string, std::vector<std::string>> product_frame_list_;
    osrf_gear::Order order_;
};

