//
// Created by zeid on 2/27/20.
//

#include "order_manager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <algorithm>


AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
// AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}
{
    order_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/orders", 10,
            &AriacOrderManager::OrderCallback, this);
}

AriacOrderManager::~AriacOrderManager(){}


void AriacOrderManager::OrderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
    ROS_WARN(">>>>> OrderCallback");
    received_orders_.push_back(*order_msg);
    // CheckOrderUpdate(*order_msg);
}


bool AriacOrderManager::CheckOrderUpdate() {
    if(received_orders_.size() < 2) return true;

    ROS_WARN_STREAM("Order update found!!");
    ROS_INFO_STREAM("Updating order...");

    auto prev__products = received_orders_[0].shipments[0].products;
    auto updated_products = received_orders_[1].shipments[0].products;

    // for(auto old_order: prev__products) {
    //     ROS_INFO_STREAM("Old :" << old_order);
    // }

    // for(auto new_order: updated_products) {
    //     ROS_INFO_STREAM("New :" << new_order);
    // }

    auto i = updated_products.begin();
    for(auto old_order = prev__products.begin(); old_order<prev__products.end(); old_order++) {
        bool discard = true, order_success = false;
        for(auto new_order = updated_products.begin(); new_order<updated_products.end(); new_order++) {
            order_success=false;
            discard=true;
            if(old_order->pose.position.x == new_order->pose.position.x && old_order->pose.position.y == new_order->pose.position.y 
                    && old_order->pose.position.z == new_order->pose.position.z) {
                if(old_order->type == new_order->type) {
                    ROS_INFO_STREAM("Same part & pose found, no change for a " << new_order->type << " with pose: " << new_order->pose);
                }
                else {
                    ROS_INFO_STREAM("Part " << old_order->type << " needs to be replaced by " << new_order->type << " at " << new_order->pose);
                }
                discard = false;
                i=new_order;
                order_success=true;
                break;
            }
        }

        for(auto new_order = updated_products.begin(); new_order<updated_products.end(); new_order++) {
            // discard = true; 
            // order_success = false;
            if(!order_success) {
                if(old_order->type == new_order->type) {
                    if(old_order->pose.position.x != new_order->pose.position.x || old_order->pose.position.y != new_order->pose.position.y 
                    || old_order->pose.position.z != new_order->pose.position.z) {
                        ROS_INFO_STREAM("Same part but different pose found for a " << new_order->type << " with pose : " << new_order->pose << " and not pose: " << old_order->pose);
                        discard = false;    
                        order_success = true;
                        i=new_order;
                        break;
                    }
                }
            }
        }
        if(order_success) updated_products.erase(i);
        if(discard) {
            ROS_WARN_STREAM("Part " << old_order->type << " not found in the new order, discarding...");    
        }

    }

    ROS_INFO_STREAM("Orders yet to be fulfiled are: ");
    for(auto new_order: updated_products) {
        ROS_INFO_STREAM("Part " << new_order.type << " at " << new_order.pose);
    }

    return true;
}

/**
 * @brief Get the product frame for a product type
 * @param product_type
 * @return
 */
std::string AriacOrderManager::GetProductFrame(std::string product_type) {
    //--Grab the last one from the list then remove it
    std::string product_frame;

    label:
    if (!product_frame_list_.empty()) {
        product_frame = product_frame_list_[product_type].back();
        product_frame_list_[product_type].pop_back();
        return product_frame;
        
    } else {
        ROS_WARN_STREAM("No product frame found for " << product_type);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        product_frame_list_ = camera_.get_product_frame_list();
        ROS_WARN_STREAM("SIZE: " << product_frame_list_.size());
        goto label;
        //this->GetProductFrame(product_type);
        
    }
    // ROS_INFO_STREAM("Frame >>>> " << product_frame);
    
}

bool AriacOrderManager::PickPartExchange(geometry_msgs::Pose part_pose, std::string product_type, std::string arm1, std::string arm2) {
    bool failed_pick;
    geometry_msgs::Pose exchange_pose;
    exchange_pose.position.x = 0.27;
    exchange_pose.position.y = 0;
    exchange_pose.position.z = 1.0;

    if(arm1 == "arm1")    {
        failed_pick = arm2_.PickPart(part_pose, product_type);
        std::vector<double> exchange_joint_pose = {0, 1.5, -1.25, 2.4, 3.6, -1.51, 0};    //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}
        arm2_.SendRobotToJointValues(exchange_joint_pose);
        arm2_.DropPartExchange(exchange_pose);
        exchange_joint_pose = {-0.25, 4.6, -1.25, 2.4, 3.6, -1.51, 0};    //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}
        arm1_.SendRobotToJointValues(exchange_joint_pose);
        exchange_pose.position.z -= 0.05;
        failed_pick = arm1_.PickPart(exchange_pose, product_type);
    }
    else {
        failed_pick = arm1_.PickPart(part_pose, product_type);
        std::vector<double> exchange_joint_pose = {0, 4.6, -1.25, 2.4, 3.6, -1.51, 0};    //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}
        arm1_.SendRobotToJointValues(exchange_joint_pose);
        arm1_.DropPartExchange(exchange_pose);
        exchange_joint_pose = {0.75, 1.5, -1.25, 2.4, 3.6, -1.51, 0};    //{lin_arm, shoulder_pan, shoulder_lift, elbow, w1, w2, w3}
        arm2_.SendRobotToJointValues(exchange_joint_pose);
        exchange_pose.position.z -= 0.05;
        failed_pick = arm2_.PickPart(exchange_pose, product_type);   
    }
    return failed_pick;
}



bool AriacOrderManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id) {
    std::string product_type = product_type_pose.first;
    ROS_WARN_STREAM("Product type >>>> " << product_type);
    //ROS_INFO_STREAM("Prod frame >>>> " << this->GetProductFrame(product_type));
    std::string product_frame = this->GetProductFrame(product_type);
    ROS_WARN_STREAM("Product frame >>>> " << product_frame);
    auto part_pose = camera_.GetPartPose("world",product_frame);
    //ROS_WARN_STREAM("Part pose obtained >>>> " << part_pose);
   

    if(product_type == "pulley_part")
        part_pose.position.z += 0.08;
    //--task the robot to pick up this part

    if(agv_id==1) {
        if(part_pose.position.y > 0) {
            ROS_WARN_STREAM("Arm 1 picking up the part...");
            bool failed_pick = arm1_.PickPart(part_pose,product_type);
        }
        else {
            ROS_WARN_STREAM("Arm 2 picking the part and exchaning....");
            bool failed_pick = PickPartExchange(part_pose, product_type, "arm1", "arm2");
        }
    }
    else {
        if(part_pose.position.y < 0) {
            ROS_WARN_STREAM("Arm 2 picking up the part...");
            bool failed_pick = arm2_.PickPart(part_pose, product_type);
        }
        else {
            ROS_WARN_STREAM("Arm 1 picking the part and exchaning...");
            bool failed_pick = PickPartExchange(part_pose, product_type, "arm2", "arm1");
        }
    }

    ros::Duration(0.5).sleep();

    // bool failed_pick = arm1_.PickPart(part_pose,product_type);
    // ROS_WARN_STREAM("Picking up state " << failed_pick);

    // while(!failed_pick){
    //     auto part_pose = camera_.GetPartPose("/world",product_frame);
    //     ROS_INFO_STREAM("Picking up...");
    //     failed_pick = arm1_.PickPart(part_pose,product_type);
    // }

    //--get the pose of the object in the tray from the order
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

    if(agv_id==1){
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = drop_pose;
        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        // StampedPose_out.pose.position.y -= 0.1;
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

    }
    else{
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = drop_pose;
        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        StampedPose_out.pose.position.y += 0.2;
        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
    }
    auto result = arm1_.DropPart(StampedPose_out.pose);
    bool quality_check = camera_.CheckQualityControl(agv_id, StampedPose_out.pose);
    ROS_WARN_STREAM("Dropped a part on AGV tray!");

    // if(quality_check) {
    //     arm1_.DiscardPart(StampedPose_out.pose);
    //     return quality_check;
    // }

    //ros::shutdown();

    return result;
}


void AriacOrderManager::ExecuteOrder() {
    // ROS_WARN(">>>>>> Executing order...");
    //scanned_objects_ = camera_.GetParts();

    //-- used to check if pick and place was successful
    bool pick_n_place_success{false};

    std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;

    ros::spinOnce();
    ros::Duration(1.0).sleep();
    product_frame_list_ = camera_.get_product_frame_list();
    
    for (const auto &order:received_orders_){
        // ROS_INFO_STREAM("Here 1");   
        auto order_id = order.order_id;
        auto shipments = order.shipments;
        for (const auto &shipment: shipments){
            auto shipment_type = shipment.shipment_type;
            auto agv = shipment.agv_id.back();//--this returns a char
            //-- if agv is any then we use AGV1, else we convert agv id to int
            //--agv-'0' converts '1' to 1 and '2' to 2
            int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';
            // ROS_INFO_STREAM("Here 2");
            auto products = shipment.products;
            ROS_INFO_STREAM("Order ID: " << order_id);
            ROS_INFO_STREAM("Shipment Type: " << shipment_type);
            ROS_INFO_STREAM("AGV ID: " << agv_id);
            int i =0;
            for (const auto &product: products){
                ros::spinOnce();
                // ROS_INFO_STREAM("Here 3");
                product_type_pose_.first = product.type;
                //ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
                product_type_pose_.second = product.pose;
                // ROS_INFO_STREAM("Order Count: " << i);
                ROS_INFO_STREAM("Product pose: " << product_type_pose_.second.position.x);
                // do {
                //     pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id);    
                // }
                // while(pick_n_place_success);               // The condition is reversed
                pick_n_place_success = PickAndPlace(product_type_pose_, agv_id);
                i++;
            }
            SubmitAGV(agv_id);
            ROS_INFO_STREAM("Submitting AGV 1");
            ros::Duration(2.0).sleep();
            ros::spinOnce();
            bool update_check_success{false};

            do {
                update_check_success =  CheckOrderUpdate();
            } 
            while(!update_check_success);
            int finish=1;
            ROS_WARN("KIT COMPLETE");
            ros::shutdown();
        }

    }
}

void AriacOrderManager::SubmitAGV(int num) {
    std::string s = std::to_string(num);
    ros::ServiceClient start_client =
            order_manager_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the client to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Service started.");
    }

    osrf_gear::AGVControl srv;
    // srv.request.kit_type = "order_0_kit_0";
    start_client.call(srv);

    if (!srv.response.success) {
        ROS_ERROR_STREAM("Service failed!");
    } else
        ROS_INFO("Service succeeded.");
}