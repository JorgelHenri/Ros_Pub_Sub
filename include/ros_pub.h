#pragma once
#ifndef ROS_PUB_H
#define ROS_PUB_H

/**
 *INCLUDE 
 */

/* each ROS nodelet must have these */
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

/** MRS INCLUDE **/

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/geometry/misc.h>

/** Mensagens Include **/
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/Float64Stamped.h>
#include "DistTarget.h"


namespace ros_pub{
    class Ros_Pub:public nodelet::Nodelet{
        public:
            virtual void onInit();
        private:
            /** Topicos UAV **/
            mrs_lib::SubscribeHandler <nav_msgs::Odometry> sh_odometry_;
            ros::Publisher uav_dist_to_waypoint_publisher_;
            
            /** Funcion Callback **/
            void dist_to_waypoint_callback(const ros::TimerEvent& te);
            
            /** Funcion **/
            double distace(geometry_msgs::Pose current_pose, geometry_msgs::Pose target_pose);

            /* Timer Ros */
            ros::Timer timer_publish_dist_to_waypoint_;

            /* Variable Drone */ 
            geometry_msgs::Pose current_pose;
            geometry_msgs::Pose target_pose;
            std::string _uav_name_;
            bool is_initialized_ = false;
    };
}
#endif