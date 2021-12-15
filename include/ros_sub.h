#pragma once
#ifndef ROS_SUB_H
#define ROS_SUB_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include "DistTarget.h"

#include <mrs_lib/param_loader.h>

#include <mrs_lib/subscribe_handler.h>

namespace ros_sub
{

class Ros_Sub : public nodelet::nodelet{
public:
    virtual void onInit();

private:
    /* Parameters */
    std::string                                         _uav_name_;

    // | ------------------------ subscriber callbacks --------------------------- |

    mrs_lib::SubscribeHandler<ros_pub_sub::DistTarget>              sh_dist_target_;

    // | ------------------------ timers --------------------------- |

    void           callbackTimerPublishDistToWaypoint(const ros::TimerEvent& te);
    ros::Timer     timer_publish_dist_to_waypoint_;
};

} // namespace ros_pub

#endif