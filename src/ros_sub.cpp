#include <ros_pub.h>
#include <pluginlib/class_list_macros.h>

namespace ros_pub
{
    /*onInit() */
void Ros_Sub::onInit(){
    
    /* waits for the ROS to publish clock */
    ros::Time::waitForValid();

    mrs_lib::ParamLoader param_loader(nh, "Ros_Sub");

    param_loader.loadParam("uav_name", _uav_name_);

    if (!param_loader.loadedSuccessfully()) {
        ROS_ERROR("[WaypointFlier]: failed to load non-optional parameters!");
        ros::shutdown();
    }

    // | ------------------ initialize subscribers ----------------- |

    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh;
    shopts.node_name          = "Ros_Pub";
    shopts.no_message_timeout = ros::Duration(1.0);
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sh_odometry_             = mrs_lib::SubscribeHandler<ros_pub_sub::DistTarget(shopts, "dist_to_target_in");

    // | -------------------- initialize timers ------------------- |

    timer_publish_dist_to_waypoint_ = nh.createTimer(ros::Rate(10), &WaypointFlier::callbackTimerPublishDistToWaypoint, this);
    
    ROS_INFO_ONCE("[WaypointFlier]: initialized");

    is_initialized_ = true;
}

    // | --------------------- timer callbacks -------------------- |

/* callbackTimerPublishDistToWaypoint() //{ */

void WaypointFlier::callbackTimerPublishDistToWaypoint([[maybe_unused]] const ros::TimerEvent& te) {
    if (!is_initialized_) {
        return;
    }

    target_dist = mrs_lib::getPose(sh_odometry_.getMsg());

}
}