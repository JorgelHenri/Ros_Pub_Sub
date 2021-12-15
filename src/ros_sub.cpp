#include <ros_sub.h>
#include <pluginlib/class_list_macros.h>

namespace ros_sub
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
    shopts.node_name          = "Ros_Sub";
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

    ros_pub_sub::DistTarget dist = sh.getMsg();

    pose_x = dist->x;
    pose_y = dist->x;
    pose_z = dist->x;
    distance = dist->dist;
    ROS_INFO("[Ros_Sub]: Distance to target: %.2f", dist);
    ROS_INFO("[Ros_Sub]: Target pose: x:%.2f y:%.2f z:%.2f", pose_x, pose_y, pose_z);

}

}  // namespace ros_pub

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ros_sub::Ros_Sub, nodelet::Nodelet);