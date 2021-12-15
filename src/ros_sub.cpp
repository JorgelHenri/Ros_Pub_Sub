#include <ros_sub.h>
#include <pluginlib/class_list_macros.h>

namespace ros_sub{

    /*onInit() */
    void Ros_Sub::onInit(){

        ros::NodeHandle nh("~");
    
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

        sh_dist_target_             = mrs_lib::SubscribeHandler<ros_pub_sub::DistTarget>(shopts, "dist_to_target_in");

        // | -------------------- initialize timers ------------------- |

        timer_publish_dist_to_waypoint_ = nh.createTimer(ros::Rate(10), &Ros_Sub::callbackTimerPublishDistToWaypoint, this);
        
        ROS_INFO_ONCE("[WaypointFlier]: initialized");

        is_initialized_ = true;

        ros::spin();

    }

    // | --------------------- timer callbacks -------------------- |

    /* callbackTimerPublishDistToWaypoint() //{ */

    void Ros_Sub::callbackTimerPublishDistToWaypoint([[maybe_unused]] const ros::TimerEvent& te) {
        if ((!is_initialized_) || (!sh_dist_target_.hasMsg())) {
            return;
        }

        ros_pub_sub::DistTargetConstPtr dist = sh_dist_target_.getMsg();

        double pose_x = dist->x;
        double pose_y = dist->y;
        double pose_z = dist->z;
        double distance = dist->dist;
        ROS_INFO("[Ros_Sub]: Distance to target: %.2f", distance);
        ROS_INFO("[Ros_Sub]: Target pose: x:%.2f y:%.2f z:%.2f", pose_x, pose_y, pose_z);

    }

}  // namespace ros_sub

/* every nodelet must include macros which export the class as a nodelet plugin */
PLUGINLIB_EXPORT_CLASS(ros_sub::Ros_Sub, nodelet::Nodelet);