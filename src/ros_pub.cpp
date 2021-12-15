#include <ros_pub.h>

namespace ros_pub{
    void Ros_Pub::onInit(){
        ros::NodeHandle nh("~");
        ros::Time::waitForValid();

        mrs_lib::ParamLoader param_loader(nh, "ros_pub");

        param_loader.loadParam("uav_name", _uav_name_);
        param_loader.loadParam("target_x", target_pose.position.x);
        param_loader.loadParam("target_y", target_pose.position.y);
        param_loader.loadParam("target_z", target_pose.position.z);

        if (!param_loader.loadedSuccessfully()) {
            ROS_ERROR("[Ros_Pub]: failed to load non-optional parameters!");
            ros::shutdown();
        } 

        mrs_lib::SubscribeHandlerOptions shopts;

        shopts.nh                 = nh;
        shopts.node_name          = "Ros_Pub";
        shopts.no_message_timeout = ros::Duration(1.0);
        shopts.threadsafe         = true;
        shopts.autostart          = true;
        shopts.queue_size         = 10;
        shopts.transport_hints    = ros::TransportHints().tcpNoDelay();
                
        sh_odometry_              = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_uav_in");

        uav_dist_to_waypoint_publisher_ = nh.advertise<ros_pub_sub::DistTarget>("dist_to_target_out", 1 );

        timer_publish_dist_to_waypoint_ = nh.createTimer(ros::Rate(10), &Ros_Pub::dist_to_waypoint_callback, this);

        is_initialized_ = true;
    }
    void Ros_Pub::dist_to_waypoint_callback([[maybe_unused]] const ros::TimerEvent& te){
        if ((!is_initialized_) || (!sh_odometry_.hasMsg())) {
            return;
        }

        current_pose = mrs_lib::getPose(sh_odometry_.getMsg());
        double dist = distace(current_pose, target_pose);

        ros_pub_sub::DistTarget dist_msg;

        dist_msg.x = target_pose.position.x;
        dist_msg.y = target_pose.position.y;
        dist_msg.z = target_pose.position.z;
        dist_msg.dist = dist;

        try {
            uav_dist_to_waypoint_publisher_.publish(dist_msg);
        }
        catch(...) {
            ROS_ERROR("Exception caught during publishing topic %s.", uav_dist_to_waypoint_publisher_.getTopic().c_str());
        }
    }

    double Ros_Pub::distace(geometry_msgs::Pose current_pose_in, geometry_msgs::Pose target_pose_in){
        return mrs_lib::geometry::dist( mrs_lib::geometry::vec_t<3> (
                                                                        current_pose_in.position.x,
                                                                        current_pose_in.position.y,
                                                                        current_pose_in.position.z
                                                                    ),
                                        mrs_lib::geometry::vec_t<3>(
                                                                        target_pose_in.position.x,
                                                                        target_pose_in.position.y,
                                                                        target_pose_in.position.z
                                                                    )
                                        );
    }

}
PLUGINLIB_EXPORT_CLASS(ros_pub::Ros_Pub, nodelet::Nodelet);