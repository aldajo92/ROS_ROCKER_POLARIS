// #include <ros/ros.h>
// #include <nav_msgs/Path.h>
// #include <nav_msgs/Odometry.h>
// #include "pt_controller/planner_path_tracking.h"

// class PathTrackingNode
// {
// public:
//     PathTrackingNode(double lookahead_distance, double max_linear_speed, double wheelbase)
//         : planner_(lookahead_distance, max_linear_speed, wheelbase)
//     {
//         path_sub_ = nh_.subscribe("/gps_path", 10, &PathTrackingNode::pathCallback, this);
//         odom_sub_ = nh_.subscribe("/gem/base_footprint/odom", 10, &PathTrackingNode::odomCallback, this);

//         // ROS_INFO("lookahead_distance: %.2f", lookahead_distance);
//         // ROS_INFO("max_linear_speed: %.2f", max_linear_speed);
//         // ROS_INFO("wheelbase: %.2f", wheelbase);
//     }

//     void pathCallback(const nav_msgs::Path::ConstPtr &msg)
//     {
//         // ROS_INFO("Path received with %ld waypoints.", msg->poses.size());
//         planner_.setPath(*msg);
//     }

//     void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
//     {
//         // ROS_INFO("Odometry received with position (x=%.2f, y=%.2f)", msg->pose.pose.position.x, msg->pose.pose.position.y);
//         planner_.updateOdometry(*msg);
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber path_sub_;
//     ros::Subscriber odom_sub_;
//     PlannerPathTracking planner_;
// };

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "path_tracking_node");

//     // Parameters
//     double lookahead_distance = 2.0;
//     double max_linear_speed = 2.0;
//     double wheelbase = 1.75;

//     PathTrackingNode node(lookahead_distance, max_linear_speed, wheelbase);
//     ros::spin();
//     return 0;
// }

#include <ros/ros.h>
#include "pt_controller/path_tracking_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_tracking_controller");

    // Parameters
    double lookahead_distance = 2.0;
    double max_linear_speed = 2.0;
    double wheelbase = 1.75;

    PathTrackingController controller(lookahead_distance, max_linear_speed, wheelbase);
    ros::spin();
    return 0;
}
