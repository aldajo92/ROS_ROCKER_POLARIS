#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "pt_controller/pathTrackingController.h"

class PathTrackingNode
{
public:
    PathTrackingNode(double lookahead_distance, double max_linear_speed, double wheelbase)
        : path_tracking_controller_(
            lookahead_distance, 
            max_linear_speed, 
            wheelbase, 
            std::bind(&PathTrackingNode::sendVelocitiesCallback, this, std::placeholders::_1, std::placeholders::_2))
    {
        path_sub_ = nh_.subscribe("/gps_path", 10, &PathTrackingNode::pathCallback, this);
        odom_sub_ = nh_.subscribe("/gem/base_footprint/odom", 10, &PathTrackingNode::odomCallback, this);

        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/gem/cmd_vel", 10);
        traveled_path_pub_ = nh_.advertise<nav_msgs::Path>("/traveled_path", 10);

        traveled_path_.header.frame_id = "base_footprint";
    }

    void pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        path_tracking_controller_.setPath(*msg);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        path_tracking_controller_.setOdom(*msg);
        path_tracking_controller_.computeControlCommand();
        updateTraveledPath(*msg);
    }

    void sendVelocitiesCallback(
        const double &linear_velocity,
        const double &angular_velocity)
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_velocity;
        cmd_vel.angular.z = angular_velocity;
        cmd_vel_pub_.publish(cmd_vel);
    }

    void updateTraveledPath(const nav_msgs::Odometry &odometry)
    {
        // Update traveled path
        geometry_msgs::PoseStamped traveled_pose;
        traveled_pose.header.stamp = ros::Time::now();
        traveled_pose.header.frame_id = "odom";
        traveled_pose.pose = odometry.pose.pose;
        traveled_path_.poses.push_back(traveled_pose);

        // Publish the traveled path
        traveled_path_.header.stamp = ros::Time::now();
        traveled_path_pub_.publish(traveled_path_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher traveled_path_pub_;
    nav_msgs::Odometry current_odometry_;
    nav_msgs::Path traveled_path_;

    PathTrackingController path_tracking_controller_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_tracking_node");
    ros::NodeHandle nh;

    // Parameters
    double lookahead_distance = 1.8;
    double max_linear_speed = 2.0;
    double wheelbase = 1.75;

    nh.param("lookahead_distance", lookahead_distance, 2.0);
    nh.param("max_linear_speed", max_linear_speed, 2.0);
    nh.param("wheelbase", wheelbase, 1.75);

    PathTrackingNode node(lookahead_distance, max_linear_speed, wheelbase);
    
    ros::spin();
    return 0;
}
