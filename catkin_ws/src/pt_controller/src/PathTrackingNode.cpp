#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "pt_controller/PathTrackingController.h"
#include "pt_controller/PathTrackingPlanner.h"

class PathTrackingNode
{
public:
    PathTrackingNode(
        const std::string &path_topic,
        const std::string &odom_topic,
        const std::string &cmd_vel_topic,
        const std::string &traveled_path_topic,
        const std::string &traveled_path_frame_id,
        double lookahead_distance,
        double max_linear_speed,
        double max_angular_speed,
        double goal_tolerance,
        double frequency)
        : path_tracking_controller_(
              lookahead_distance,
              max_linear_speed,
              max_angular_speed,
              goal_tolerance,
              std::bind(&PathTrackingNode::sendVelocitiesCallback, this, std::placeholders::_1, std::placeholders::_2)),
          path_tracking_planner_(
            frequency,
            &path_tracking_controller_,
            std::bind(&PathTrackingNode::stateCallback, this, std::placeholders::_1)),
            odom_topic_(odom_topic),
            path_topic_(path_topic)
    {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);
        traveled_path_pub_ = nh_.advertise<nav_msgs::Path>(traveled_path_topic, 10);

        traveled_path_.header.frame_id = traveled_path_frame_id;
    }

    void startPlanner(){
        path_sub_ = nh_.subscribe(path_topic_, 10, &PathTrackingNode::pathCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 10, &PathTrackingNode::odomCallback, this);

        path_tracking_planner_.execute();
    }

    void pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        path_tracking_controller_.setPath(*msg);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        path_tracking_controller_.setOdom(*msg);
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

    void stateCallback(const PathTrackingState &state)
    {
        ROS_INFO("Current state: %s", path_tracking_planner_.stateToString(state).c_str());
    }

    void updateTraveledPath(const nav_msgs::Odometry &odometry)
    {
        geometry_msgs::PoseStamped traveled_pose;
        traveled_pose.header.stamp = ros::Time::now();
        traveled_pose.header.frame_id = "odom";
        traveled_pose.pose = odometry.pose.pose;
        traveled_path_.poses.push_back(traveled_pose);

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
    PathTrackingPlanner path_tracking_planner_;

    std::string odom_topic_;
    std::string path_topic_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_tracking_node");
    ros::NodeHandle nh("~");

    // Parameters for topics
    std::string path_topic;
    std::string odom_topic;
    std::string cmd_vel_topic;
    std::string traveled_path_topic;
    std::string traveled_path_frame_id;

    if (!nh.getParam("path_topic", path_topic))
    {
        path_topic = "/gps_path";
    }

    if (!nh.getParam("odom_topic", odom_topic))
    {
        odom_topic = "/odom";
    }

    if (!nh.getParam("cmd_vel_topic", cmd_vel_topic))
    {
        cmd_vel_topic = "/cmd_vel";
    }

    if (!nh.getParam("traveled_path_topic", traveled_path_topic))
    {
        traveled_path_topic = "/traveled_path";
    }

    if (!nh.getParam("traveled_path_frame_id", traveled_path_frame_id))
    {
        traveled_path_frame_id = "base_footprint";
    }

    // Parameter for path tracking controller
    double lookahead_distance;
    double max_linear_speed;
    double max_angular_speed;
    double goal_tolearance;
    double frequency;

    if (!nh.getParam("lookahead_distance", lookahead_distance))
    {
        ROS_WARN("lookahead_distance param not found, using default value of 2.0");
        lookahead_distance = 2.0;
    }

    if (!nh.getParam("max_linear_speed", max_linear_speed))
    {
        ROS_WARN("max_linear_speed param not found, using default value of 2.0");
        max_linear_speed = 2.0;
    }

    if (!nh.getParam("max_angular_speed", max_angular_speed))
    {
        ROS_WARN("max_angular_speed param not found, using default value of 0.5");
        max_angular_speed = 0.5;
    }

    if (!nh.getParam("goal_tolerance", goal_tolearance))
    {
        ROS_WARN("goal_tolerance param not found, using default value of 0.1");
        goal_tolearance = 0.1;
    }

    if (!nh.getParam("frequency", frequency))
    {
        ROS_WARN("frequency param not found, using default value of 10.0");
        frequency = 10.0;
    }

    PathTrackingNode node(
        path_topic,
        odom_topic,
        cmd_vel_topic,
        traveled_path_topic,
        traveled_path_frame_id,
        lookahead_distance,
        max_linear_speed,
        max_angular_speed,
        goal_tolearance,
        frequency);

    node.startPlanner();

    ros::spin();
    return 0;
}
