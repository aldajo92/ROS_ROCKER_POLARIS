#ifndef PATH_TRACKING_CONTROLLER_H
#define PATH_TRACKING_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cmath>
#include <limits>

class PathTrackingController
{
public:
    PathTrackingController(double lookahead_distance, double max_linear_speed, double wheelbase);
    void setPath(const nav_msgs::Path &path);
    void updateOdometry(const nav_msgs::Odometry &odometry);
    bool isPathReceived() const;
    bool isPathCompleted() const;
    bool computeControlCommand();
    void stopRobot();

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher traveled_path_pub_;
    nav_msgs::Path path_;
    nav_msgs::Odometry current_odometry_;
    double lookahead_distance_;
    double max_linear_speed_;
    double wheelbase_;
    bool path_received_;
    bool odom_received_;
    geometry_msgs::Point starting_point_;
    bool starting_point_set_;
    size_t waypoints_completed_;
    const size_t waypoints_threshold_;
    nav_msgs::Path traveled_path_; // Stores the traveled path
};

#endif // PATH_TRACKING_CONTROLLER_H
