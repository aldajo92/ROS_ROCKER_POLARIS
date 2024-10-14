#ifndef PATH_TRACKING_CONTROLLER_H
#define PATH_TRACKING_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <vector>
#include <utility> // For std::pair

class PathTrackingController
{
public:
    PathTrackingController(double lookahead_distance, double max_linear_speed, double wheelbase);

    void pathCallback(const nav_msgs::Path::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher traveled_path_pub_; // Publisher for the traveled path

    nav_msgs::Path path_;
    nav_msgs::Path traveled_path_; // Stores the traveled path
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

    ros::Duration log_interval_;
    ros::Time last_log_time_;

    std::pair<double, int> calculateMinDistanceInPath(
        const std::vector<double> &position,
        const nav_msgs::Path &path);

    std::vector<double> computeControlAction(
        const std::vector<double> &position,
        const std::vector<double> &lookahead_point);

    void computeControlCommand();
    void stopRobot();
};

#endif // PATH_TRACKING_CONTROLLER_H
