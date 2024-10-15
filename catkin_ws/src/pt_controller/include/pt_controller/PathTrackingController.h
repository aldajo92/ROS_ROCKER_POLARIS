#ifndef PATH_TRACKING_CONTROLLER_H
#define PATH_TRACKING_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <vector>
#include <utility>
#include <functional>

using ControllerActionCallback = std::function<void(double, double)>;

class PathTrackingController
{
public:
    PathTrackingController(double lookahead_distance, double max_linear_speed, double wheelbase, ControllerActionCallback controller_action_callback);

    void setPath(const nav_msgs::Path &path);
    void setOdom(const nav_msgs::Odometry &odom);

    void computeControlCommand();
    void stopRobot();

    std::pair<double, int> calculateMinDistanceInPath(
        const std::vector<double> &position,
        const nav_msgs::Path &path);

    std::vector<double> computeControlAction(
        const std::vector<double> &position,
        const std::vector<double> &lookahead_point);

private:
    nav_msgs::Path path_;
    nav_msgs::Odometry current_odometry_;
    double lookahead_distance_;
    double max_linear_speed_;
    double wheelbase_;
    ControllerActionCallback controller_action_callback_;

    bool path_received_;
    bool odom_received_;

    geometry_msgs::Point starting_point_;
    bool starting_point_set_;
    size_t waypoints_completed_;
    const size_t waypoints_threshold_;

    // ros::Duration log_interval_;
    // ros::Time last_log_time_;
};

#endif // PATH_TRACKING_CONTROLLER_H
