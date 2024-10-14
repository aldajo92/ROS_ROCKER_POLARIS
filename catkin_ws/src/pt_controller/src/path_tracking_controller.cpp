#include "pt_controller/path_tracking_controller.h"

PathTrackingController::PathTrackingController(double lookahead_distance, double max_linear_speed, double wheelbase)
    : lookahead_distance_(lookahead_distance),
      max_linear_speed_(max_linear_speed),
      wheelbase_(wheelbase),
      path_received_(false),
      odom_received_(false),
      starting_point_set_(false),
      waypoints_completed_(0),
      waypoints_threshold_(100)
{
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/gem/cmd_vel", 10);
    traveled_path_pub_ = nh_.advertise<nav_msgs::Path>("/traveled_path", 10);
    traveled_path_.header.frame_id = "base_footprint";
}

void PathTrackingController::setPath(const nav_msgs::Path &path)
{
    path_ = path;
    path_received_ = true;
    if (!starting_point_set_ && !path_.poses.empty())
    {
        starting_point_ = path_.poses.front().pose.position;
        starting_point_set_ = true;
    }
}

void PathTrackingController::updateOdometry(const nav_msgs::Odometry &odometry)
{
    current_odometry_ = odometry;
    odom_received_ = true;
}

bool PathTrackingController::isPathReceived() const
{
    return path_received_;
}

bool PathTrackingController::isPathCompleted() const
{
    return waypoints_completed_ >= path_.poses.size();
}

bool PathTrackingController::computeControlCommand()
{
    if (!path_received_ || !odom_received_)
    {
        return false;
    }

    // Current position and orientation
    double robot_x = current_odometry_.pose.pose.position.x;
    double robot_y = current_odometry_.pose.pose.position.y;
    double robot_yaw = tf::getYaw(current_odometry_.pose.pose.orientation);

    double min_distance = std::numeric_limits<double>::max();
    int closest_point_index = -1;

    for (size_t i = 0; i < path_.poses.size(); ++i)
    {
        double dx = path_.poses[i].pose.position.x - robot_x;
        double dy = path_.poses[i].pose.position.y - robot_y;
        double distance = hypot(dx, dy);

        if (distance < min_distance)
        {
            min_distance = distance;
            closest_point_index = i;
        }
    }

    geometry_msgs::Point lookahead_point;
    bool lookahead_point_found = false;

    for (size_t i = closest_point_index; i < path_.poses.size(); ++i)
    {
        double dx = path_.poses[i].pose.position.x - robot_x;
        double dy = path_.poses[i].pose.position.y - robot_y;
        double distance = hypot(dx, dy);

        if (distance >= lookahead_distance_)
        {
            double angle_to_point = atan2(dy, dx);
            double angle_diff = angle_to_point - robot_yaw;
            angle_diff = atan2(sin(angle_diff), cos(angle_diff)); // Normalize angle

            if (fabs(angle_diff) <= M_PI / 2)
            {
                lookahead_point = path_.poses[i].pose.position;
                lookahead_point_found = true;
                waypoints_completed_++;
                break;
            }
        }
    }

    if (!lookahead_point_found)
    {
        stopRobot();
        ROS_INFO("Goal reached or no valid lookahead point ahead. Stopping the robot.");
        return false;
    }

    // Check if the robot has completed a loop
    double dx_to_start = starting_point_.x - robot_x;
    double dy_to_start = starting_point_.y - robot_y;
    double distance_to_start = hypot(dx_to_start, dy_to_start);

    if (waypoints_completed_ > waypoints_threshold_ && distance_to_start < lookahead_distance_)
    {
        stopRobot();
        ROS_INFO("Completed a loop. Stopping the robot.");
        return false;
    }

    // Compute the steering angle
    double dx = lookahead_point.x - robot_x;
    double dy = lookahead_point.y - robot_y;
    double angle_to_goal = atan2(dy, dx);
    double alpha = angle_to_goal - robot_yaw;
    alpha = atan2(sin(alpha), cos(alpha)); // Normalize alpha

    if (fabs(alpha) > M_PI / 2)
    {
        stopRobot();
        ROS_INFO("Lookahead point is behind the robot. Stopping.");
        return false;
    }

    double kappa = (2 * sin(alpha)) / lookahead_distance_;
    double linear_speed = max_linear_speed_;
    double angular_speed = linear_speed * kappa;

    // Publish the control command
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_speed;
    cmd_vel.angular.z = angular_speed;
    cmd_vel_pub_.publish(cmd_vel);

    // Update traveled path
    geometry_msgs::PoseStamped traveled_pose;
    traveled_pose.header.stamp = ros::Time::now();
    traveled_pose.header.frame_id = "odom";           // Same frame as odometry
    traveled_pose.pose = current_odometry_.pose.pose; // Use the current robot pose
    traveled_path_.poses.push_back(traveled_pose);

    // Publish the traveled path
    traveled_path_.header.stamp = ros::Time::now(); // Update the timestamp
    traveled_path_pub_.publish(traveled_path_);

    return true;
}

void PathTrackingController::stopRobot()
{
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    cmd_vel_pub_.publish(stop_msg);
}
