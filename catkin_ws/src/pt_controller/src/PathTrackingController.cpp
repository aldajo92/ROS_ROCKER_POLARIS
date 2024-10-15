#include "pt_controller/PathTrackingController.h"

PathTrackingController::PathTrackingController(
    double lookahead_distance,
    double max_linear_speed,
    double max_angular_speed,
    double goal_tolerance,
    ControllerActionCallback controller_action_callback)
    : lookahead_distance_(lookahead_distance),
      max_linear_speed_(max_linear_speed),
      max_angular_speed_(max_angular_speed),
      goal_tolerance_(goal_tolerance),
      controller_action_callback_(controller_action_callback),
      path_received_(false),
      odom_received_(false),
      starting_point_set_(false),
      log_interval_(std::chrono::duration<double>(0.5)),
      last_log_time_(std::chrono::system_clock::now())
{
}

void PathTrackingController::setPath(const nav_msgs::Path &path)
{
    if (!path_received_)
    {
        path_ = path;
        path_received_ = true;

        if (!starting_point_set_ && !path_.poses.empty())
        {
            starting_point_ = path_.poses.front().pose.position;
            starting_point_set_ = true;
        }
    }
}

void PathTrackingController::setOdom(const nav_msgs::Odometry &odom)
{
    if (!path_received_)
    {
        // ROS_WARN("No path to follow.");
        return;
    }

    current_odometry_ = odom;
    odom_received_ = true;
}

std::pair<double, int> PathTrackingController::calculateMinDistanceInPath(
    const std::vector<double> &position,
    const nav_msgs::Path &path)
{
    if (position.size() != 2)
    {
        // ROS_WARN("Position vector must have exactly 2 elements (x, y).");
        return {std::numeric_limits<double>::max(), -1}; // Return invalid result
    }

    double robot_x = position[0];
    double robot_y = position[1];

    double min_distance = std::numeric_limits<double>::max();
    int closest_point_index = -1;

    for (size_t i = 0; i < path.poses.size(); ++i)
    {
        double dx = path.poses[i].pose.position.x - robot_x;
        double dy = path.poses[i].pose.position.y - robot_y;
        double distance = hypot(dx, dy);

        if (distance < min_distance)
        {
            min_distance = distance;
            closest_point_index = i;
        }
    }

    return {min_distance, closest_point_index};
}

std::vector<double> PathTrackingController::computeControlAction(
    const std::vector<double> &position,
    const std::vector<double> &lookahead_point)
{
    if (position.size() != 3)
    {
        // ROS_INFO("Position vector must have exactly 3 elements (x, y, yaw).");
        return {0.0, 0.0};
    }

    double robot_x = position[0];
    double robot_y = position[1];
    double robot_yaw = position[2];

    double lookahead_point_x = lookahead_point[0];
    double lookahead_point_y = lookahead_point[1];

    double dx = lookahead_point_x - robot_x;
    double dy = lookahead_point_y - robot_y;
    double angle_to_goal = atan2(dy, dx);
    double alpha = angle_to_goal - robot_yaw;
    alpha = atan2(sin(alpha), cos(alpha));

    // Compute the curvature (kappa)
    double kappa = (2 * sin(alpha)) / lookahead_distance_;

    double alignment_factor = cos(alpha/2);
    double linear_speed = max_linear_speed_ * std::max(alignment_factor, 0.0);

    double angular_speed = max_angular_speed_ * kappa;

    return {linear_speed, angular_speed, alpha, kappa};
}

void PathTrackingController::computeControlCommand()
{
    double robot_x = current_odometry_.pose.pose.position.x;
    double robot_y = current_odometry_.pose.pose.position.y;
    double robot_yaw = getYawFromQuaternion(current_odometry_.pose.pose.orientation);

    std::pair<double, int> min_distance_index = calculateMinDistanceInPath({robot_x, robot_y}, path_);

    double min_distance = min_distance_index.first;
    int closest_point_index = min_distance_index.second;

    geometry_msgs::Point lookahead_point;
    bool lookahead_point_found = false;
    double angle_diff;
    double distance;

    for (size_t i = closest_point_index; i < path_.poses.size(); ++i)
    {
        double dx = path_.poses[i].pose.position.x - robot_x;
        double dy = path_.poses[i].pose.position.y - robot_y;
        distance = hypot(dx, dy);

        if (distance >= lookahead_distance_)
        {
            // Compute the angle between the robot's heading and the vector to the point
            double angle_to_point = atan2(dy, dx);
            angle_diff = angle_to_point - robot_yaw;
            angle_diff = atan2(sin(angle_diff), cos(angle_diff)); // Normalize angle

            if (fabs(angle_diff) <= M_PI / 2)
            {
                // The point is ahead of the robot
                lookahead_point = path_.poses[i].pose.position;
                lookahead_point_found = true;
                break;
            }
        }
    }

    if (!lookahead_point_found)
    {
        // ROS_INFO("Goal reached or no valid lookahead point ahead. Stopping the robot.");
        stopRobot();
        return;
    }

    std::vector<double> control = computeControlAction(
        {robot_x, robot_y, robot_yaw},
        {lookahead_point.x, lookahead_point.y});

    double linear_speed = control[0];
    double angular_speed = control[1];
    double alpha = control[2];
    double kappa = control[3];

    // Check if the lookahead point is behind the robot
    if (fabs(alpha) > M_PI / 2)
    {
        // ROS_INFO("Lookahead point is behind the robot. Stopping.");
        stopRobot();
    }

    // Publish velocity command
    if (controller_action_callback_)
    {
        controller_action_callback_(linear_speed, angular_speed);
    }

    if (std::chrono::system_clock::now() - last_log_time_ >= log_interval_)
    {
        last_log_time_ = std::chrono::system_clock::now();

        // ROS_INFO("---- Pure Pursuit Controller ----");
        // ROS_INFO("Robot Position: (x=%.2f, y=%.2f), Yaw: %.2f", robot_x, robot_y, robot_yaw);
        // ROS_INFO("Closest Path Point Index: %d", closest_point_index);
        // ROS_INFO("Lookahead Point: (x=%.2f, y=%.2f)", lookahead_point.x, lookahead_point.y);
        // ROS_INFO("Alpha (Heading Error): %.2f", alpha);
        // ROS_INFO("Curvature (kappa): %.2f", kappa);
        // ROS_INFO("Distance: %.2f", distance);
        // ROS_INFO("Linear Speed: %.2f", linear_speed);
        // ROS_INFO("Angular Speed: %.2f", angular_speed);
        // ROS_INFO("Angle diff: %.2f", angle_diff);
        // ROS_INFO("----------------------------------");
    }
}

bool PathTrackingController::isReachGoal() const
{
    if (path_.poses.empty())
    {
        return false;
    }

    geometry_msgs::Point last_point = path_.poses.back().pose.position;
    double robot_x = current_odometry_.pose.pose.position.x;
    double robot_y = current_odometry_.pose.pose.position.y;

    double dx = robot_x - last_point.x;
    double dy = robot_y - last_point.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // ROS_INFO("---- Error Measure ----");
    // ROS_INFO("Distance: %.2f", distance);
    // ROS_INFO("----------------------------------");
    
    bool reached = distance <= goal_tolerance_;

    return reached;
}

bool PathTrackingController::isOdomReceived() const
{
    return odom_received_;
}

bool PathTrackingController::isPathReceived() const
{
    return path_received_;
}

void PathTrackingController::stopRobot()
{
    if (controller_action_callback_)
    {
        controller_action_callback_(0.0, 0.0);
    }
}

double PathTrackingController::getYawFromQuaternion(const geometry_msgs::Quaternion &q)
{
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}
