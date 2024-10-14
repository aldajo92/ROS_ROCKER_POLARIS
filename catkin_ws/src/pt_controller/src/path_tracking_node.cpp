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
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cmath>
#include <limits>

#include <vector>
#include <utility>

class PathTrackingController
{
public:
    PathTrackingController(double lookahead_distance, double max_linear_speed, double wheelbase)
        : lookahead_distance_(lookahead_distance),
          max_linear_speed_(max_linear_speed),
          wheelbase_(wheelbase),
          path_received_(false),
          odom_received_(false),
          starting_point_set_(false),
          waypoints_completed_(0),
          waypoints_threshold_(100)
    {
        path_sub_ = nh_.subscribe("/gps_path", 10, &PathTrackingController::pathCallback, this);
        odom_sub_ = nh_.subscribe("/gem/base_footprint/odom", 10, &PathTrackingController::odomCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/gem/cmd_vel", 10);
        traveled_path_pub_ = nh_.advertise<nav_msgs::Path>("/traveled_path", 10); // New publisher for traveled path

        traveled_path_.header.frame_id = "base_footprint"; // Set the frame for traveled path to match odometry

        log_interval_ = ros::Duration(0.5);
        last_log_time_ = ros::Time::now();
    }

    void pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        if (!path_received_)
        {
            path_ = *msg;
            path_received_ = true;
            ROS_INFO("Path received with %ld waypoints.", path_.poses.size());

            if (!starting_point_set_ && !path_.poses.empty())
            {
                starting_point_ = path_.poses.front().pose.position;
                starting_point_set_ = true;
            }
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (!path_received_)
        {
            ROS_WARN("No path to follow.");
            return;
        }

        current_odometry_ = *msg;
        odom_received_ = true;

        computeControlCommand();
    }

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
        const nav_msgs::Path &path)
    {
        if (position.size() != 2)
        {
            ROS_INFO("Position vector must have exactly 2 elements (x and y).");
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

    std::vector<double> computeControlAction(
        const std::vector<double> &position,
        const std::vector<double> &lookahead_point)
    {
        if (position.size() != 3)
        {

            ROS_INFO("Position vector must have exactly 3 elements (x, y, yaw).");
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
        alpha = atan2(sin(alpha), cos(alpha)); // Normalize alpha

        // Check if the lookahead point is behind the robot
        if (fabs(alpha) > M_PI / 2)
        {
            ROS_INFO("Lookahead point is behind the robot. Stopping.");
            return {0.0, 0.0};
        }

        // Compute the curvature (kappa)
        double kappa = (2 * sin(alpha)) / lookahead_distance_;

        // Compute linear and angular velocities
        double linear_speed = max_linear_speed_;
        double angular_speed = linear_speed * kappa;

        return {linear_speed, angular_speed};
    }

    // Compute the control command using Pure Pursuit with nearest point search
    void computeControlCommand()
    {
        if (!path_received_ || !odom_received_)
        {
            return;
        }

        // Current position and orientation
        double robot_x = current_odometry_.pose.pose.position.x;
        double robot_y = current_odometry_.pose.pose.position.y;
        double robot_yaw = tf::getYaw(current_odometry_.pose.pose.orientation);

        std::pair<double, int> min_distance_index = calculateMinDistanceInPath({robot_x, robot_y}, path_);

        double min_distance = min_distance_index.first;
        int closest_point_index = min_distance_index.second;

        geometry_msgs::Point lookahead_point;
        bool lookahead_point_found = false;
        double angle_diff;

        for (size_t i = closest_point_index; i < path_.poses.size(); ++i)
        {
            double dx = path_.poses[i].pose.position.x - robot_x;
            double dy = path_.poses[i].pose.position.y - robot_y;
            double distance = hypot(dx, dy);

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
                    waypoints_completed_++;
                    break;
                }
            }
        }

        if (!lookahead_point_found)
        {
            ROS_INFO("Goal reached or no valid lookahead point ahead. Stopping the robot.");
            stopRobot();
            return;
        }

        std::vector<double> control = computeControlAction(
            {robot_x, robot_y, robot_yaw},
            {lookahead_point.x, lookahead_point.y});

        double linear_speed = control[0];
        double angular_speed = control[1];

        // Publish velocity command
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

        if (ros::Time::now() - last_log_time_ >= log_interval_)
        {
            last_log_time_ = ros::Time::now();

            ROS_INFO("---- Pure Pursuit Controller ----");
            ROS_INFO("Robot Position: (x=%.2f, y=%.2f), Yaw: %.2f", robot_x, robot_y, robot_yaw);
            ROS_INFO("Closest Path Point Index: %d", closest_point_index);
            ROS_INFO("Lookahead Point: (x=%.2f, y=%.2f)", lookahead_point.x, lookahead_point.y);
            // ROS_INFO("Alpha (Heading Error): %.2f", alpha);
            // ROS_INFO("Curvature (kappa): %.2f", kappa);
            ROS_INFO("Linear Speed: %.2f", linear_speed);
            ROS_INFO("Angular Speed: %.2f", angular_speed);
            ROS_INFO("Angle diff: %.2f", angle_diff);
            ROS_INFO("----------------------------------");
        }
    }

    void stopRobot()
    {
        geometry_msgs::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        cmd_vel_pub_.publish(stop_msg);
    }
};

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
