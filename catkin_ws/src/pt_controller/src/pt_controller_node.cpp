#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <ackermann_msgs/AckermannDrive.h> // Include the Ackermann message type

class PathTrackingController
{
public:
    PathTrackingController(double kp_linear, double kp_angular)
        : Kp_linear_(kp_linear), Kp_angular_(kp_angular) // Initialize Kp values
    {
        path_sub_ = nh_.subscribe("/gps_path", 10, &PathTrackingController::pathCallback, this);
        odom_sub_ = nh_.subscribe("/gem/base_footprint/odom", 10, &PathTrackingController::odomCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/gem/cmd_vel", 10);

        current_waypoint_index_ = 0;
        max_linear_speed_ = 2;
        max_angular_speed_ = 0.5;

        // Initialize logging control
        log_interval_ = ros::Duration(0.5); // Log every 0.5 seconds for testing
        last_log_time_ = ros::Time::now();
    }

    void pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        if (path_.poses.empty())
        {
            // Only load the path if it's not already being followed
            path_ = *msg;
            current_waypoint_index_ = 0; // Start from the first waypoint
            ROS_INFO("Path received with %ld waypoints.", path_.poses.size());
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (path_.poses.empty())
        {
            ROS_WARN("No path to follow.");
            return; // No path to follow
        }

        // Get the robot's current position and orientation
        double robot_x = msg->pose.pose.position.x;
        double robot_y = msg->pose.pose.position.y;
        double robot_yaw = tf::getYaw(msg->pose.pose.orientation);

        // Follow the path and adjust velocity
        trackPath(robot_x, robot_y, robot_yaw);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_vel_pub_;

    nav_msgs::Path path_;
    int current_waypoint_index_;
    double max_linear_speed_;
    double max_angular_speed_;

    double Kp_linear_;  // Proportional gain for linear velocity
    double Kp_angular_; // Proportional gain for angular velocity

    // Logging variables
    ros::Duration log_interval_;
    ros::Time last_log_time_;

    double steering_angle_;          // Steering angle from Ackermann commands
    double steering_angle_velocity_; // Steering angle velocity from Ackermann commands

    // Track the path and compute velocity
    void trackPath(double robot_x, double robot_y, double robot_yaw)
    {
        if (current_waypoint_index_ >= path_.poses.size())
        {
            stopRobot(); // Stop when the path is complete
            ROS_INFO("Goal reached. Stopping the robot.");
            return;
        }

        // Get the target waypoint
        geometry_msgs::PoseStamped target_pose = path_.poses[current_waypoint_index_];
        double target_x = target_pose.pose.position.x;
        double target_y = target_pose.pose.position.y;

        // Compute the error to the target
        double dx = target_x - robot_x;
        double dy = target_y - robot_y;
        double distance_to_target = sqrt(dx * dx + dy * dy);
        double angle_to_target = atan2(dy, dx);

        // Compute heading error and control velocities
        double heading_error = angle_to_target - robot_yaw;
        heading_error = atan2(sin(heading_error), cos(heading_error)); // Normalize

        // Calculate velocities using Kp
        double linear_speed = Kp_linear_ * distance_to_target;
        double angular_speed = Kp_angular_ * heading_error;

        // Clamp velocities to the maximum allowed values
        linear_speed = std::min(linear_speed, max_linear_speed_);
        angular_speed = std::min(std::max(angular_speed, -max_angular_speed_), max_angular_speed_);

        // Debug information: Print the key variables at a controlled rate
        if (ros::Time::now() - last_log_time_ >= log_interval_)
        {
            last_log_time_ = ros::Time::now(); // Update the last log time

            ROS_INFO("---- Robot Status ----");
            ROS_INFO("Position: (x=%.2f, y=%.2f), Yaw: %.2f", robot_x, robot_y, robot_yaw);
            ROS_INFO("Target Waypoint: (x=%.2f, y=%.2f)", target_x, target_y);
            ROS_INFO("Distance to Target: %.2f", distance_to_target);
            ROS_INFO("Angle to Target: %.2f", angle_to_target);
            ROS_INFO("Heading Error: %.2f", heading_error);
            ROS_INFO("Computed Velocities - Linear: %.2f, Angular: %.2f", linear_speed, angular_speed);
            ROS_INFO("-----------------------");
        }

        // Publish velocity command
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_speed;
        cmd_vel.angular.z = angular_speed;
        cmd_vel_pub_.publish(cmd_vel);

        // Move to the next waypoint if close enough to the current one
        if (distance_to_target < 0.1)
        {
            ROS_INFO("Waypoint %d reached.", current_waypoint_index_);
            current_waypoint_index_++;
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

    // Set Kp values directly in the main function
    double Kp_linear = 1.0;  // Set your desired value for linear gain
    double Kp_angular = 0.5; // Set your desired value for angular gain

    PathTrackingController controller(Kp_linear, Kp_angular);
    ros::spin();
    return 0;
}
