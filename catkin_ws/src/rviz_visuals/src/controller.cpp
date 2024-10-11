#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

class PathTrackingController
{
public:
    PathTrackingController()
    {
        // Initialize subscribers and publishers
        path_sub_ = nh_.subscribe("/path", 10, &PathTrackingController::pathCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 10, &PathTrackingController::odomCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        current_waypoint_index_ = 0;
        max_linear_speed_ = 0.5;
        max_angular_speed_ = 1.0;
    }

    void pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        path_ = *msg;
        current_waypoint_index_ = 0; // Reset to the first waypoint
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (path_.poses.empty())
            return; // No path to follow

        // Get the robot's current position and orientation
        double robot_x = msg->pose.pose.position.x;
        double robot_y = msg->pose.pose.position.y;
        double robot_yaw = tf::getYaw(msg->pose.pose.orientation);

        // Get the target waypoint from the path
        geometry_msgs::PoseStamped target_pose = path_.poses[current_waypoint_index_];
        double target_x = target_pose.pose.position.x;
        double target_y = target_pose.pose.position.y;

        // Compute the difference between the robot and the target
        double dx = target_x - robot_x;
        double dy = target_y - robot_y;
        double distance_to_target = sqrt(dx * dx + dy * dy);
        double angle_to_target = atan2(dy, dx);

        // Compute the heading error
        double heading_error = angle_to_target - robot_yaw;

        // Normalize the heading error to [-pi, pi]
        heading_error = atan2(sin(heading_error), cos(heading_error));

        // Simple proportional control for linear and angular velocity
        double linear_speed = max_linear_speed_ * distance_to_target;
        double angular_speed = max_angular_speed_ * heading_error;

        // Clamp the speeds
        linear_speed = std::min(linear_speed, max_linear_speed_);
        angular_speed = std::min(std::max(angular_speed, -max_angular_speed_), max_angular_speed_);

        // Create and publish the velocity command
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_speed;
        cmd_vel.angular.z = angular_speed;
        cmd_vel_pub_.publish(cmd_vel);

        // Move to the next waypoint if close enough
        if (distance_to_target < 0.1 && current_waypoint_index_ < path_.poses.size() - 1)
        {
            current_waypoint_index_++;
        }
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
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_tracking_controller");
    PathTrackingController controller;
    ros::spin();
    return 0;
}
