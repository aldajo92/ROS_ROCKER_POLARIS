#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

enum NavigationState
{
    IDLE,
    PATH_RECEIVED,
    TRACKING_PATH,
    WAYPOINT_REACHED,
    GOAL_REACHED,
    ERROR,
    RECOVERY
};

class PathTrackingController
{
public:
    PathTrackingController()
    {
        path_sub_ = nh_.subscribe("/path", 10, &PathTrackingController::pathCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 10, &PathTrackingController::odomCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        current_waypoint_index_ = 0;
        max_linear_speed_ = 0.5;
        max_angular_speed_ = 1.0;

        state_ = IDLE; // Start in IDLE state
    }

    void pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        path_ = *msg;
        current_waypoint_index_ = 0;
        state_ = PATH_RECEIVED; // Transition to PATH_RECEIVED when path is received
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (path_.poses.empty())
        {
            state_ = ERROR;
            return;
        }

        // Get the robot's current position and orientation
        double robot_x = msg->pose.pose.position.x;
        double robot_y = msg->pose.pose.position.y;
        double robot_yaw = tf::getYaw(msg->pose.pose.orientation);

        // State Machine Execution
        switch (state_)
        {
        case IDLE:
            ROS_INFO("State: IDLE - Waiting for path...");
            break;

        case PATH_RECEIVED:
            ROS_INFO("State: PATH_RECEIVED - Starting to follow the path.");
            state_ = TRACKING_PATH;
            break;

        case TRACKING_PATH:
            trackPath(robot_x, robot_y, robot_yaw);
            break;

        case WAYPOINT_REACHED:
            handleWaypointReached();
            break;

        case GOAL_REACHED:
            stopRobot();
            ROS_INFO("State: GOAL_REACHED - Path tracking complete.");
            state_ = IDLE; // Return to IDLE after reaching the goal
            break;

        case ERROR:
            ROS_ERROR("State: ERROR - Something went wrong.");
            state_ = RECOVERY; // Transition to recovery state
            break;

        case RECOVERY:
            ROS_WARN("State: RECOVERY - Attempting to recover.");
            // Add recovery behavior (e.g., repositioning)
            // If recovery succeeds:
            state_ = TRACKING_PATH;
            break;

        default:
            ROS_ERROR("Unknown state!");
            break;
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

    NavigationState state_; // The current state of the state machine

    // Track the path based on the current state
    void trackPath(double robot_x, double robot_y, double robot_yaw)
    {
        if (current_waypoint_index_ >= path_.poses.size())
        {
            state_ = GOAL_REACHED;
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

        double linear_speed = max_linear_speed_ * distance_to_target;
        double angular_speed = max_angular_speed_ * heading_error;

        linear_speed = std::min(linear_speed, max_linear_speed_);
        angular_speed = std::min(std::max(angular_speed, -max_angular_speed_), max_angular_speed_);

        // Publish velocity command
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_speed;
        cmd_vel.angular.z = angular_speed;
        cmd_vel_pub_.publish(cmd_vel);

        // Transition to WAYPOINT_REACHED if we are close enough
        if (distance_to_target < 0.1)
        {
            state_ = WAYPOINT_REACHED;
        }
    }

    void handleWaypointReached()
    {
        ROS_INFO("State: WAYPOINT_REACHED - Moving to the next waypoint.");
        if (current_waypoint_index_ < path_.poses.size() - 1)
        {
            current_waypoint_index_++;
            state_ = TRACKING_PATH;
        }
        else
        {
            state_ = GOAL_REACHED; // All waypoints have been reached
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
    PathTrackingController controller;
    ros::spin();
    return 0;
}
