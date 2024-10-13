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
    PathTrackingController(double lookahead_distance, double max_linear_speed, double wheelbase)
        : lookahead_distance_(lookahead_distance),
          max_linear_speed_(max_linear_speed),
          wheelbase_(wheelbase),
          path_received_(false),
          odom_received_(false)
    {
        path_sub_ = nh_.subscribe("/gps_path", 10, &PathTrackingController::pathCallback, this);
        odom_sub_ = nh_.subscribe("/gem/base_footprint/odom", 10, &PathTrackingController::odomCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/gem/cmd_vel", 10);

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

    nav_msgs::Path path_;
    nav_msgs::Odometry current_odometry_;
    double lookahead_distance_;
    double max_linear_speed_;
    double wheelbase_;
    bool path_received_;
    bool odom_received_;

    
    ros::Duration log_interval_;
    ros::Time last_log_time_;

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
                // Compute the angle between the robot's heading and the vector to the point
                double angle_to_point = atan2(dy, dx);
                double angle_diff = angle_to_point - robot_yaw;
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
            
            stopRobot();
            ROS_INFO("Goal reached or no valid lookahead point ahead. Stopping the robot.");
            return;
        }

        // Compute the steering angle
        double dx = lookahead_point.x - robot_x;
        double dy = lookahead_point.y - robot_y;
        double angle_to_goal = atan2(dy, dx);
        double alpha = angle_to_goal - robot_yaw;
        alpha = atan2(sin(alpha), cos(alpha)); // Normalize alpha

        // Check if the lookahead point is behind the robot
        if (fabs(alpha) > M_PI / 2)
        {
            stopRobot();
            ROS_INFO("Lookahead point is behind the robot. Stopping.");
            return;
        }

        // Compute the curvature (kappa)
        double kappa = (2 * sin(alpha)) / lookahead_distance_;

        // Compute linear and angular velocities
        double linear_speed = max_linear_speed_;
        double angular_speed = linear_speed * kappa;

        // Publish velocity command
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_speed;
        cmd_vel.angular.z = angular_speed;
        cmd_vel_pub_.publish(cmd_vel);

        if (ros::Time::now() - last_log_time_ >= log_interval_)
        {
            last_log_time_ = ros::Time::now();

            ROS_INFO("---- Pure Pursuit Controller ----");
            ROS_INFO("Robot Position: (x=%.2f, y=%.2f), Yaw: %.2f", robot_x, robot_y, robot_yaw);
            ROS_INFO("Closest Path Point Index: %d", closest_point_index);
            ROS_INFO("Lookahead Point: (x=%.2f, y=%.2f)", lookahead_point.x, lookahead_point.y);
            ROS_INFO("Alpha (Heading Error): %.2f", alpha);
            ROS_INFO("Curvature (kappa): %.2f", kappa);
            ROS_INFO("Linear Speed: %.2f", linear_speed);
            ROS_INFO("Angular Speed: %.2f", angular_speed);
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
    double lookahead_distance = 2.0; // Adjust as needed
    double max_linear_speed = 2.0;   // Adjust as needed
    double wheelbase = 1.75;         // Adjust based on your robot's wheelbase

    PathTrackingController controller(lookahead_distance, max_linear_speed, wheelbase);
    ros::spin();
    return 0;
}
