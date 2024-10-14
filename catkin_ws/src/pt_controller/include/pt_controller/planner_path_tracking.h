#ifndef PLANNER_PATH_TRACKING_H
#define PLANNER_PATH_TRACKING_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <vector>
#include <limits>
#include "path_tracking_controller.h"

enum PlannerState
{
    INIT,
    FOLLOW_PATH,
    ERROR,
    SUCESS_END_PATH
};

class PlannerPathTracking
{
public:
    PlannerPathTracking(double lookahead_distance, double max_linear_speed, double wheelbase)
        : lookahead_distance_(lookahead_distance),
          max_linear_speed_(max_linear_speed),
          wheelbase_(wheelbase),
          path_tracking_controller_(lookahead_distance, max_linear_speed, wheelbase),
          current_state_(INIT)
    {
    }

    void setPath(const nav_msgs::Path &path)
    {
        path_tracking_controller_.setPath(path);
        current_state_ = FOLLOW_PATH;
    }

    void updateOdometry(const nav_msgs::Odometry &odometry)
    {
        path_tracking_controller_.updateOdometry(odometry);
        updateState();
    }

    PlannerState getState() const
    {
        return current_state_;
    }

private:
    double lookahead_distance_;
    double max_linear_speed_;
    double wheelbase_;
    PlannerState current_state_;
    PathTrackingController path_tracking_controller_;

    void updateState()
    {
        switch (current_state_)
        {
        case INIT:
            // Initialization logic
            if (path_tracking_controller_.isPathReceived())
            {
                current_state_ = FOLLOW_PATH;
            }
            break;
        case FOLLOW_PATH:
            // Path following logic
            if (!path_tracking_controller_.computeControlCommand())
            {
                current_state_ = ERROR;
            }
            else if (path_tracking_controller_.isPathCompleted())
            {
                current_state_ = SUCESS_END_PATH;
            }
            break;
        case ERROR:
            // Error handling logic
            path_tracking_controller_.stopRobot();
            break;
        case SUCESS_END_PATH:
            // Success handling logic
            path_tracking_controller_.stopRobot();
            ROS_INFO("Path successfully completed.");
            break;
        }
    }
};

#endif // PLANNER_PATH_TRACKING_H
