#ifndef PLANNER_PATH_TRACKING_H
#define PLANNER_PATH_TRACKING_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <vector>
#include <limits>

enum PlannerState
{
    INIT,
    FOLLOW_PATH,
    ERROR,
    END_PATH
};

class PlannerPathTracking
{
public:
    PlannerPathTracking(double lookahead_distance, double max_linear_speed, double wheelbase)
        : lookahead_distance_(lookahead_distance),
          max_linear_speed_(max_linear_speed),
          wheelbase_(wheelbase),
          current_state_(INIT)
    {
    }

    void setPath(const nav_msgs::Path &path)
    {
        current_state_ = FOLLOW_PATH;
    }

    void updateOdometry(const nav_msgs::Odometry &odometry)
    {
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

    void updateState()
    {
        switch (current_state_)
        {
        case INIT:
            break;
        case FOLLOW_PATH:
            break;
        case ERROR:
            break;
        case END_PATH:
            break;
        }
    }
};

#endif // PLANNER_PATH_TRACKING_H
