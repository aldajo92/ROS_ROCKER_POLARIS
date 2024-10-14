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
    PlannerPathTracking(double lookahead_distance, double max_linear_speed, double wheelbase);
    void setPath(const nav_msgs::Path &path);
    void updateOdometry(const nav_msgs::Odometry &odometry);
    PlannerState getState() const;

private:
    double lookahead_distance_;
    double max_linear_speed_;
    double wheelbase_;
    PlannerState current_state_;
    PathTrackingController path_tracking_controller_;

    void updateState();
};

#endif // PLANNER_PATH_TRACKING_H
