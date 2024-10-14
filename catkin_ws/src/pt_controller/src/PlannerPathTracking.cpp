#include "pt_controller/planner_path_tracking.h"

PlannerPathTracking::PlannerPathTracking(double lookahead_distance, double max_linear_speed, double wheelbase)
    : lookahead_distance_(lookahead_distance),
      max_linear_speed_(max_linear_speed),
      wheelbase_(wheelbase),
      path_tracking_controller_(lookahead_distance, max_linear_speed, wheelbase),
      current_state_(INIT)
{
}

void PlannerPathTracking::setPath(const nav_msgs::Path &path)
{
    // path_tracking_controller_.setPath(path);
    // current_state_ = FOLLOW_PATH;
}

void PlannerPathTracking::updateOdometry(const nav_msgs::Odometry &odometry)
{
    // path_tracking_controller_.updateOdometry(odometry);
    // updateState();
}

PlannerState PlannerPathTracking::getState() const
{
    return current_state_;
}

void PlannerPathTracking::updateState()
{
    // switch (current_state_)
    // {
    // case INIT:
    //     // Initialization logic
    //     if (path_tracking_controller_.isPathReceived())
    //     {
    //         current_state_ = FOLLOW_PATH;
    //     }
    //     break;
    // case FOLLOW_PATH:
    //     // Path following logic
    //     if (!path_tracking_controller_.computeControlCommand())
    //     {
    //         current_state_ = ERROR;
    //     }
    //     else if (path_tracking_controller_.isPathCompleted())
    //     {
    //         current_state_ = SUCESS_END_PATH;
    //     }
    //     break;
    // case ERROR:
    //     // Error handling logic
    //     path_tracking_controller_.stopRobot();
    //     break;
    // case SUCESS_END_PATH:
    //     // Success handling logic
    //     path_tracking_controller_.stopRobot();
    //     ROS_INFO("Path successfully completed.");
    //     break;
    // }
}
