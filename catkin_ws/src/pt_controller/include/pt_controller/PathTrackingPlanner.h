#ifndef PATH_TRACKING_PLANNER_H
#define PATH_TRACKING_PLANNER_H

#include "pt_controller/PathTrackingController.h"

#include <atomic>
#include <thread>
#include <string>
#include <iostream>
#include <chrono>

enum PathTrackingState
{
    INIT,
    FOLLOW_PATH,
    NO_INPUT_DATA,
    WAITING_ODOM,
    WAITING_PATH,
    ERROR,
    SUCCESS_END_PATH
};

using PathTrackingStateCallback = std::function<void(PathTrackingState)>;

class PathTrackingPlanner
{
public:
    PathTrackingPlanner(double frequency);
    PathTrackingPlanner(double frequency, AbstractPathTrackingController *controller, PathTrackingStateCallback state_callback);
    ~PathTrackingPlanner();

    void execute();
    void stop();
    PathTrackingState getState() const;
    std::string stateToString(PathTrackingState state) const;

private:
    void stateMonitor();
    void updateState(PathTrackingState state);

    std::atomic<bool> running_;
    std::thread planner_thread_;
    PathTrackingState current_state_;
    double frequency_;
    AbstractPathTrackingController *controller_;
    PathTrackingStateCallback state_callback_;
};

#endif // PATH_TRACKING_PLANNER_H
