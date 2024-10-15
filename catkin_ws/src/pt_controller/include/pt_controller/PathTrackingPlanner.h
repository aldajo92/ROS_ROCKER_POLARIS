#ifndef PATH_TRACKING_PLANNER_H
#define PATH_TRACKING_PLANNER_H

#include <atomic>
#include <chrono>
#include <thread>
#include <string>
#include "pt_controller/PathTrackingController.h"

enum PathTrackingState
{
    INIT,
    FOLLOW_PATH,
    MISSING_ODOM,
    MISSING_PATH,
    ERROR,
    SUCCESS_END_PATH
};

class PathTrackingPlanner
{
public:
    PathTrackingPlanner(double frequency);
    PathTrackingPlanner(double frequency, PathTrackingController *controller);
    ~PathTrackingPlanner();

    std::string stateToString(PathTrackingState state) const;

    void execute();
    void stop();
    PathTrackingState getState() const;

private:
    PathTrackingState current_state_;
    std::thread planner_thread_;
    PathTrackingController *controller_;
    std::atomic<bool> running_;
    double frequency_;

    void stateMonitor();
    void updateState(PathTrackingState state);
};

#endif // PATH_TRACKING_PLANNER_H
