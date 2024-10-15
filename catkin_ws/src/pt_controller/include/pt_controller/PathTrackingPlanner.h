#ifndef PLANNER_PATH_TRACKING_H
#define PLANNER_PATH_TRACKING_H

#include <atomic>
#include <chrono>
#include <thread>
#include <string>
#include "pt_controller/PathTrackingController.h"

enum PlannerState
{
    INIT,
    FOLLOW_PATH,
    ERROR,
    SUCCESS_END_PATH
};

class PathTrackingPlanner
{
public:
    PathTrackingPlanner();
    PathTrackingPlanner(PathTrackingController *controller);
    ~PathTrackingPlanner();

    std::string stateToString(PlannerState state) const;

    void execute();
    void stop();
    PlannerState getState() const;

private:
    void stateMonitor();

    PlannerState current_state_;
    std::thread planner_thread_;
    PathTrackingController *controller_;
    std::atomic<bool> running_;
};

#endif // PLANNER_PATH_TRACKING_H
