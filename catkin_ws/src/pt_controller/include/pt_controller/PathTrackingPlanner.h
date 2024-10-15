#ifndef PATH_TRACKING_PLANNER_H
#define PATH_TRACKING_PLANNER_H

#include <atomic>
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
    PathTrackingPlanner(double frequency, AbstractPathTrackingController *controller);
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
};

#endif // PATH_TRACKING_PLANNER_H
