#include "pt_controller/PathTrackingPlanner.h"
#include <iostream>
#include <chrono>
#include <thread>

PathTrackingPlanner::PathTrackingPlanner(double frequency)
    : current_state_(INIT), running_(false), frequency_(frequency), controller_(nullptr)
{
}

PathTrackingPlanner::PathTrackingPlanner(double frequency, PathTrackingController *controller)
    : current_state_(INIT), running_(false), frequency_(frequency), controller_(controller)
{
}

PathTrackingPlanner::~PathTrackingPlanner()
{
    stop();
}

void PathTrackingPlanner::execute()
{
    running_.store(true);
    planner_thread_ = std::thread(&PathTrackingPlanner::stateMonitor, this);
}

void PathTrackingPlanner::stop()
{
    running_.store(false);
    if (planner_thread_.joinable())
    {
        planner_thread_.join();
    }
}

PathTrackingState PathTrackingPlanner::getState() const
{
    return current_state_;
}

std::string PathTrackingPlanner::stateToString(PathTrackingState state) const
{
    switch (state)
    {
    case INIT:
        return "INIT";
    case FOLLOW_PATH:
        return "FOLLOW_PATH";
    case MISSING_ODOM:
        return "MISSING_ODOM";
    case MISSING_PATH:
        return "MISSING_PATH";
    case ERROR:
        return "ERROR";
    case SUCCESS_END_PATH:
        return "SUCCESS_END_PATH";
    default:
        return "UNKNOWN";
    }
}

void PathTrackingPlanner::stateMonitor()
{
    updateState(INIT);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (controller_ == nullptr)
    {
        current_state_ = ERROR;
        updateState(ERROR);
    }

    while (running_.load() && controller_ != nullptr)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 / frequency_)));

        if (!controller_->isPathReceived())
        {
            updateState(MISSING_PATH);
            continue;
        }

        if (!controller_->isOdomReceived())
        {
            updateState(MISSING_ODOM);
            continue;
        }

        updateState(FOLLOW_PATH);

        controller_->computeControlCommand();

        if (controller_->isReachGoal())
        {
            updateState(SUCCESS_END_PATH);
        }
    }
}

void PathTrackingPlanner::updateState(PathTrackingState state)
{
    current_state_ = state;
    std::cout << "Current state: " << stateToString(current_state_) << std::endl;
}
