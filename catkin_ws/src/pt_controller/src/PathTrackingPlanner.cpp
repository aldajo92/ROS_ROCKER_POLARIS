#include "pt_controller/PathTrackingPlanner.h"
#include <iostream>
#include <chrono>
#include <thread>

PathTrackingPlanner::PathTrackingPlanner()
    : current_state_(INIT), running_(false), controller_(nullptr)
{
}

PathTrackingPlanner::PathTrackingPlanner(PathTrackingController *controller)
    : current_state_(INIT), running_(false), controller_(controller)
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

PlannerState PathTrackingPlanner::getState() const
{
    return current_state_;
}

std::string PathTrackingPlanner::stateToString(PlannerState state) const
{
    switch (state)
    {
    case INIT:
        return "INIT";
    case FOLLOW_PATH:
        return "FOLLOW_PATH";
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
    while (running_.load())
    {
        current_state_ = INIT;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Current state: " << stateToString(current_state_) << std::endl;

        current_state_ = FOLLOW_PATH;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Current state: " << stateToString(current_state_) << std::endl;

        current_state_ = ERROR;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Current state: " << stateToString(current_state_) << std::endl;

        current_state_ = SUCCESS_END_PATH;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Current state: " << stateToString(current_state_) << std::endl;
    }
}