#include "pt_controller/PathTrackingPlanner.h"

PathTrackingPlanner::PathTrackingPlanner(double frequency)
    : current_state_(INIT), running_(false), frequency_(frequency), controller_(nullptr), state_callback_()
{
}

PathTrackingPlanner::PathTrackingPlanner(double frequency, AbstractPathTrackingController *controller, PathTrackingStateCallback state_callback)
    : current_state_(INIT), running_(false), frequency_(frequency), controller_(controller), state_callback_(state_callback)
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
    case NO_INPUT_DATA:
        return "NO_INPUT_DATA";
    case WAITING_ODOM:
        return "WAITING_ODOM";
    case WAITING_PATH:
        return "WAITING_PATH";
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

    if (controller_ == nullptr)
    {
        current_state_ = ERROR;
        updateState(ERROR);
    }

    while (running_.load() && controller_ != nullptr)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 / frequency_)));

        if (!controller_->isOdomReceived() && !controller_->isPathReceived())
        {
            updateState(NO_INPUT_DATA);
            continue;
        }

        if (!controller_->isPathReceived() && controller_->isOdomReceived())
        {
            updateState(WAITING_PATH);
            continue;
        }

        if (!controller_->isOdomReceived() && controller_->isPathReceived())
        {
            updateState(WAITING_ODOM);
            continue;
        }

        updateState(FOLLOW_PATH);

        controller_->computeControlCommand();

        if (controller_->isReachGoal())
        {
            updateState(SUCCESS_END_PATH);
            controller_->stopRobot();
            running_.store(false);
        }
    }
}

void PathTrackingPlanner::updateState(PathTrackingState state)
{
    current_state_ = state;
    if (state_callback_)
    {
        state_callback_(current_state_);
    }
}
