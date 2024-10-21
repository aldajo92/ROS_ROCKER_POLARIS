#include "pt_controller/PathTrackingPlanner.h"

PathTrackingPlanner::PathTrackingPlanner(double frequency)
    : current_state_(PathTrackingState::INIT), running_(false), frequency_(frequency), controller_(nullptr), state_callback_()
{
}

PathTrackingPlanner::PathTrackingPlanner(double frequency, AbstractPathTrackingController *controller, PathTrackingStateCallback state_callback)
    : current_state_(PathTrackingState::INIT), running_(false), frequency_(frequency), controller_(controller), state_callback_(state_callback)
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
    case PathTrackingState::INIT:
        return "INIT";
    case PathTrackingState::FOLLOW_PATH:
        return "FOLLOW_PATH";
    case PathTrackingState::NO_INPUT_DATA:
        return "NO_INPUT_DATA";
    case PathTrackingState::WAITING_ODOM:
        return "WAITING_ODOM";
    case PathTrackingState::WAITING_PATH:
        return "WAITING_PATH";
    case PathTrackingState::ERROR:
        return "ERROR";
    case PathTrackingState::SUCCESS_END_PATH:
        return "SUCCESS_END_PATH";
    default:
        return "UNKNOWN";
    }
}

PathTrackingState PathTrackingPlanner::calculateNextState(){
    if (!controller_->isOdomReceived() && !controller_->isPathReceived())
    {
        return PathTrackingState::NO_INPUT_DATA;
    }

    if (!controller_->isPathReceived() && controller_->isOdomReceived())
    {
        return PathTrackingState::WAITING_PATH;
    }

    if (!controller_->isOdomReceived() && controller_->isPathReceived())
    {
        return PathTrackingState::WAITING_ODOM;
    }

    if(controller_->isReachGoal()){
        return PathTrackingState::SUCCESS_END_PATH;
    }

    return PathTrackingState::FOLLOW_PATH;
}

void PathTrackingPlanner::stateMonitor()
{
    PathTrackingState current_state = PathTrackingState::INIT;
    updateState(current_state);

    if (controller_ == nullptr)
    {
        current_state = PathTrackingState::ERROR;
        updateState(current_state);
    }

    while (running_.load() && controller_ != nullptr)
    {
        current_state = calculateNextState();
        updateState(current_state);

        switch (current_state)
        {
            case PathTrackingState::INIT:
                break;
            case PathTrackingState::FOLLOW_PATH:
                controller_->computeControlCommand();
                break;
            case PathTrackingState::NO_INPUT_DATA:
                break;
            case PathTrackingState::WAITING_ODOM:
                break;
            case PathTrackingState::WAITING_PATH:
                break;
            case PathTrackingState::ERROR:
                break;
            case PathTrackingState::SUCCESS_END_PATH:
                controller_->stopRobot();
                running_.store(false);
                break;
            default:
                break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 / frequency_)));
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
