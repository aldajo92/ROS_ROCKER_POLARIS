#include <gtest/gtest.h>
#include "pt_controller/PathTrackingPlanner.h"
#include "pt_controller/PathTrackingController.h"

class MockPathTrackingController : public AbstractPathTrackingController
{
public:
    void setPath(const nav_msgs::Path &path) override {}
    void setOdom(const nav_msgs::Odometry &odom) override {}
    void computeControlCommand() override {}
    void stopRobot() override {}
    bool isReachGoal() const override { return reach_goal_; }
    bool isOdomReceived() const override { return odom_received_; }
    bool isPathReceived() const override { return path_received_; }

    void setReachGoal(bool value) { reach_goal_ = value; }
    void setOdomReceived(bool value) { odom_received_ = value; }
    void setPathReceived(bool value) { path_received_ = value; }

private:
    bool reach_goal_ = false;
    bool odom_received_ = false;
    bool path_received_ = false;
};

TEST(PathTrackingPlannerTest, InitialState)
{
    MockPathTrackingController mock_controller;
    PathTrackingPlanner planner(10.0, &mock_controller, nullptr);

    EXPECT_EQ(planner.getState(), INIT);
}

TEST(PathTrackingPlannerTest, NoInputDataState)
{
    MockPathTrackingController mock_controller;
    PathTrackingPlanner planner(10.0, &mock_controller, nullptr);

    mock_controller.setPathReceived(false);
    mock_controller.setOdomReceived(false);

    planner.execute();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    planner.stop();

    EXPECT_EQ(planner.getState(), NO_INPUT_DATA);
}

TEST(PathTrackingPlannerTest, WaitingPathState)
{
    MockPathTrackingController mock_controller;
    PathTrackingPlanner planner(10.0, &mock_controller, nullptr);

    mock_controller.setPathReceived(false);
    mock_controller.setOdomReceived(true);

    planner.execute();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    planner.stop();

    EXPECT_EQ(planner.getState(), WAITING_PATH);
}

TEST(PathTrackingPlannerTest, WaitingOdomState)
{
    MockPathTrackingController mock_controller;
    PathTrackingPlanner planner(10.0, &mock_controller, nullptr);

    mock_controller.setPathReceived(true);
    mock_controller.setOdomReceived(false);

    planner.execute();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    planner.stop();

    EXPECT_EQ(planner.getState(), WAITING_ODOM);
}

TEST(PathTrackingPlannerTest, FollowPathState)
{
    MockPathTrackingController mock_controller;
    PathTrackingPlanner planner(10.0, &mock_controller, nullptr);

    mock_controller.setPathReceived(true);
    mock_controller.setOdomReceived(true);

    planner.execute();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    planner.stop();

    EXPECT_EQ(planner.getState(), FOLLOW_PATH);
}

TEST(PathTrackingPlannerTest, ReachGoalState)
{
    MockPathTrackingController mock_controller;
    PathTrackingPlanner planner(10.0, &mock_controller, nullptr);

    mock_controller.setPathReceived(true);
    mock_controller.setOdomReceived(true);
    mock_controller.setReachGoal(true);

    planner.execute();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    planner.stop();

    EXPECT_EQ(planner.getState(), SUCCESS_END_PATH);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
