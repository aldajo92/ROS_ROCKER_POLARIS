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

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
