#include <gtest/gtest.h>
#include "pt_controller/PathTrackingController.h"

// Test fixture class
class PathTrackingControllerTest : public ::testing::Test
{
protected:
  PathTrackingController *controller;

  void SetUp() override
  {
    controller = new PathTrackingController(1.8, 2.0, 2.0, 0.1, [](double, double) {});
  }

  void TearDown() override
  {
    delete controller;
  }
};

// Test for calculateMinDistanceInPath
TEST_F(PathTrackingControllerTest, CalculateMinDistanceInPath)
{
  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose1, pose2, pose3;

  pose1.pose.position.x = 1.0;
  pose1.pose.position.y = 1.0;
  path.poses.push_back(pose1);

  pose2.pose.position.x = 2.0;
  pose2.pose.position.y = 2.0;
  path.poses.push_back(pose2);

  pose3.pose.position.x = 3.0;
  pose3.pose.position.y = 3.0;
  path.poses.push_back(pose3);

  std::vector<double> position = {2.5, 2.5};
  auto result = controller->calculateMinDistanceInPath(position, path);

  EXPECT_NEAR(result.first, 0.707107, 0.001);
  EXPECT_EQ(result.second, 1);
}

// Test for computeControlAction
TEST_F(PathTrackingControllerTest, ComputeControlAction)
{
  std::vector<double> position = {0.0, 0.0, 0.0};
  std::vector<double> lookahead_point = {1.0, 1.0};

  auto control = controller->computeControlAction(position, lookahead_point);

  EXPECT_NEAR(control[0], 1.847759, 1e-2);                          // Linear speed
  EXPECT_NEAR(control[1], 1.571348, 1e-2);                          // Angular speed
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
