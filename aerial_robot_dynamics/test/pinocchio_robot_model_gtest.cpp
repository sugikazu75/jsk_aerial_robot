#include <gtest/gtest.h>

#include <aerial_robot_dynamics/robot_model_ros.h>
#include <aerial_robot_dynamics/robot_model_test.h>
#include <ros/ros.h>

#include <memory>
#include <string>

namespace
{
bool waitForParam(const std::string& param_name, const ros::WallDuration& timeout)
{
  const ros::WallTime start = ros::WallTime::now();
  while (ros::ok() && (ros::WallTime::now() - start) < timeout)
  {
    if (ros::param::has(param_name))
      return true;
    ros::WallDuration(0.05).sleep();
  }
  return ros::param::has(param_name);
}
}  // namespace

class PinocchioRobotModelGTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_TRUE(waitForParam("/robot_description", ros::WallDuration(10.0)));
    ASSERT_TRUE(waitForParam("/pinocchio_robot_description", ros::WallDuration(10.0)));

    ros::NodeHandle nh;
    robot_model_ros_ = std::make_unique<aerial_robot_dynamics::PinocchioRobotModelRos>(nh);
    robot_model_ = robot_model_ros_->getPinocchioRobotModel();
    ASSERT_NE(robot_model_, nullptr);
  }

  std::unique_ptr<aerial_robot_dynamics::PinocchioRobotModelRos> robot_model_ros_;
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> robot_model_;
};

TEST_F(PinocchioRobotModelGTest, ForwardDynamics)
{
  constexpr int kTrials = 3;
  for (int i = 0; i < kTrials; ++i)
  {
    EXPECT_TRUE(aerial_robot_dynamics::forwardDynamicsTest(*robot_model_, false)) << "trial=" << i;
  }
}

TEST_F(PinocchioRobotModelGTest, ForwardDynamicsDerivatives)
{
  constexpr int kTrials = 3;
  for (int i = 0; i < kTrials; ++i)
  {
    EXPECT_TRUE(aerial_robot_dynamics::forwardDynamicsDerivativesTest(*robot_model_, false)) << "trial=" << i;
  }
}

// TEST_F(PinocchioRobotModelGTest, InverseDynamics)
// {
//   constexpr int kTrials = 3;
//   for (int i = 0; i < kTrials; ++i)
//   {
//     EXPECT_TRUE(aerial_robot_dynamics::inverseDynamicsTest(*robot_model_, false)) << "trial=" << i;
//   }
// }

TEST_F(PinocchioRobotModelGTest, TauExtByThrustDerivativeQDerivatives)
{
  constexpr int kTrials = 3;
  for (int i = 0; i < kTrials; ++i)
  {
    EXPECT_TRUE(aerial_robot_dynamics::computeTauExtByThrustDerivativeQDerivativesTest(*robot_model_, false))
        << "trial=" << i;
  }
}

TEST_F(PinocchioRobotModelGTest, TauExtByThrustQDerivative)
{
  constexpr int kTrials = 3;
  for (int i = 0; i < kTrials; ++i)
  {
    EXPECT_TRUE(aerial_robot_dynamics::computeTauExtByThrustQDerivativeTest(*robot_model_, false)) << "trial=" << i;
  }
}

TEST_F(PinocchioRobotModelGTest, TauExtByThrustQDerivativeComparison)
{
  constexpr int kTrials = 3;
  for (int i = 0; i < kTrials; ++i)
  {
    EXPECT_TRUE(aerial_robot_dynamics::computeTauExtByThrustQDerivativeComparisonTest(*robot_model_, false))
        << "trial=" << i;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pinocchio_robot_model_gtest");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
