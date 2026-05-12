#include <gtest/gtest.h>

#include <aerial_robot_dynamics/robot_model_test.h>
#include <ros/ros.h>

namespace
{
void ensureRosInitialized()
{
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = nullptr;
    ros::init(argc, argv, "pinocchio_robot_model_gtest", ros::init_options::AnonymousName);
  }
}
}  // namespace

class PinocchioRobotModelGTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ensureRosInitialized();
    test_model_ = std::make_unique<aerial_robot_dynamics::PinocchioRobotModelTest>(false);
  }

  std::unique_ptr<aerial_robot_dynamics::PinocchioRobotModelTest> test_model_;
};

TEST_F(PinocchioRobotModelGTest, ForwardDynamics)
{
  constexpr int kTrials = 3;
  for (int i = 0; i < kTrials; ++i)
  {
    EXPECT_TRUE(test_model_->forwardDynamicsTest(false)) << "trial=" << i;
  }
}

TEST_F(PinocchioRobotModelGTest, ForwardDynamicsDerivatives)
{
  constexpr int kTrials = 3;
  for (int i = 0; i < kTrials; ++i)
  {
    EXPECT_TRUE(test_model_->forwardDynamicsDerivativesTest(false)) << "trial=" << i;
  }
}

// TEST_F(PinocchioRobotModelGTest, InverseDynamics)
// {
//   constexpr int kTrials = 3;
//   for (int i = 0; i < kTrials; ++i)
//   {
//     EXPECT_TRUE(test_model_->inverseDynamicsTest(false)) << "trial=" << i;
//   }
// }

TEST_F(PinocchioRobotModelGTest, TauExtByThrustDerivativeQDerivatives)
{
  constexpr int kTrials = 3;
  for (int i = 0; i < kTrials; ++i)
  {
    EXPECT_TRUE(test_model_->computeTauExtByThrustDerivativeQDerivativesTest(false)) << "trial=" << i;
  }
}

TEST_F(PinocchioRobotModelGTest, TauExtByThrustQDerivative)
{
  constexpr int kTrials = 3;
  for (int i = 0; i < kTrials; ++i)
  {
    EXPECT_TRUE(test_model_->computeTauExtByThrustQDerivativeTest(false)) << "trial=" << i;
  }
}

TEST_F(PinocchioRobotModelGTest, TauExtByThrustQDerivativeComparison)
{
  constexpr int kTrials = 3;
  for (int i = 0; i < kTrials; ++i)
  {
    EXPECT_TRUE(test_model_->computeTauExtByThrustQDerivativeComparisonTest(false)) << "trial=" << i;
  }
}
