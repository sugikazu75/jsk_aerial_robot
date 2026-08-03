// -*- mode: c++ -*-

#include <delta/control/delta_controller.h>
#include <delta/control/delta_wrench_allocation.h>

#include <gtest/gtest.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace aerial_robot_control;

class WrenchAllocationGradientTest : public testing::Test
{
protected:
  static constexpr int m_ = 6;  // number of equality constraints

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  boost::shared_ptr<DeltaRobotModel> robot_model_;
  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator_;
  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator_;
  boost::shared_ptr<DeltaController> controller_;

  std::mt19937 engine_;
  double delta_;
  double gradient_diff_thre_;
  int sample_num_;
  int motor_num_;
  int n_;

  void SetUp() override
  {
    ::testing::Test::SetUp();

    nhp_ = ros::NodeHandle("~");
    nhp_.param("delta", delta_, 1.0e-6);
    nhp_.param("gradient_diff_thre", gradient_diff_thre_, 1.0e-4);
    nhp_.param("sample_num", sample_num_, 10);

    /* the controller only stores the estimator/navigator, so the default instances are enough */
    robot_model_ = boost::make_shared<DeltaRobotModel>();
    estimator_ = boost::make_shared<aerial_robot_estimation::StateEstimator>();
    navigator_ = boost::make_shared<aerial_robot_navigation::BaseNavigator>();

    controller_ = boost::make_shared<DeltaController>();
    controller_->initialize(nh_, nhp_, robot_model_, estimator_, navigator_, 40.0);

    motor_num_ = robot_model_->getRotorNum();
    n_ = 2 * motor_num_;

    engine_.seed(0);  // fixed seed to keep the test reproducible
  }

  /* randomize the robot configuration and the target wrench referred by the callbacks */
  void randomizeControllerState()
  {
    auto robot_model_for_control = controller_->getRobotModelForControl();

    sensor_msgs::JointState joint_state;
    const auto& link_joint_names = robot_model_for_control->getLinkJointNames();
    const auto& lower_limits = robot_model_for_control->getLinkJointLowerLimits();
    const auto& upper_limits = robot_model_for_control->getLinkJointUpperLimits();
    for (int i = 0; i < link_joint_names.size(); i++)
    {
      std::uniform_real_distribution<double> dist(lower_limits.at(i), upper_limits.at(i));
      joint_state.name.push_back(link_joint_names.at(i));
      joint_state.position.push_back(dist(engine_));
    }
    std::uniform_real_distribution<double> gimbal_dist(-M_PI, M_PI);
    for (int i = 0; i < motor_num_; i++)
    {
      joint_state.name.push_back(std::string("gimbal") + std::to_string(i + 1));
      joint_state.position.push_back(gimbal_dist(engine_));
    }
    robot_model_for_control->updateRobotModel(robot_model_for_control->jointMsgToKdl(joint_state));

    /* the target wrench is a constant offset of the constraints, but it must not be an empty vector */
    std::uniform_real_distribution<double> acc_dist(-5.0, 5.0);
    Eigen::VectorXd target_acc_cog = Eigen::VectorXd::Zero(6);
    for (int i = 0; i < 6; i++)
      target_acc_cog(i) = acc_dist(engine_);
    controller_->setTargetAccCog(target_acc_cog);
  }

  std::vector<double> randomVariables()
  {
    std::uniform_real_distribution<double> lambda_dist(0.0, robot_model_->getThrustUpperLimit());
    std::uniform_real_distribution<double> phi_dist(-M_PI, M_PI);

    std::vector<double> x(n_, 0.0);
    for (int i = 0; i < motor_num_; i++)
    {
      x.at(i) = lambda_dist(engine_);
      x.at(i + motor_num_) = phi_dist(engine_);
    }
    return x;
  }

  double evalObjective(const std::vector<double>& x)
  {
    std::vector<double> no_grad;  // empty grad means "value only"
    return nonlinearWrenchAllocationMinObjective(x, no_grad, controller_.get());
  }

  std::vector<double> evalConstraints(const std::vector<double>& x)
  {
    std::vector<double> result(m_, 0.0);
    nonlinearWrenchAllocationEqConstraints(m_, result.data(), n_, x.data(), NULL, controller_.get());
    return result;
  }

  std::string variableName(int j)
  {
    if (j < motor_num_)
      return std::string("lambda") + std::to_string(j + 1);
    return std::string("phi") + std::to_string(j - motor_num_ + 1);
  }
};

TEST_F(WrenchAllocationGradientTest, MinObjectiveGradient)
{
  for (int sample = 0; sample < sample_num_; sample++)
  {
    randomizeControllerState();
    std::vector<double> x = randomVariables();

    /* analytical */
    std::vector<double> analytical_grad(n_, 0.0);
    nonlinearWrenchAllocationMinObjective(x, analytical_grad, controller_.get());

    /* numerical (central difference) */
    for (int j = 0; j < n_; j++)
    {
      std::vector<double> x_plus = x, x_minus = x;
      x_plus.at(j) += delta_;
      x_minus.at(j) -= delta_;

      double numerical_grad = (evalObjective(x_plus) - evalObjective(x_minus)) / (2.0 * delta_);

      EXPECT_NEAR(analytical_grad.at(j), numerical_grad, gradient_diff_thre_)
          << "objective gradient w.r.t. " << variableName(j) << " differs at sample " << sample;
    }
  }
}

TEST_F(WrenchAllocationGradientTest, EqConstraintsGradient)
{
  for (int sample = 0; sample < sample_num_; sample++)
  {
    randomizeControllerState();
    std::vector<double> x = randomVariables();

    /* analytical: row major m x n */
    std::vector<double> result(m_, 0.0);
    std::vector<double> analytical_grad(m_ * n_, 0.0);
    nonlinearWrenchAllocationEqConstraints(m_, result.data(), n_, x.data(), analytical_grad.data(), controller_.get());

    /* numerical (central difference) */
    for (int j = 0; j < n_; j++)
    {
      std::vector<double> x_plus = x, x_minus = x;
      x_plus.at(j) += delta_;
      x_minus.at(j) -= delta_;

      std::vector<double> result_plus = evalConstraints(x_plus);
      std::vector<double> result_minus = evalConstraints(x_minus);

      for (int i = 0; i < m_; i++)
      {
        double numerical_grad = (result_plus.at(i) - result_minus.at(i)) / (2.0 * delta_);

        EXPECT_NEAR(analytical_grad.at(i * n_ + j), numerical_grad, gradient_diff_thre_)
            << "constraint " << i << " gradient w.r.t. " << variableName(j) << " differs at sample " << sample;
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wrench_allocation_gradient_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
