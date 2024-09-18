#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <ros/ros.h>

#include "crocoddyl/core/action-base.hpp"
#include "crocoddyl/core/activations/weighted-quadratic.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/costs/residual.hpp"
#include <crocoddyl/core/diff-action-base.hpp>
#include "crocoddyl/core/integ-action-base.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/core/residuals/control.hpp"
#include <crocoddyl/core/state-base.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/floating-base-thrusters.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include "crocoddyl/multibody/residuals/frame-placement.hpp"
#include "crocoddyl/multibody/residuals/state.hpp"

#include <vector>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "crocoddyl_test");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::string fileName = "/home/leus/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/mini_quadrotor/urdf/robot.urdf";

  boost::shared_ptr<pinocchio::Model> model;
  model = boost::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(fileName, pinocchio::JointModelFreeFlyer(), *model.get(), true);

  ROS_WARN_STREAM("[model][pinocchio] model name: " << model->name);
  ROS_WARN_STREAM("[model][pinocchio] model.nq: " << model->nq);
  ROS_WARN_STREAM("[model][pinocchio] model.nv: " << model->nv);
  ROS_WARN_STREAM("[model][pinocchio] model.njoints: " << model->njoints);

  std::vector<crocoddyl::Thruster> thrusters;
  double rotor_x = 0.085;
  double rotor_z = 0.026;
  double m_f_rate = 0.011;
  pinocchio::SE3 p1(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-rotor_x, -rotor_x, rotor_z));
  pinocchio::SE3 p2(Eigen::Matrix3d::Identity(), Eigen::Vector3d(rotor_x, -rotor_x, rotor_z));
  pinocchio::SE3 p3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(rotor_x, rotor_x, rotor_z));
  pinocchio::SE3 p4(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-rotor_x, rotor_x, rotor_z));
  thrusters.push_back(crocoddyl::Thruster(p1, m_f_rate, crocoddyl::ThrusterType::CCW));
  thrusters.push_back(crocoddyl::Thruster(p2, m_f_rate, crocoddyl::ThrusterType::CW));
  thrusters.push_back(crocoddyl::Thruster(p3, m_f_rate, crocoddyl::ThrusterType::CW));
  thrusters.push_back(crocoddyl::Thruster(p4, m_f_rate, crocoddyl::ThrusterType::CCW));

  boost::shared_ptr<crocoddyl::StateMultibody> state = boost::make_shared<crocoddyl::StateMultibody>(model);
  boost::shared_ptr<crocoddyl::ActuationModelFloatingBaseThrusters> actuation = boost::make_shared<crocoddyl::ActuationModelFloatingBaseThrusters>(state, thrusters);

  int nu = actuation->get_nu();
  ROS_WARN_STREAM("[actuation] actuation->get_nu(): " << nu);
  boost::shared_ptr<crocoddyl::CostModelSum> running_cost_model = boost::make_shared<crocoddyl::CostModelSum>(state, nu);
  boost::shared_ptr<crocoddyl::CostModelSum> terminal_cost_model = boost::make_shared<crocoddyl::CostModelSum>(state, nu);


  // costs
  boost::shared_ptr<crocoddyl::ResidualModelState> x_residual = boost::make_shared<crocoddyl::ResidualModelState>(state, state->zero(), nu);
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> x_activation = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(Eigen::VectorXd::Zero(20)); // todo set correct dimension
  std::cout << "x residual" << std::endl;

  boost::shared_ptr<crocoddyl::ResidualModelControl> u_residual = boost::make_shared<crocoddyl::ResidualModelControl>(state, nu);
  std::cout << "u residual" << std::endl;
  boost::shared_ptr<crocoddyl::CostModelResidual> x_reg_cost = boost::make_shared<crocoddyl::CostModelResidual>(state, x_activation, x_residual);
  std::cout << "x reg cost" << std::endl;
  boost::shared_ptr<crocoddyl::CostModelResidual> u_reg_cost = boost::make_shared<crocoddyl::CostModelResidual>(state, u_residual);
  std::cout << "u reg cost" << std::endl;
  boost::shared_ptr<crocoddyl::ResidualModelFramePlacement> goal_tracking_residual = boost::make_shared<crocoddyl::ResidualModelFramePlacement>(state, model->getFrameId("fc"), pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 1.0)), nu);
  std::cout << "tracking cost" << std::endl;

  boost::shared_ptr<crocoddyl::CostModelResidual> goal_tracking_cost = boost::make_shared<crocoddyl::CostModelResidual>(state, goal_tracking_residual);

  running_cost_model->addCost("xReg", x_reg_cost, 1e-6);
  running_cost_model->addCost("uREg", u_reg_cost, 1e-6);
  running_cost_model->addCost("trackPose", goal_tracking_cost, 1e-2);
  terminal_cost_model->addCost("goalPose", goal_tracking_cost, 3.0);


  double dt = 3e-2;

  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> running_model = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation, running_cost_model), dt);

  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> terminal_model = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation, terminal_cost_model), dt);

  int t = 33;

  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract> > running_models(t, running_model);

  const Eigen::VectorXd& x0 = Eigen::VectorXd::Zero(25); // todo: set correct dimension
  boost::shared_ptr<crocoddyl::ShootingProblem> problem = boost::make_shared<crocoddyl::ShootingProblem>(x0, running_models, terminal_model);

}
