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
#include "crocoddyl/core/solvers/fddp.hpp"
#include <crocoddyl/core/state-base.hpp>
#include "crocoddyl/core/utils/timer.hpp"
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
  ros::NodeHandle nhp("~");
  std::string fileName = "";
  nhp.getParam("filename", fileName);

  boost::shared_ptr<pinocchio::Model> model;
  model = boost::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(fileName, pinocchio::JointModelFreeFlyer(), *model.get(), true);

  ROS_WARN_STREAM("[model][pinocchio] model name: " << model->name);
  ROS_WARN_STREAM("[model][pinocchio] model.nq: " << model->nq);
  ROS_WARN_STREAM("[model][pinocchio] model.nv: " << model->nv);
  ROS_WARN_STREAM("[model][pinocchio] model.njoints: " << model->njoints);
  for (size_t i = 0; i < model->joints.size(); ++i) {
    std::cout << "Joint " << i << ": " << model->names[i]
              << ", Type: " << model->joints[i].shortname() << std::endl;
  }

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
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> x_activation = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(Eigen::VectorXd::Zero(20)); // maybe nx(12) + nu(8)
  boost::shared_ptr<crocoddyl::CostModelResidual> x_reg_cost = boost::make_shared<crocoddyl::CostModelResidual>(state, x_activation, x_residual);

  boost::shared_ptr<crocoddyl::ResidualModelControl> u_residual = boost::make_shared<crocoddyl::ResidualModelControl>(state, nu);
  boost::shared_ptr<crocoddyl::CostModelResidual> u_reg_cost = boost::make_shared<crocoddyl::CostModelResidual>(state, u_residual);

  boost::shared_ptr<crocoddyl::ResidualModelFramePlacement> goal_tracking_residual = boost::make_shared<crocoddyl::ResidualModelFramePlacement>(state, model->getFrameId("fc"), pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 1.0)), nu);
  boost::shared_ptr<crocoddyl::CostModelResidual> goal_tracking_cost = boost::make_shared<crocoddyl::CostModelResidual>(state, goal_tracking_residual);

  running_cost_model->addCost("xReg", x_reg_cost, 1e-6);
  running_cost_model->addCost("uREg", u_reg_cost, 1e-6);
  running_cost_model->addCost("trackPose", goal_tracking_cost, 1e-2);
  terminal_cost_model->addCost("goalPose", goal_tracking_cost, 3.0);

  double dt = 3e-2;
  int t = 33;

  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> running_model = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation, running_cost_model), dt);
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> terminal_model = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation, terminal_cost_model), dt);

  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract> > running_models(t, running_model);

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(25); // model->nq + model->nv
  bool hovering_approximate;
  nhp.getParam("hovering_approximate", hovering_approximate);
  if(hovering_approximate)
    {
      x0(2) = 1.0;
      x0(6) = 1.0;
    }

  boost::shared_ptr<crocoddyl::ShootingProblem> problem = boost::make_shared<crocoddyl::ShootingProblem>(x0, running_models, terminal_model);

  boost::shared_ptr<crocoddyl::SolverFDDP> solver = boost::make_shared<crocoddyl::SolverFDDP>(problem);
  std::vector<Eigen::VectorXd> xs_init(t, x0);
  std::vector<Eigen::VectorXd> us_init = solver->get_problem()->quasiStatic_xs(xs_init);
  xs_init.push_back(x0);

  int num_threads = 1;
  nhp.getParam("num_threads", num_threads);
  solver->get_problem()->set_nthreads(num_threads);

  crocoddyl::Timer timer;
  solver->solve(xs_init, us_init);
  double time = timer.get_duration();
  std::cout << "total calculation time: " << time << "[ms]" << std::endl;
  std::cout << "Number of iterations: " << solver->get_iter() << std::endl;
  std::cout << "time per iterate :" << time / solver->get_iter() << std::endl;
  std::cout << "Total cost: " << solver->get_cost() << std::endl;
  std::cout << "Gradient norm: " << solver->get_stop() << std::endl;

  std::vector<Eigen::VectorXd> xs = solver->get_xs();
  std::vector<Eigen::VectorXd> us = solver->get_us();

  for(int i = 0; i < xs.size(); i++)
    {
      std::cout << xs.at(i).transpose() << std::endl;
    }
  std::cout << std::endl;
  for(int i = 0; i < us.size(); i++)
    {
      std::cout << us.at(i).transpose() << std::endl;
    }
}
