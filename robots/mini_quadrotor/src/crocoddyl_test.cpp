#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "crocoddyl/core/action-base.hpp"
#include "crocoddyl/core/activations/weighted-quadratic.hpp"
#include "crocoddyl/core/activations/quadratic-barrier.hpp"
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
#include "crocoddyl/multibody/residuals/com-position.hpp"
#include "crocoddyl/multibody/residuals/frame-placement.hpp"
#include "crocoddyl/multibody/residuals/frame-rotation.hpp"
#include "crocoddyl/multibody/residuals/state.hpp"

#include <vector>
#include <limits>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "crocoddyl_test");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  tf::TransformBroadcaster robot_base_broadcaster;

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
  int rotor_num = 4;
  double rotor_x = 0.085;
  double rotor_z = 0.026;
  double m_f_rate = 0.011;

  Eigen::Matrix3d R1, R2, R3, R4;
  R1 = Eigen::AngleAxisd(0., Eigen::Vector3d::UnitX());
  R2 = Eigen::AngleAxisd(0., Eigen::Vector3d::UnitY());
  R3 = Eigen::AngleAxisd(0., Eigen::Vector3d::UnitX());
  R4 = Eigen::AngleAxisd(0., Eigen::Vector3d::UnitY());

  pinocchio::SE3 p1(R1, Eigen::Vector3d(-rotor_x, -rotor_x, rotor_z));
  pinocchio::SE3 p2(R2, Eigen::Vector3d(rotor_x, -rotor_x, rotor_z));
  pinocchio::SE3 p3(R3, Eigen::Vector3d(rotor_x, rotor_x, rotor_z));
  pinocchio::SE3 p4(R4, Eigen::Vector3d(-rotor_x, rotor_x, rotor_z));
  thrusters.push_back(crocoddyl::Thruster(p1, m_f_rate, crocoddyl::ThrusterType::CCW));
  thrusters.push_back(crocoddyl::Thruster(p2, m_f_rate, crocoddyl::ThrusterType::CW));
  thrusters.push_back(crocoddyl::Thruster(p3, m_f_rate, crocoddyl::ThrusterType::CW));
  thrusters.push_back(crocoddyl::Thruster(p4, m_f_rate, crocoddyl::ThrusterType::CCW));

  boost::shared_ptr<crocoddyl::StateMultibody> state = boost::make_shared<crocoddyl::StateMultibody>(model);
  ROS_WARN_STREAM("[crocoddyl][state] " << "state->get_ndx(): " << state->get_ndx());
  ROS_WARN_STREAM("[crocoddyl][state] " << "state->get_nx(): " << state->get_nx());
  ROS_WARN_STREAM("[crocoddyl][state] " << "state->get_nq(): " << state->get_nq());
  ROS_WARN_STREAM("[crocoddyl][state] " << "state->get_nv(): " << state->get_nv());

  boost::shared_ptr<crocoddyl::ActuationModelFloatingBaseThrusters> actuation = boost::make_shared<crocoddyl::ActuationModelFloatingBaseThrusters>(state, thrusters);
  int nu = actuation->get_nu();
  ROS_WARN_STREAM("[crocoddyl][actuation] actuation->get_nu(): " << nu);
  std::cout << "W_thrust: \n" << actuation->get_Wthrust() << std::endl;

  boost::shared_ptr<crocoddyl::CostModelSum> running_cost_model = boost::make_shared<crocoddyl::CostModelSum>(state, nu);
  boost::shared_ptr<crocoddyl::CostModelSum> terminal_cost_model = boost::make_shared<crocoddyl::CostModelSum>(state, nu);

  // costs
  // state point
  Eigen::VectorXd xref = Eigen::VectorXd::Zero(state->get_nq() + state->get_nv());
  xref(2) = 1.0;
  xref(6) = 1.0;
  boost::shared_ptr<crocoddyl::ResidualModelAbstract> x_residual = boost::make_shared<crocoddyl::ResidualModelState>(state, xref, nu);
  Eigen::VectorXd x_activation_weights(state->get_nv() * 2);
  {
    std::vector<double> whatever(state->get_nv() * 2);
    nhp.getParam("x_state_weight", whatever);
    for(int i = 0; i < state->get_nv() * 2; i++)
      x_activation_weights(i) = whatever.at(i);
  }
  std::cout << "x_activation_weights: " << x_activation_weights.transpose() << std::endl;
  boost::shared_ptr<crocoddyl::CostModelAbstract> x_reg_cost = boost::make_shared<crocoddyl::CostModelResidual>(state, boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(x_activation_weights), x_residual);

  // thrust input
  boost::shared_ptr<crocoddyl::ResidualModelAbstract> u_residual = boost::make_shared<crocoddyl::ResidualModelControl>(state, nu);
  boost::shared_ptr<crocoddyl::ActivationModelAbstract> u_activation = boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(crocoddyl::ActivationBounds(Eigen::VectorXd::Zero(rotor_num), 8.0 * Eigen::VectorXd::Ones(rotor_num)));
  boost::shared_ptr<crocoddyl::CostModelAbstract> u_reg_cost = boost::make_shared<crocoddyl::CostModelResidual>(state, u_activation, u_residual);

  // goal tracking
  boost::shared_ptr<crocoddyl::ResidualModelAbstract> goal_tracking_residual = boost::make_shared<crocoddyl::ResidualModelFramePlacement>(state, model->getFrameId("fc"), pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 1.0)), nu);
  boost::shared_ptr<crocoddyl::CostModelAbstract> goal_tracking_cost = boost::make_shared<crocoddyl::CostModelResidual>(state, goal_tracking_residual);

  // frame rotation
  boost::shared_ptr<crocoddyl::ResidualModelAbstract> frame_rotation_residual = boost::make_shared<crocoddyl::ResidualModelFrameRotation>(state, model->getFrameId("fc"), Eigen::Matrix3d::Identity(), nu);
  boost::shared_ptr<crocoddyl::CostModelAbstract> frame_rotation_cost = boost::make_shared<crocoddyl::CostModelResidual>(state, frame_rotation_residual);

  double x_reg_weight, x_reg_final_weight, u_reg_weight, goal_tracking_weight, rotation_tracking_weight, terminal_goal_tracking_weight, terminal_rotation_tracking_weight;
  nhp.getParam("x_reg_weight", x_reg_weight);
  nhp.getParam("x_reg_final_weight", x_reg_final_weight);
  nhp.getParam("u_reg_weight", u_reg_weight);
  nhp.getParam("goal_tracking_weight", goal_tracking_weight);
  nhp.getParam("rotation_tracking_weight", rotation_tracking_weight);
  nhp.getParam("terminal_goal_tracking_weight", terminal_goal_tracking_weight);
  nhp.getParam("terminal_rotation_tracking_weight", terminal_rotation_tracking_weight);

  running_cost_model->addCost("xReg", x_reg_cost, x_reg_weight);
  running_cost_model->addCost("uReg", u_reg_cost, u_reg_weight);
  running_cost_model->addCost("goalTracking", goal_tracking_cost, goal_tracking_weight);
  running_cost_model->addCost("rotationTracking", frame_rotation_cost, rotation_tracking_weight);
  terminal_cost_model->addCost("xReg_final", x_reg_cost, x_reg_final_weight);
  terminal_cost_model->addCost("terminalGoalTracking", goal_tracking_cost, terminal_goal_tracking_weight);
  terminal_cost_model->addCost("terminalRotationTracking", frame_rotation_cost, terminal_rotation_tracking_weight);

  double horizon, dt;
  nhp.getParam("horizon", horizon);
  nhp.getParam("dt", dt);

  int N = horizon / dt;

  boost::shared_ptr<crocoddyl::ActionModelAbstract> running_model = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation, running_cost_model), dt);
  boost::shared_ptr<crocoddyl::ActionModelAbstract> terminal_model = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation, terminal_cost_model), dt);

  double epsilon = 0.000001;
  Eigen::VectorXd u_lb = Eigen::VectorXd::Zero(nu);
  Eigen::VectorXd u_ub = Eigen::VectorXd::Zero(nu);
  for(int i = 0; i < rotor_num; i++)
    {
      u_lb(i) = 0;
      u_ub(i) = 8;
    }
  running_model->set_u_lb(u_lb);
  running_model->set_u_ub(u_ub);
  terminal_model->set_u_lb(u_lb);
  terminal_model->set_u_ub(u_ub);

  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract> > running_models(N, running_model);

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model->nq + model->nv);
  x0(6) = 1.0;
  bool hovering_approximate;
  nhp.getParam("hovering_approximate", hovering_approximate);
  if(hovering_approximate)
    {
      x0(2) = 1.0;
    }

  boost::shared_ptr<crocoddyl::ShootingProblem> problem = boost::make_shared<crocoddyl::ShootingProblem>(x0, running_models, terminal_model);

  boost::shared_ptr<crocoddyl::SolverAbstract> solver = boost::make_shared<crocoddyl::SolverFDDP>(problem);

  int num_threads = 1;
  nhp.getParam("num_threads", num_threads);
  solver->get_problem()->set_nthreads(num_threads);

  std::vector<Eigen::VectorXd> xs_init(N, x0);
  std::vector<Eigen::VectorXd> us_init = solver->get_problem()->quasiStatic_xs(xs_init);
  xs_init.push_back(x0);
  
  while(ros::ok())
  {
    ros::Rate loop_rate(1.0 / dt);

    crocoddyl::Timer timer;
    solver->solve(xs_init, us_init);
    double time = timer.get_duration();
    std::cout << "total calculation time: " << time << "[ms]" << std::endl;
    std::cout << "Number of iterations: " << solver->get_iter() << std::endl;
    std::cout << "time per iterate: " << time / solver->get_iter() << std::endl;
    std::cout << "Total cost: " << solver->get_cost() << std::endl;
    std::cout << "Gradient norm: " << solver->get_stop() << std::endl;

    xs_init = solver->get_xs();
    us_init = solver->get_us();

    problem->set_x0(xs_init.at(1));

    std::cout << xs_init.at(1).transpose() << std::endl;
    std::cout << us_init.at(0).transpose() << std::endl;
    std::cout << std::endl;

    Eigen::VectorXd q = xs_init.at(0);
    geometry_msgs::TransformStamped baseState;
    baseState.header.stamp = ros::Time::now();
    baseState.header.frame_id = "world";
    baseState.child_frame_id  = "BODY";
    baseState.transform.translation.x = q[0];
    baseState.transform.translation.y = q[1];
    baseState.transform.translation.z = q[2];
    baseState.transform.rotation.x = q[3];
    baseState.transform.rotation.y = q[4];
    baseState.transform.rotation.z = q[5];
    baseState.transform.rotation.w = q[6];
    robot_base_broadcaster.sendTransform(baseState);
  }

  // for(int i = 0; i < xs.size(); i++)
  //   {
  //     std::cout << xs.at(i).transpose() << std::endl;
  //   }
  // std::cout << std::endl;
  // for(int i = 0; i < us.size(); i++)
  //   {
  //     std::cout << us.at(i).transpose() << std::endl;
  //   }

}
