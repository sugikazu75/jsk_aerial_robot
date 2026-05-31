// -*- mode: c++ -*-
#include <aerial_robot_control/control/fwddyn_mpc_control_problem.hpp>

#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/solvers/box-fddp.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/utils/timer.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn-with-thrusts.hpp>
#include <crocoddyl/multibody/actuations/floating-base-thrust-rates.hpp>
#include <crocoddyl/multibody/contacts/multiple-contacts.hpp>
#include <crocoddyl/multibody/residuals/com-position.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <crocoddyl/multibody/states/multibody-with-thrusts.hpp>

#include <aerial_robot_dynamics/robot_model.h>

#include <cmath>

namespace aerial_robot_control
{

void FwddynMpcControlProblem::initialize(
    const std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel>& pinocchio_robot_model,
    const Parameters& parameters)
{
  pinocchio_robot_model_ = pinocchio_robot_model;
  pin_model_ = pinocchio_robot_model_->getModel();
  parameters_ = parameters;

  const int rotor_num = pinocchio_robot_model_->getRotorNum();

  state_mb_ = std::make_shared<crocoddyl::StateMultibody>(pin_model_);

  std::vector<crocoddyl::DistributedThruster> thrusters;
  const auto& rotor_frame_indices = pinocchio_robot_model_->getRotorFrameIndices();
  const auto& joint_M_rotors = pinocchio_robot_model_->getJointMRotors();
  const double m_f_rate = std::abs(pinocchio_robot_model_->getMFRate());
  for (int i = 0; i < rotor_num; ++i)
  {
    const int dir = pinocchio_robot_model_->getRotorDirection(i);
    thrusters.emplace_back(rotor_frame_indices.at(i), joint_M_rotors.at(i), m_f_rate,
                           (dir == 1) ? crocoddyl::DT_CCW : crocoddyl::DT_CW,
                           pinocchio_robot_model_->getThrustLowerLimits()(i),
                           pinocchio_robot_model_->getThrustUpperLimits()(i), parameters_.delta_thrust_max);
  }

  state_with_thrusts_ = std::make_shared<crocoddyl::StateMultibodyWithThrusts>(state_mb_, rotor_num);
  actuation_ = std::make_shared<crocoddyl::ActuationModelFloatingBaseThrusterRates>(state_with_thrusts_, thrusters);

  initialized_ = true;
  reset();
}

void FwddynMpcControlProblem::reset()
{
  shooting_problem_.reset();
  solver_.reset();
  xs_.clear();
  us_.clear();
  com_residuals_.clear();
  state_residuals_.clear();
}

std::shared_ptr<crocoddyl::IntegratedActionModelEulerWithThrusts>
FwddynMpcControlProblem::createMPCNode(const Eigen::Vector3d& com_target, const Eigen::VectorXd& x_ref)
{
  const std::size_t nu = actuation_->get_nu();

  // contact
  auto contacts = std::make_shared<crocoddyl::ContactModelMultiple>(state_mb_, nu);
  auto costs = std::make_shared<crocoddyl::CostModelSum>(state_mb_, nu);

  // CoM tracking
  auto com_res = std::make_shared<crocoddyl::ResidualModelCoMPosition>(state_mb_, com_target, nu);
  com_residuals_.push_back(com_res);
  auto com_act = std::make_shared<crocoddyl::ActivationModelWeightedQuad>(parameters_.com_track_weight);
  costs->addCost("comTrack", std::make_shared<crocoddyl::CostModelResidual>(state_mb_, com_act, com_res), 1.0);

  // State regularisation
  auto state_res = std::make_shared<crocoddyl::ResidualModelState>(state_mb_, x_ref, nu);
  state_residuals_.push_back(state_res);
  auto state_act = std::make_shared<crocoddyl::ActivationModelWeightedQuad>(parameters_.x_state_weight);
  costs->addCost("stateReg", std::make_shared<crocoddyl::CostModelResidual>(state_mb_, state_act, state_res), 1.0);

  // Control regularisation
  auto ctrl_res = std::make_shared<crocoddyl::ResidualModelControl>(state_mb_, nu);
  costs->addCost("ctrlReg", std::make_shared<crocoddyl::CostModelResidual>(state_mb_, ctrl_res),
                 parameters_.control_weight);

  auto dam = std::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamicsWithThrusts>(
      state_with_thrusts_, actuation_, contacts, costs, 1e-6, false);

  // Thurst regularization
  dam->set_thrust_reg_weight(
      Eigen::VectorXd::Constant(pinocchio_robot_model_->getRotorNum(), parameters_.thrust_reg_weight));

  // Thrust saturation barrier
  dam->set_thrust_barrier(
      Eigen::VectorXd::Constant(pinocchio_robot_model_->getRotorNum(), parameters_.thrust_barrier_weight),
      pinocchio_robot_model_->getThrustLowerLimits(), pinocchio_robot_model_->getThrustUpperLimits());

  return std::make_shared<crocoddyl::IntegratedActionModelEulerWithThrusts>(dam, parameters_.dt);
}

void FwddynMpcControlProblem::buildMPCProblem(const Eigen::VectorXd& x0, const Eigen::Vector3d& com_target,
                                              const Eigen::VectorXd& x_ref)
{
  com_residuals_.clear();
  state_residuals_.clear();

  std::vector<std::shared_ptr<crocoddyl::ActionModelAbstract>> running_models;
  running_models.reserve(parameters_.num_nodes);
  for (int i = 0; i < parameters_.num_nodes; ++i)
    running_models.push_back(createMPCNode(com_target, x_ref));

  auto terminal = createMPCNode(com_target, x_ref);

  shooting_problem_ = std::make_shared<crocoddyl::ShootingProblem>(x0, running_models, terminal);
  solver_ = std::make_shared<crocoddyl::SolverBoxFDDP>(shooting_problem_);

  xs_.assign(parameters_.num_nodes + 1, x0);
  us_.resize(parameters_.num_nodes);
  for (int i = 0; i < parameters_.num_nodes; ++i)
  {
    us_[i] = Eigen::VectorXd::Zero(actuation_->get_nu());
    running_models[i]->quasiStatic(running_models[i]->createData(), us_[i], x0);
  }
}

void FwddynMpcControlProblem::setInitialState(const Eigen::VectorXd& x0)
{
  if (!shooting_problem_ || xs_.empty())
    return;

  shooting_problem_->set_x0(x0);
  xs_[0] = x0;
}

void FwddynMpcControlProblem::slideHorizon(const Eigen::Vector3d& com_target, const Eigen::VectorXd& x_ref)
{
  // retreive the first running node and data (which will become the new terminal node)
  std::shared_ptr<crocoddyl::ActionModelAbstract> new_terminal = shooting_problem_->get_runningModels().front();
  std::shared_ptr<crocoddyl::ActionDataAbstract> new_terminal_data = shooting_problem_->get_runningDatas().front();

  // append a terminal node to the end of running nodes, and remove the first node
  shooting_problem_->circularAppend(shooting_problem_->get_terminalModel(), shooting_problem_->get_terminalData());

  // update the terminal node with the first running node
  shooting_problem_->updateNode(shooting_problem_->get_T(), new_terminal, new_terminal_data);

  // update the references for the new terminal node
  std::shared_ptr<crocoddyl::ResidualModelCoMPosition> new_com_res = com_residuals_.front();
  std::shared_ptr<crocoddyl::ResidualModelState> new_state_res = state_residuals_.front();
  new_com_res->set_reference(com_target);
  new_state_res->set_reference(x_ref);

  com_residuals_.erase(com_residuals_.begin());
  state_residuals_.erase(state_residuals_.begin());

  com_residuals_.push_back(new_com_res);
  state_residuals_.push_back(new_state_res);

  // update the initial guess by shifting the previous solution
  xs_.erase(xs_.begin());
  xs_.push_back(xs_.back());
  us_.erase(us_.begin());
  us_.push_back(us_.back());
}

bool FwddynMpcControlProblem::solveMPC(int max_iter, bool verbose)
{
  if (!solver_)
    return false;

  crocoddyl::Timer timer;
  const bool solved = solver_->solve(xs_, us_, max_iter, false);
  double solve_time = timer.get_duration();

  if (verbose)
  {
    std::cout << "MPC solved: " << solved << std::endl;
    std::cout << "total calculation time:" << solve_time << std::endl;
    std::cout << "Number of iterations: " << solver_->get_iter() << std::endl;
    std::cout << "time per iterate:" << solve_time / solver_->get_iter() << std::endl;
    std::cout << "Total cost: " << solver_->get_cost() << std::endl;
    std::cout << "Gradient norm: " << solver_->get_stop() << std::endl;
    std::cout << std::endl;
  }

  xs_ = solver_->get_xs();
  us_ = solver_->get_us();
  return solved;
}

}  // namespace aerial_robot_control
