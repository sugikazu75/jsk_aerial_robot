// -*- mode: c++ -*-
#include <aerial_robot_control/control/fwddyn_mpc_control_problem.hpp>

#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
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
                           (dir == 1) ? crocoddyl::DT_CCW : crocoddyl::DT_CW, parameters_.thrust_lb(i),
                           parameters_.thrust_ub(i));
  }

  state_with_thrusts_ = std::make_shared<crocoddyl::StateMultibodyWithThrusts>(state_mb_, rotor_num);
  actuation_ = std::make_shared<crocoddyl::ActuationModelFloatingBaseThrusterRates>(state_with_thrusts_, thrusters);

  initialized_ = true;
  reset();
}

void FwddynMpcControlProblem::reset()
{
  first_run_ = true;
  elapsed_time_ = 0.0;
  shooting_problem_.reset();
  solver_.reset();
  xs_.clear();
  us_.clear();
  com_residuals_.clear();
  state_residuals_.clear();
}

bool FwddynMpcControlProblem::update(const Eigen::VectorXd& x0, const Eigen::Vector3d& com_target,
                                     const Eigen::VectorXd& x_ref, double elapsed_time)
{
  if (!initialized_)
    return false;

  if (first_run_)
  {
    buildMPCProblem(x0, com_target, x_ref);
    solveMPC(parameters_.max_init_iter, true);
    first_run_ = false;
    return true;
  }

  shooting_problem_->set_x0(x0);
  xs_[0] = x0;
  updateReferences(com_target, x_ref);

  elapsed_time_ += elapsed_time;
  if (elapsed_time_ >= parameters_.dt - 1e-9)
  {
    slideHorizon(com_target, x_ref);
    elapsed_time_ -= parameters_.dt;
  }

  solveMPC(parameters_.max_iter);
  return true;
}

std::shared_ptr<crocoddyl::IntegratedActionModelEulerWithThrusts>
FwddynMpcControlProblem::createMPCNode(const Eigen::Vector3d& com_target)
{
  const std::size_t nu = actuation_->get_nu();

  auto contacts = std::make_shared<crocoddyl::ContactModelMultiple>(state_mb_, nu);
  auto costs = std::make_shared<crocoddyl::CostModelSum>(state_mb_, nu);

  auto com_res = std::make_shared<crocoddyl::ResidualModelCoMPosition>(state_mb_, com_target, nu);
  com_residuals_.push_back(com_res);
  auto com_act = std::make_shared<crocoddyl::ActivationModelWeightedQuad>(parameters_.com_track_weight);
  costs->addCost("comTrack", std::make_shared<crocoddyl::CostModelResidual>(state_mb_, com_act, com_res), 1.0);

  auto state_res = std::make_shared<crocoddyl::ResidualModelState>(state_mb_, parameters_.x_ref, nu);
  state_residuals_.push_back(state_res);
  auto state_act = std::make_shared<crocoddyl::ActivationModelWeightedQuad>(parameters_.x_state_weight);
  costs->addCost("stateReg", std::make_shared<crocoddyl::CostModelResidual>(state_mb_, state_act, state_res), 1.0);

  auto ctrl_res = std::make_shared<crocoddyl::ResidualModelControl>(state_mb_, nu);
  costs->addCost("ctrlReg", std::make_shared<crocoddyl::CostModelResidual>(state_mb_, ctrl_res),
                 parameters_.control_weight);

  auto dam = std::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamicsWithThrusts>(
      state_with_thrusts_, actuation_, contacts, costs, 1e-6, false);

  dam->set_thrust_barrier(
      Eigen::VectorXd::Constant(pinocchio_robot_model_->getRotorNum(), parameters_.thrust_barrier_weight),
      parameters_.thrust_lb, parameters_.thrust_ub);

  return std::make_shared<crocoddyl::IntegratedActionModelEulerWithThrusts>(dam, parameters_.dt);
}

void FwddynMpcControlProblem::buildMPCProblem(const Eigen::VectorXd& x0, const Eigen::Vector3d& com_target,
                                              const Eigen::VectorXd& x_ref)
{
  parameters_.x_ref = x_ref;
  com_residuals_.clear();
  state_residuals_.clear();

  std::vector<std::shared_ptr<crocoddyl::ActionModelAbstract>> running_models;
  running_models.reserve(parameters_.num_nodes);
  for (int i = 0; i < parameters_.num_nodes; ++i)
    running_models.push_back(createMPCNode(com_target));

  auto terminal = createMPCNode(com_target);

  shooting_problem_ = std::make_shared<crocoddyl::ShootingProblem>(x0, running_models, terminal);
  solver_ = std::make_shared<crocoddyl::SolverFDDP>(shooting_problem_);

  xs_.assign(parameters_.num_nodes + 1, x0);
  us_.resize(parameters_.num_nodes);
  for (int i = 0; i < parameters_.num_nodes; ++i)
  {
    us_[i] = Eigen::VectorXd::Zero(actuation_->get_nu());
    running_models[i]->quasiStatic(running_models[i]->createData(), us_[i], x0);
  }
}

void FwddynMpcControlProblem::updateReferences(const Eigen::Vector3d& com_target, const Eigen::VectorXd& x_ref)
{
  parameters_.x_ref = x_ref;
  for (auto& cr : com_residuals_)
    cr->set_reference(com_target);
  for (auto& sr : state_residuals_)
    sr->set_reference(x_ref);
}

void FwddynMpcControlProblem::slideHorizon(const Eigen::Vector3d& com_target, const Eigen::VectorXd& x_ref)
{
  parameters_.x_ref = x_ref;
  auto new_terminal = createMPCNode(com_target);

  shooting_problem_->circularAppend(shooting_problem_->get_terminalModel());
  shooting_problem_->updateModel(shooting_problem_->get_T(), new_terminal);

  com_residuals_.erase(com_residuals_.begin());
  state_residuals_.erase(state_residuals_.begin());

  xs_.erase(xs_.begin());
  xs_.push_back(xs_.back());
  us_.erase(us_.begin());
  us_.push_back(us_.back());
}

void FwddynMpcControlProblem::solveMPC(int max_iter, bool verbose)
{
  solver_->solve(xs_, us_, max_iter, verbose);
  xs_ = solver_->get_xs();
  us_ = solver_->get_us();
}

}  // namespace aerial_robot_control
