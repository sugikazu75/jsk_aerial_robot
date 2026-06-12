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
#include <crocoddyl/core/solvers/intro.hpp>
#include <crocoddyl/core/solvers/hpipm-sqp.hpp>
#include <crocoddyl/core/utils/timer.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn-with-thrusts.hpp>
#include <crocoddyl/multibody/actuations/floating-base-thrust-rates.hpp>
#include <crocoddyl/multibody/contacts/multiple-contacts.hpp>
#include <crocoddyl/multibody/residuals/centroidal-momentum.hpp>
#include <crocoddyl/multibody/residuals/com-position.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <crocoddyl/multibody/states/multibody-with-thrusts.hpp>

#include <aerial_robot_dynamics/robot_model.h>

#include <cmath>

namespace aerial_robot_control
{

namespace
{
std::vector<Eigen::VectorXd>
toDoubleTrajectory(const std::vector<Eigen::Matrix<FwddynMpcControlProblem::Scalar, Eigen::Dynamic, 1>>& trajectory)
{
  std::vector<Eigen::VectorXd> result;
  result.reserve(trajectory.size());
  for (const auto& value : trajectory)
    result.push_back(value.template cast<double>());
  return result;
}

}  // namespace

void FwddynMpcControlProblem::initialize(
    const std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel>& pinocchio_robot_model,
    const Parameters& parameters)
{
  pinocchio_robot_model_ = pinocchio_robot_model;
  pin_model_ = pinocchio_robot_model_->getModel();
  pin_model_f_ = std::make_shared<pinocchio::ModelTpl<Scalar>>(pin_model_->cast<Scalar>());
  parameters_ = parameters;

  const int rotor_num = pinocchio_robot_model_->getRotorNum();

  state_mb_ = std::make_shared<crocoddyl::StateMultibodyTpl<Scalar>>(pin_model_f_);

  std::vector<crocoddyl::DistributedThrusterTpl<Scalar>> thrusters;
  const auto& rotor_frame_indices = pinocchio_robot_model_->getRotorFrameIndices();
  const auto& joint_M_rotors = pinocchio_robot_model_->getJointMRotors();
  const Scalar m_f_rate = static_cast<Scalar>(std::abs(pinocchio_robot_model_->getMFRate()));
  for (int i = 0; i < rotor_num; ++i)
  {
    const int dir = pinocchio_robot_model_->getRotorDirection(i);
    thrusters.emplace_back(rotor_frame_indices.at(i), joint_M_rotors.at(i).cast<Scalar>(), m_f_rate,
                           (dir == 1) ? crocoddyl::DT_CCW : crocoddyl::DT_CW,
                           static_cast<Scalar>(pinocchio_robot_model_->getThrustLowerLimits()(i)),
                           static_cast<Scalar>(pinocchio_robot_model_->getThrustUpperLimits()(i)),
                           static_cast<Scalar>(parameters_.delta_thrust_max));
  }

  state_with_thrusts_ = std::make_shared<crocoddyl::StateMultibodyWithThrustsTpl<Scalar>>(state_mb_, rotor_num);
  actuation_ =
      std::make_shared<crocoddyl::ActuationModelFloatingBaseThrusterRatesTpl<Scalar>>(state_with_thrusts_, thrusters);

  initialized_ = true;
  reset();
}

void FwddynMpcControlProblem::reset()
{
  shooting_problem_.reset();
  solver_.reset();
  xs_f_.clear();
  us_f_.clear();
  xs_.clear();
  us_.clear();
  com_residuals_.clear();
  state_residuals_.clear();
}

std::shared_ptr<crocoddyl::IntegratedActionModelEulerWithThrustsTpl<FwddynMpcControlProblem::Scalar>>
FwddynMpcControlProblem::createMPCNode(const Eigen::Matrix<Scalar, 3, 1>& com_target,
                                       const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& x_ref)
{
  const std::size_t nu = actuation_->get_nu();

  // contact
  auto contacts = std::make_shared<crocoddyl::ContactModelMultipleTpl<Scalar>>(state_mb_, nu);
  auto costs = std::make_shared<crocoddyl::CostModelSumTpl<Scalar>>(state_mb_, nu);

  // CoM tracking
  auto com_res = std::make_shared<crocoddyl::ResidualModelCoMPositionTpl<Scalar>>(state_mb_, com_target, nu);
  com_residuals_.push_back(com_res);
  auto com_act =
      std::make_shared<crocoddyl::ActivationModelWeightedQuadTpl<Scalar>>(parameters_.com_track_weight.cast<Scalar>());
  costs->addCost("comTrack", std::make_shared<crocoddyl::CostModelResidualTpl<Scalar>>(state_mb_, com_act, com_res),
                 Scalar(1.0));

  // State regularisation
  auto state_res = std::make_shared<crocoddyl::ResidualModelStateTpl<Scalar>>(state_mb_, x_ref, nu);
  state_residuals_.push_back(state_res);
  auto state_act =
      std::make_shared<crocoddyl::ActivationModelWeightedQuadTpl<Scalar>>(parameters_.x_state_weight.cast<Scalar>());
  costs->addCost("stateReg", std::make_shared<crocoddyl::CostModelResidualTpl<Scalar>>(state_mb_, state_act, state_res),
                 Scalar(1.0));

  // Centroidal momentum regularisation
  auto cm_res = std::make_shared<crocoddyl::ResidualModelCentroidalMomentumTpl<Scalar>>(
      state_mb_, Eigen::Matrix<Scalar, 6, 1>::Zero(), nu);
  auto cm_act = std::make_shared<crocoddyl::ActivationModelWeightedQuadTpl<Scalar>>(
      parameters_.centroidal_momentum_weight.cast<Scalar>());
  costs->addCost("centroidalMomentumReg",
                 std::make_shared<crocoddyl::CostModelResidualTpl<Scalar>>(state_mb_, cm_act, cm_res), Scalar(1.0));

  // Control regularisation
  auto ctrl_res = std::make_shared<crocoddyl::ResidualModelControlTpl<Scalar>>(state_mb_, nu);
  costs->addCost("ctrlReg", std::make_shared<crocoddyl::CostModelResidualTpl<Scalar>>(state_mb_, ctrl_res),
                 static_cast<Scalar>(parameters_.control_weight));

  auto dam = std::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamicsWithThrustsTpl<Scalar>>(
      state_with_thrusts_, actuation_, contacts, costs, Scalar(1e-6), false);

  // Thurst regularization
  dam->set_thrust_reg_weight(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Constant(
      pinocchio_robot_model_->getRotorNum(), static_cast<Scalar>(parameters_.thrust_reg_weight)));

  // Thrust saturation barrier
  dam->set_thrust_barrier(
      Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Constant(pinocchio_robot_model_->getRotorNum(),
                                                         static_cast<Scalar>(parameters_.thrust_barrier_weight)),
      pinocchio_robot_model_->getThrustLowerLimits().cast<Scalar>(),
      pinocchio_robot_model_->getThrustUpperLimits().cast<Scalar>());

  return std::make_shared<crocoddyl::IntegratedActionModelEulerWithThrustsTpl<Scalar>>(
      dam, static_cast<Scalar>(parameters_.dt));
}

void FwddynMpcControlProblem::buildMPCProblem(const Eigen::VectorXd& x0, const Eigen::Vector3d& com_target,
                                              const Eigen::VectorXd& x_ref)
{
  com_residuals_.clear();
  state_residuals_.clear();

  const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> x0_f = x0.cast<Scalar>();
  const Eigen::Matrix<Scalar, 3, 1> com_target_f = com_target.cast<Scalar>();
  const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> x_ref_f = x_ref.cast<Scalar>();

  // create running and terminal models
  std::vector<std::shared_ptr<crocoddyl::ActionModelAbstractTpl<Scalar>>> running_models;
  running_models.reserve(parameters_.num_nodes);
  for (int i = 0; i < parameters_.num_nodes; ++i)
    running_models.push_back(createMPCNode(com_target_f, x_ref_f));

  auto terminal = createMPCNode(com_target_f, x_ref_f);

  // create OCP and solver
  shooting_problem_ = std::make_shared<crocoddyl::ShootingProblemTpl<Scalar>>(x0_f, running_models, terminal);
  shooting_problem_->set_nthreads(parameters_.num_threads);

  if (parameters_.solver_type == SolverType::FDDP)
    solver_ = std::make_shared<crocoddyl::SolverFDDPTpl<Scalar>>(shooting_problem_);
  else if (parameters_.solver_type == SolverType::BOX_FDDP)
    solver_ = std::make_shared<crocoddyl::SolverBoxFDDPTpl<Scalar>>(shooting_problem_);
  else if (parameters_.solver_type == SolverType::INTRO)
    solver_ = std::make_shared<crocoddyl::SolverIntroTpl<Scalar>>(shooting_problem_);
  else if (parameters_.solver_type == SolverType::HPIPM_SQP)
    solver_ = std::make_shared<crocoddyl::SolverHpipmSQPTpl<Scalar>>(shooting_problem_);
  else
    throw std::runtime_error("Invalid solver type");

  xs_f_.assign(parameters_.num_nodes + 1, x0_f);
  us_f_.resize(parameters_.num_nodes);
  for (int i = 0; i < parameters_.num_nodes; ++i)
  {
    us_f_[i] = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(actuation_->get_nu());
    running_models[i]->quasiStatic(running_models[i]->createData(), us_f_[i], x0_f);
  }

  xs_ = toDoubleTrajectory(xs_f_);
  us_ = toDoubleTrajectory(us_f_);
}

void FwddynMpcControlProblem::setInitialState(const Eigen::VectorXd& x0)
{
  if (!shooting_problem_ || xs_.empty())
    return;

  const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> x0_f = x0.cast<Scalar>();
  shooting_problem_->set_x0(x0_f);
  xs_f_[0] = x0_f;
  xs_[0] = x0;
}

void FwddynMpcControlProblem::slideHorizon(const Eigen::Vector3d& com_target, const Eigen::VectorXd& x_ref)
{
  // retreive the first running node and data (which will become the new terminal node)
  std::shared_ptr<crocoddyl::ActionModelAbstractTpl<Scalar>> new_terminal =
      shooting_problem_->get_runningModels().front();
  std::shared_ptr<crocoddyl::ActionDataAbstractTpl<Scalar>> new_terminal_data =
      shooting_problem_->get_runningDatas().front();

  // append a terminal node to the end of running nodes, and remove the first node
  shooting_problem_->circularAppend(shooting_problem_->get_terminalModel(), shooting_problem_->get_terminalData());

  // update the terminal node with the first running node
  shooting_problem_->updateNode(shooting_problem_->get_T(), new_terminal, new_terminal_data);

  // update the references for the new terminal node
  std::shared_ptr<crocoddyl::ResidualModelCoMPositionTpl<Scalar>> new_com_res = com_residuals_.front();
  std::shared_ptr<crocoddyl::ResidualModelStateTpl<Scalar>> new_state_res = state_residuals_.front();
  new_com_res->set_reference(com_target.cast<Scalar>());
  new_state_res->set_reference(x_ref.cast<Scalar>());

  com_residuals_.erase(com_residuals_.begin());
  state_residuals_.erase(state_residuals_.begin());

  com_residuals_.push_back(new_com_res);
  state_residuals_.push_back(new_state_res);

  // update the initial guess by shifting the previous solution
  xs_f_.erase(xs_f_.begin());
  xs_f_.push_back(xs_f_.back());
  us_f_.erase(us_f_.begin());
  us_f_.push_back(us_f_.back());

  // rollout terminal node to close the dynamics gap introduced by the shift
  const auto& last_model = shooting_problem_->get_runningModels().back();
  const auto& last_data = shooting_problem_->get_runningDatas().back();
  last_model->calc(last_data, xs_f_[xs_f_.size() - 2], us_f_.back());
  xs_f_.back() = last_data->xnext;

  xs_ = toDoubleTrajectory(xs_f_);
  us_ = toDoubleTrajectory(us_f_);
}

bool FwddynMpcControlProblem::solveMPC(int max_iter, bool verbose, bool is_feasible)
{
  if (!solver_)
    return false;

  crocoddyl::Timer timer;
  const bool solved = solver_->solve(xs_f_, us_f_, max_iter, is_feasible);
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

  xs_f_ = solver_->get_xs();
  us_f_ = solver_->get_us();
  xs_ = toDoubleTrajectory(xs_f_);
  us_ = toDoubleTrajectory(us_f_);
  return solved;
}

}  // namespace aerial_robot_control
