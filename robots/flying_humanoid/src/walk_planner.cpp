#include <flying_humanoid/walk_planner.h>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <crocoddyl/core/activations/quadratic-barrier.hpp>
#include <crocoddyl/core/activations/weighted-quadratic-barrier.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/constraints/constraint-manager.hpp>
#include <crocoddyl/core/constraints/residual.hpp>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/residuals/select.hpp>
#include <crocoddyl/core/solvers/box-fddp.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#ifdef CROCODDYL_WITH_HPIPM
#include <crocoddyl/core/solvers/hpipm-sqp.hpp>
#endif
#include <crocoddyl/core/numdiff/action.hpp>
#include <crocoddyl/core/utils/callbacks.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn-with-thrusts.hpp>
#include <crocoddyl/multibody/actuations/floating-base-thrust-rates.hpp>
#include <crocoddyl/multibody/contacts/contact-6d.hpp>
#include <crocoddyl/multibody/contacts/multiple-contacts.hpp>
#include <crocoddyl/multibody/residuals/centroidal-momentum.hpp>
#include <crocoddyl/multibody/residuals/com-position.hpp>
#include <crocoddyl/multibody/residuals/contact-wrench-cone.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/frame-velocity.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/wrench-cone.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace flying_humanoid
{
typedef crocoddyl::IntegratedActionModelEulerWithThrustsTpl<double> IntegratedActionModelEulerWithThrusts;
typedef crocoddyl::DifferentialActionModelContactFwdDynamicsWithThrustsTpl<double>
    DifferentialActionModelContactFwdDynamicsWithThrusts;

WalkPlanner::WalkPlanner(const std::string& robot_description, const WalkPlannerParams& params) : params_(params)
{
  logger_ = std::make_shared<SolverLogger>();
  model_ = buildPinocchioModel(robot_description, &rotors_);
  data_ = pinocchio::Data(model_);
  if (rotors_.empty())
    throw std::runtime_error("no rotor (rotor1, rotor2, ...) in robot_description");

  for (const auto& rotor : rotors_)
  {
    // thrust is applied at the rotor frame, which is expressed in its parent joint
    thrusters_.emplace_back(rotor.frame_id, model_.frames[rotor.frame_id].placement, std::abs(params_.m_f_rate),
                            rotor.direction == 1 ? crocoddyl::DT_CCW : crocoddyl::DT_CW, rotor.min_thrust,
                            rotor.max_thrust);
  }

  state_ = std::make_shared<crocoddyl::StateMultibody>(std::make_shared<pinocchio::Model>(model_));
  state_with_thrusts_ = std::make_shared<crocoddyl::StateMultibodyWithThrusts>(state_, thrusters_.size());

  lfoot_id_ = model_.getFrameId(params_.lfoot);
  rfoot_id_ = model_.getFrameId(params_.rfoot);
  if (lfoot_id_ >= model_.frames.size() || rfoot_id_ >= model_.frames.size())
    throw std::runtime_error("foot frame " + params_.lfoot + " or " + params_.rfoot + " is not found");

  if (static_cast<int>(params_.initial_configuration.size()) != model_.nq)
    throw std::runtime_error("initial_configuration must have " + std::to_string(model_.nq) + " elements");
  if (params_.x_weights.size() != 2 * model_.nv)
    params_.x_weights = Eigen::VectorXd::Ones(2 * model_.nv);
  if (params_.thrust_reg_weight.size() != static_cast<int>(thrusters_.size()))
    params_.thrust_reg_weight = Eigen::VectorXd::Zero(thrusters_.size());
}

Eigen::VectorXd WalkPlanner::defaultConfiguration()
{
  // initial_configuration with the feet on the ground (z = 0) around the origin
  Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(params_.initial_configuration.data(), model_.nq);
  q.segment<4>(3) << 0.0, 0.0, 0.0, 1.0;
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  q.head<3>() -= 0.5 * (data_.oMf[lfoot_id_].translation() + data_.oMf[rfoot_id_].translation());
  return q;
}

Eigen::VectorXd WalkPlanner::measuredConfiguration(const pinocchio::SE3& world_M_baselink,
                                                   const std::map<std::string, double>& joint_positions,
                                                   const std::string& baselink)
{
  Eigen::VectorXd q = pinocchio::neutral(model_);
  for (pinocchio::JointIndex i = 2; i < model_.njoints; ++i)
  {
    auto it = joint_positions.find(model_.names[i]);
    if (it == joint_positions.end())
      throw std::runtime_error("no measured position of joint " + model_.names[i]);
    q(model_.joints[i].idx_q()) = it->second;
  }

  // the odometry gives the pose of the baselink, while q has the pose of the root link
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  const pinocchio::SE3 world_M_root = world_M_baselink * data_.oMf[model_.getFrameId(baselink)].inverse();
  const Eigen::Quaterniond quat(world_M_root.rotation());
  q.head<3>() = world_M_root.translation();
  q.segment<4>(3) << quat.x(), quat.y(), quat.z(), quat.w();
  return q;
}

bool WalkPlanner::plan(int num_steps, const Eigen::VectorXd& q_start)
{
  if (q_start.size() != model_.nq)
    throw std::runtime_error("q_start must have " + std::to_string(model_.nq) + " elements");
  q0_ = q_start;
  q0_.segment<4>(3).normalize();
  createProblem(num_steps);

  // initial guess: stand still at the initial state, holding the static equilibrium thrust
  const int nq = model_.nq;
  const int nv = model_.nv;
  const int nr = thrusters_.size();
  auto first_knot = std::static_pointer_cast<IntegratedActionModelEulerWithThrusts>(knots_.front());
  auto dmodel = std::static_pointer_cast<DifferentialActionModelContactFwdDynamicsWithThrusts>(
      first_knot->get_differential());
  const Eigen::VectorXd f_eq = dmodel->computeEquilibriumThrust(dmodel->createData(), q0_);
  std::cout << "equilibrium thrust: " << f_eq.transpose() << std::endl;

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nq + nv + nr);
  x0.head(nq) = q0_;
  x0.tail(nr) = f_eq;
  problem_->set_x0(x0);

  xs_.assign(problem_->get_T() + 1, x0);
  us_ = problem_->quasiStatic_xs(std::vector<Eigen::VectorXd>(xs_.begin(), xs_.end() - 1));

  std::shared_ptr<crocoddyl::SolverAbstractTpl<double>> solver;
  switch (params_.solver_type)
  {
    case 1:
      solver = std::make_shared<crocoddyl::SolverBoxFDDP>(problem_);
      break;
    case 3:
#ifdef CROCODDYL_WITH_HPIPM
      solver = std::make_shared<crocoddyl::SolverHpipmSQPTpl<double>>(problem_);
#else
      throw std::runtime_error("solver_type 3 (HPIPM-SQP) needs crocoddyl built with BUILD_WITH_HPIPM=ON");
#endif
      break;
    default:
      solver = std::make_shared<crocoddyl::SolverFDDP>(problem_);
      break;
  }
  logger_->clear();
  solver->setCallbacks({ std::make_shared<crocoddyl::CallbackVerbose>(), logger_ });
  const bool converged = solver->solve(xs_, us_, params_.max_iter, true);
  std::cout << "walk planning (" << num_steps << " steps): " << (converged ? "converged" : "not converged") << ", "
            << solver->get_iter() << " iterations, cost " << solver->get_cost() << std::endl;

  xs_ = solver->get_xs();
  us_ = solver->get_us();
  fs_ = solver->get_fs();
  return converged;
}

bool WalkPlanner::saveCsv(const std::string& filename) const
{
  std::ofstream ofs(filename);
  if (!ofs)
    return false;

  const int nq = model_.nq;
  const int nv = model_.nv;
  const int nr = thrusters_.size();

  ofs << "time,gait_phase";
  for (int i = 0; i < nq; ++i)
    ofs << ",q" << i;
  for (int i = 0; i < nv; ++i)
    ofs << ",v" << i;
  for (int i = 0; i < nv - 6; ++i)
    ofs << ",tau" << i;
  for (int i = 0; i < nr; ++i)
    ofs << ",thrust" << i + 1;
  ofs << "\n";

  ofs.precision(10);
  for (std::size_t k = 0; k < us_.size(); ++k)
  {
    const Eigen::VectorXd& x = xs_[k];
    ofs << k * params_.time_step << "," << gait_phases_[k];
    for (int i = 0; i < nq + nv; ++i)
      ofs << "," << x(i);
    for (int i = 0; i < nv - 6; ++i)
      ofs << "," << us_[k](nr + i);  // u = [thrust rates, joint torques]
    for (int i = 0; i < nr; ++i)
      ofs << "," << x(nq + nv + i);
    ofs << "\n";
  }
  return true;
}

void WalkPlanner::createProblem(int num_steps)
{
  knots_.clear();
  gait_phases_.clear();

  // Everything is relative to the state at the start (q0_): the robot walks along its heading (yaw of the root),
  // and each foot keeps its height, which may not be zero because of the offset of the motion capture.
  pinocchio::forwardKinematics(model_, data_, q0_);
  pinocchio::updateFramePlacements(model_, data_);
  const Eigen::Matrix3d root_rotation = Eigen::Quaterniond(q0_(6), q0_(3), q0_(4), q0_(5)).toRotationMatrix();
  heading_ = Eigen::AngleAxisd(std::atan2(root_rotation(1, 0), root_rotation(0, 0)), Eigen::Vector3d::UnitZ())
                 .toRotationMatrix();
  const Eigen::Vector3d forward = heading_.col(0);

  const pinocchio::SE3 lfoot0 = data_.oMf[lfoot_id_];
  const pinocchio::SE3 rfoot0 = data_.oMf[rfoot_id_];
  FootTasks feet = { { rfoot_id_, rfoot0 }, { lfoot_id_, lfoot0 } };
  Eigen::Vector3d com_ref = 0.5 * (lfoot0.translation() + rfoot0.translation());
  com_ref.z() = pinocchio::centerOfMass(model_, data_, q0_).z();

  // pose of a foot moved forward by distance from its pose at the start
  double foot_x = 0.0;
  auto foot_target = [&](pinocchio::FrameIndex id) {
    pinocchio::SE3 target = (id == lfoot_id_) ? lfoot0 : rfoot0;
    target.translation() += foot_x * forward;
    return target;
  };

  const int half_support_knots = params_.support_knots / 2;

  // stand still, then move the weight onto the left foot
  for (int i = 0; i < params_.support_knots * 5; ++i)
    addKnot(com_ref, feet, {});
  if (num_steps > 0)
  {
    for (int i = 0; i < half_support_knots; ++i)
      addKnot(com_ref, feet, {}, 0.5 + 0.5 * i / (half_support_knots - 1));
  }

  // each foot moves forward by step_length, alternating from the right foot
  for (int i = 1; i <= num_steps; ++i)
  {
    foot_x += 0.5 * params_.step_length;
    const bool right_step = (i % 2 == 1);
    const pinocchio::FrameIndex swing_id = right_step ? rfoot_id_ : lfoot_id_;
    const pinocchio::FrameIndex support_id = right_step ? lfoot_id_ : rfoot_id_;
    addStepModels({ support_id }, { { swing_id, foot_target(swing_id) } }, feet, com_ref);

    // move the weight onto the foot which has just landed
    for (int j = 0; j < params_.support_knots; ++j)
    {
      const double ratio = static_cast<double>(j) / (params_.support_knots - 1);
      addKnot(com_ref, feet, {}, right_step ? 1.0 - ratio : ratio);
    }
  }

  // bring the other foot next to the last swing foot
  if (num_steps > 0)
  {
    const bool right_last = (num_steps % 2 == 1);
    const pinocchio::FrameIndex swing_id = right_last ? lfoot_id_ : rfoot_id_;
    const pinocchio::FrameIndex support_id = right_last ? rfoot_id_ : lfoot_id_;
    addStepModels({ support_id }, { { swing_id, foot_target(swing_id) } }, feet, com_ref);

    // back to the even weight
    for (int i = 0; i < half_support_knots; ++i)
    {
      const double ratio = static_cast<double>(i) / (half_support_knots - 1);
      addKnot(com_ref, feet, {}, right_last ? 0.5 * ratio : 1.0 - 0.5 * ratio);
    }
  }

  // stand still; the last knot is the terminal node
  for (int i = 0; i < params_.support_knots * 5 - 1; ++i)
    addKnot(com_ref, feet, {});
  addKnot(com_ref, feet, {}, 0.5, false, true);

  std::shared_ptr<crocoddyl::ActionModelAbstract> terminal = knots_.back();
  knots_.pop_back();
  gait_phases_.pop_back();

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model_.nq + model_.nv + thrusters_.size());
  x0.head(model_.nq) = q0_;
  problem_ = std::make_shared<crocoddyl::ShootingProblem>(x0, knots_, terminal);
  problem_->set_nthreads(params_.num_threads);
}

void WalkPlanner::addStepModels(const std::vector<pinocchio::FrameIndex>& support_feet,
                                const FootTasks& swing_target, FootTasks& feet, Eigen::Vector3d& com_ref)
{
  auto current_pose = [&feet](pinocchio::FrameIndex id) {
    for (const auto& foot : feet)
      if (foot.first == id)
        return foot.second;
    return pinocchio::SE3::Identity();
  };

  // the CoM moves from the center of the current feet to the center of the feet after the step
  Eigen::Vector3d mid_start = Eigen::Vector3d::Zero();
  Eigen::Vector3d mid_end = Eigen::Vector3d::Zero();
  for (const auto id : support_feet)
  {
    mid_start += current_pose(id).translation();
    mid_end += current_pose(id).translation();
  }
  for (const auto& target : swing_target)
  {
    mid_start += current_pose(target.first).translation();
    mid_end += target.second.translation();
  }
  const double n = support_feet.size() + swing_target.size();
  mid_start /= n;
  mid_end /= n;
  const double com_height = com_ref.z() - mid_start.z();

  FootTasks support_tasks;
  for (const auto id : support_feet)
    support_tasks.emplace_back(id, current_pose(id));

  for (int k = 0; k < params_.step_knots; ++k)
  {
    // smoothstep: zero velocity at both ends
    double s = (params_.step_knots <= 1) ? 1.0 : static_cast<double>(k) / (params_.step_knots - 1);
    s = s * s * (3.0 - 2.0 * s);

    FootTasks swing_tasks;
    for (const auto& target : swing_target)
    {
      const pinocchio::SE3 start = current_pose(target.first);
      Eigen::Vector3d p = start.translation() + s * (target.second.translation() - start.translation());
      p.z() += params_.step_height * 4.0 * s * (1.0 - s);  // parabola, highest at the middle
      swing_tasks.emplace_back(target.first, pinocchio::SE3(target.second.rotation(), p));
    }

    Eigen::Vector3d com = (1.0 - s) * mid_start + s * mid_end;
    com.z() += com_height;
    addKnot(com, support_tasks, swing_tasks, 0.5, k == params_.step_knots - 1);
  }

  for (const auto& target : swing_target)
    for (auto& foot : feet)
      if (foot.first == target.first)
        foot.second = target.second;

  com_ref = mid_end;
  com_ref.z() += com_height;
}

void WalkPlanner::addKnot(const Eigen::Vector3d& com_ref, const FootTasks& support_feet, const FootTasks& swing_feet,
                          double left_foot_weight, bool touchdown, bool terminal)
{
  auto actuation =
      std::make_shared<crocoddyl::ActuationModelFloatingBaseThrusterRatesTpl<double>>(state_with_thrusts_, thrusters_);
  const std::size_t nu = actuation->get_nu();

  // with one support foot, the whole weight is on it
  double w = std::clamp(left_foot_weight, 0.0, 1.0);
  if (support_feet.size() == 1)
    w = (support_feet.front().first == lfoot_id_) ? 1.0 : 0.0;

  // shift the CoM laterally (perpendicular to the heading) toward the loaded foot
  Eigen::Vector3d com = com_ref;
  {
    const Eigen::Vector3d lateral = heading_.col(1);
    double left = 0.0, right = 0.0;
    for (const auto* tasks : { &support_feet, &swing_feet })
      for (const auto& foot : *tasks)
        (foot.first == lfoot_id_ ? left : right) = foot.second.translation().dot(lateral);
    const double mid = 0.5 * (left + right);
    const double target = mid + params_.lateral_shift_scale * (w * left + (1.0 - w) * right - mid);
    com += (target - com.dot(lateral)) * lateral;
  }

  auto contacts = std::make_shared<crocoddyl::ContactModelMultiple>(state_, nu);
  auto costs = std::make_shared<crocoddyl::CostModelSum>(state_, nu);
  auto constraints = std::make_shared<crocoddyl::ConstraintModelManager>(state_, nu);

  // CoM tracking
  costs->addCost("comTrack",
                 std::make_shared<crocoddyl::CostModelResidual>(
                     state_, std::make_shared<crocoddyl::ActivationModelWeightedQuad>(params_.com_track_weight),
                     std::make_shared<crocoddyl::ResidualModelCoMPosition>(state_, com, nu)),
                 1.0);

  // support feet: flat contact and wrench cone (friction, CoP, minimum normal force)
  for (const auto& foot : support_feet)
  {
    const std::string name = model_.frames[foot.first].name;
    contacts->addContact(name + "_contact", std::make_shared<crocoddyl::ContactModel6D>(
                                                state_, foot.first, foot.second, pinocchio::LOCAL_WORLD_ALIGNED, nu,
                                                params_.contact_gains));

    const double load = (support_feet.size() == 1) ? 1.0 : (foot.first == lfoot_id_ ? w : 1.0 - w);
    crocoddyl::WrenchCone cone(foot.second.rotation(), params_.mu, params_.foot_size);
    cone.set_min_nforce(load * params_.minimum_contact_force);
    cone.update();
    auto wrench_activation = std::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(
        crocoddyl::ActivationBounds(cone.get_lb(), cone.get_ub(), params_.contact_wrench_margin));
    costs->addCost(name + "_wrenchCone",
                   std::make_shared<crocoddyl::CostModelResidual>(
                       state_, wrench_activation,
                       std::make_shared<crocoddyl::ResidualModelContactWrenchCone>(state_, foot.first, cone, nu)),
                   params_.contact_wrench_weight);
  }

  // swing feet: track the swing trajectory, and land on the target with zero velocity.
  // The landing is a cost, and also an equality constraint with touchdown_constraint. The DDP solvers (FDDP,
  // box-FDDP) enforce equality constraints only on the terminal node, so the constraint needs HPIPM-SQP.
  for (const auto& foot : swing_feet)
  {
    const std::string name = model_.frames[foot.first].name;
    auto placement = std::make_shared<crocoddyl::ResidualModelFramePlacement>(state_, foot.first, foot.second, nu);
    auto velocity = std::make_shared<crocoddyl::ResidualModelFrameVelocity>(state_, foot.first,
                                                                            pinocchio::Motion::Zero(), pinocchio::LOCAL, nu);
    costs->addCost(name + "_footTrack",
                   std::make_shared<crocoddyl::CostModelResidual>(
                       state_, std::make_shared<crocoddyl::ActivationModelWeightedQuad>(params_.foot_track_weight),
                       placement),
                   1.0);
    if (!touchdown)
      continue;
    costs->addCost(name + "_touchdownVel", std::make_shared<crocoddyl::CostModelResidual>(state_, velocity),
                   params_.touchdown_velocity_weight);
    if (params_.touchdown_constraint)
    {
      constraints->addConstraint(name + "_placementEq",
                                 std::make_shared<crocoddyl::ConstraintModelResidual>(state_, placement));
      constraints->addConstraint(name + "_zeroVel", std::make_shared<crocoddyl::ConstraintModelResidual>(state_, velocity));
    }
  }

  // keep the centroidal momentum small
  costs->addCost(
      "centroidalMomentumTrack",
      std::make_shared<crocoddyl::CostModelResidual>(
          state_, std::make_shared<crocoddyl::ActivationModelWeightedQuad>(params_.centroidal_momentum_weight),
          std::make_shared<crocoddyl::ResidualModelCentroidalMomentum>(state_, Eigen::VectorXd::Zero(6), nu)),
      1.0);

  // state regularization toward the posture at the start, with the root upright along the heading
  Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(model_.nq + model_.nv);
  x_ref.head(model_.nq) = q0_;
  const Eigen::Quaterniond heading_quat(heading_);
  x_ref.segment<4>(3) << heading_quat.x(), heading_quat.y(), heading_quat.z(), heading_quat.w();
  costs->addCost("stateReg",
                 std::make_shared<crocoddyl::CostModelResidual>(
                     state_, std::make_shared<crocoddyl::ActivationModelWeightedQuad>(params_.x_weights),
                     std::make_shared<crocoddyl::ResidualModelState>(state_, x_ref, nu)),
                 params_.state_weight);

  // terminal node: the joint angles return to the initial ones (equality constraint)
  if (terminal && params_.terminal_posture_constraint)
  {
    // rows of the state residual (tangent space: root 6, joints nv - 6, velocities nv) for the joint angles
    std::vector<std::size_t> joint_rows;
    for (int i = 6; i < model_.nv; ++i)
      joint_rows.push_back(i);
    auto joint_residual = std::make_shared<crocoddyl::ResidualModelSelect>(
        std::make_shared<crocoddyl::ResidualModelState>(state_, x_ref, nu), joint_rows);
    constraints->addConstraint("terminalPosture",
                               std::make_shared<crocoddyl::ConstraintModelResidual>(state_, joint_residual));
  }

  // joint position/velocity limits (not on the floating base pose)
  {
    const int ndx = state_->get_ndx();
    Eigen::VectorXd lb = state_->get_lb().tail(ndx);
    Eigen::VectorXd ub = state_->get_ub().tail(ndx);
    Eigen::VectorXd weight = Eigen::VectorXd::Constant(ndx, params_.state_bound_weight);
    lb.head(6).setConstant(-std::numeric_limits<double>::infinity());
    ub.head(6).setConstant(std::numeric_limits<double>::infinity());
    weight.head(6).setZero();
    costs->addCost("stateBound",
                   std::make_shared<crocoddyl::CostModelResidual>(
                       state_,
                       std::make_shared<crocoddyl::ActivationModelWeightedQuadraticBarrier>(
                           crocoddyl::ActivationBounds(lb, ub, params_.state_bound_margin), weight),
                       std::make_shared<crocoddyl::ResidualModelState>(state_, state_->zero(), nu)),
                   1.0);
  }

  // control regularization (thrust rates and joint torques)
  costs->addCost("ctrlReg",
                 std::make_shared<crocoddyl::CostModelResidual>(
                     state_, std::make_shared<crocoddyl::ResidualModelControl>(state_, nu)),
                 params_.control_weight);

  auto dmodel = std::make_shared<DifferentialActionModelContactFwdDynamicsWithThrusts>(
      state_with_thrusts_, actuation, contacts, costs, constraints, params_.contact_jacobian_damping, true);
  Eigen::VectorXd thrust_lb(thrusters_.size()), thrust_ub(thrusters_.size());
  for (std::size_t i = 0; i < thrusters_.size(); ++i)
  {
    thrust_lb(i) = thrusters_[i].min_thrust_;
    thrust_ub(i) = thrusters_[i].max_thrust_;
  }
  dmodel->set_thrust_reg_weight(params_.thrust_reg_weight);
  dmodel->set_thrust_barrier(Eigen::VectorXd::Constant(thrusters_.size(), params_.thrust_barrier_weight), thrust_lb,
                             thrust_ub);
  knots_.push_back(std::make_shared<IntegratedActionModelEulerWithThrusts>(dmodel, params_.time_step));

  if (support_feet.size() == 2)
    gait_phases_.push_back(DOUBLE_SUPPORT);
  else
    gait_phases_.push_back(support_feet.front().first == lfoot_id_ ? SWING_RIGHT : SWING_LEFT);
}
bool WalkPlanner::saveSolverLog(const std::string& filename) const
{
  return logger_->save(filename);
}

bool WalkPlanner::saveNodeDiagnosis(const std::string& filename)
{
  std::ofstream ofs(filename);
  if (!ofs || !problem_)
    return false;

  // evaluate every node at the solution
  problem_->calc(xs_, us_);

  // long format: one row per node and item
  //   cost       : weighted cost of each term
  //   constraint : infinity norm of the residual of each active equality constraint
  //   defect     : infinity norm of the dynamics gap (f(x_t, u_t) - x_{t+1}) of the q, v and thrust parts
  //   control    : infinity norm of the thrust rates and of the joint torques
  ofs << "node,gait_phase,kind,name,value\n";
  const int nv = model_.nv;
  const int nr = thrusters_.size();
  const auto& datas = problem_->get_runningDatas();
  for (std::size_t t = 0; t < datas.size(); ++t)
  {
    auto data = std::static_pointer_cast<crocoddyl::IntegratedActionDataEulerWithThrustsTpl<double>>(datas[t]);
    auto ddata = std::static_pointer_cast<crocoddyl::DifferentialActionDataContactFwdDynamicsWithThrustsTpl<double>>(
        data->differential);
    const std::string prefix = std::to_string(t) + "," + std::to_string(gait_phases_[t]) + ",";

    auto model = std::static_pointer_cast<IntegratedActionModelEulerWithThrusts>(knots_[t]);
    auto dmodel = std::static_pointer_cast<DifferentialActionModelContactFwdDynamicsWithThrusts>(model->get_differential());
    for (const auto& cost : dmodel->get_costs()->get_costs())
    {
      if (!cost.second->active)
        continue;
      ofs << prefix << "cost," << cost.first << "," << cost.second->weight * ddata->costs->costs.at(cost.first)->cost
          << "\n";
    }
    ofs << prefix << "cost,total," << data->cost << "\n";

    if (dmodel->get_constraints())
    {
      for (const auto& constraint : dmodel->get_constraints()->get_constraints())
      {
        if (!constraint.second->active)
          continue;
        ofs << prefix << "constraint," << constraint.first << ","
            << ddata->constraints->constraints.at(constraint.first)->h.lpNorm<Eigen::Infinity>() << "\n";
      }
    }

    if (t < fs_.size() - 1)
    {
      const Eigen::VectorXd& f = fs_[t + 1];  // fs_[t + 1] is the gap at the end of node t
      ofs << prefix << "defect,q," << f.head(nv).lpNorm<Eigen::Infinity>() << "\n";
      ofs << prefix << "defect,v," << f.segment(nv, nv).lpNorm<Eigen::Infinity>() << "\n";
      ofs << prefix << "defect,thrust," << f.tail(nr).lpNorm<Eigen::Infinity>() << "\n";
    }
    ofs << prefix << "control,thrust_rate," << us_[t].head(nr).lpNorm<Eigen::Infinity>() << "\n";
    ofs << prefix << "control,joint_torque," << us_[t].tail(nv - 6).lpNorm<Eigen::Infinity>() << "\n";
  }
  return true;
}

void WalkPlanner::checkDerivatives()
{
  if (!problem_ || xs_.empty())
    return;

  // representative nodes: the first knot, the middle of the first swing, its touchdown and the next double support
  std::vector<std::size_t> nodes = { 0 };
  for (std::size_t t = 1; t < gait_phases_.size(); ++t)
  {
    if (gait_phases_[t] != DOUBLE_SUPPORT && gait_phases_[t - 1] == DOUBLE_SUPPORT)
    {
      nodes.push_back(t + params_.step_knots / 2);
      nodes.push_back(t + params_.step_knots - 1);
      nodes.push_back(t + params_.step_knots + 1);
      break;
    }
  }

  const int nv = model_.nv;
  const int nr = thrusters_.size();
  // relative error of each block of the derivatives (columns: dq, dv, dthrust for x, thrust rate and torque for u)
  auto relative_error = [](const Eigen::MatrixXd& analytic, const Eigen::MatrixXd& numeric) {
    const double scale = std::max(1.0, numeric.lpNorm<Eigen::Infinity>());
    return (analytic - numeric).lpNorm<Eigen::Infinity>() / scale;
  };

  std::cout << "derivative check (relative error of analytic vs finite difference, columns dq / dv / dthrust | "
               "dthrust_rate / dtau)"
            << std::endl;
  for (const std::size_t t : nodes)
  {
    if (t >= knots_.size())
      continue;
    auto model = knots_[t];
    auto numdiff = std::make_shared<crocoddyl::ActionModelNumDiff>(model);
    auto data = model->createData();
    auto ndata = numdiff->createData();
    model->calc(data, xs_[t], us_[t]);
    model->calcDiff(data, xs_[t], us_[t]);
    numdiff->calc(ndata, xs_[t], us_[t]);
    numdiff->calcDiff(ndata, xs_[t], us_[t]);

    auto blocks = [&](const char* name, const Eigen::MatrixXd& a, const Eigen::MatrixXd& n) {
      std::cout << "  " << name << ": " << relative_error(a.leftCols(nv), n.leftCols(nv)) << " / "
                << relative_error(a.middleCols(nv, nv), n.middleCols(nv, nv)) << " / "
                << relative_error(a.rightCols(nr), n.rightCols(nr));
    };
    auto ublocks = [&](const Eigen::MatrixXd& a, const Eigen::MatrixXd& n) {
      std::cout << " | " << relative_error(a.leftCols(nr), n.leftCols(nr)) << " / "
                << relative_error(a.rightCols(nv - 6), n.rightCols(nv - 6)) << std::endl;
    };
    std::cout << " node " << t << " (phase " << gait_phases_[t] << ")" << std::endl;
    blocks("Fx|Fu", data->Fx, ndata->Fx);
    ublocks(data->Fu, ndata->Fu);
    blocks("Lx|Lu", data->Lx.transpose(), ndata->Lx.transpose());
    ublocks(data->Lu.transpose(), ndata->Lu.transpose());
  }
}
}  // namespace flying_humanoid
