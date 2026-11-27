// pinocchio before boost (and ROS)
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <gimbalrotor/control/gimbalrotor_whole_body_controller.h>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <tf_conversions/tf_eigen.h>

namespace aerial_robot_control
{
namespace
{
double toDouble(XmlRpc::XmlRpcValue& v)
{
  return v.getType() == XmlRpc::XmlRpcValue::TypeInt ? static_cast<double>(static_cast<int>(v)) :
                                                       static_cast<double>(v);
}

std::vector<double> toVector(XmlRpc::XmlRpcValue& v)
{
  std::vector<double> out;
  for (int i = 0; i < v.size(); i++)
    out.push_back(toDouble(v[i]));
  return out;
}

Eigen::Matrix3d toEigen(const tf::Matrix3x3& m)
{
  Eigen::Matrix3d out;
  tf::matrixTFToEigen(m, out);
  return out;
}

constexpr double INF = 1e20;
}  // namespace

void GimbalrotorWholeBodyController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                                double ctrl_loop_rate)
{
  GimbalrotorController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  if (gimbal_calc_in_fc_)
  {
    ROS_WARN("[whole body] the allocation is on this side: gimbal_calc_in_fc set to false");
    gimbal_calc_in_fc_ = false;
  }
  if (gimbal_dof_ != 1)
    throw std::runtime_error("[whole body] only gimbal_dof 1 is supported");

  // the model: the URDF with a free flyer at its root
  std::string urdf;
  if (!nh_.getParam("robot_description", urdf))
    throw std::runtime_error("[whole body] no robot_description");
  pinocchio::urdf::buildModelFromXML(urdf, pinocchio::JointModelFreeFlyer(), model_);
  for (int i = 0; i < motor_num_; i++)
  {
    const std::string thrust = "thrust" + std::to_string(i + 1), gimbal = "gimbal" + std::to_string(i + 1);
    if (!model_.existFrame(thrust) || !model_.existJointName(gimbal))
      throw std::runtime_error("[whole body] no " + thrust + " or " + gimbal + " in the model");
    thrust_frames_.push_back(model_.getFrameId(thrust));
    gimbal_joints_.push_back(model_.getJointId(gimbal));
    const auto axis = robot_model_->getUrdfModel().getJoint(gimbal)->axis;
    gimbal_axes_.push_back(Eigen::Vector3d(axis.x, axis.y, axis.z).normalized());
  }
  baselink_frame_ = model_.getFrameId(robot_model_->getBaselinkName());

  // the parameters
  ros::NodeHandle wb(nh_, "controller/whole_body");
  wb.param("min_lift", min_lift_, 0.0);
  wb.param("max_tilt", max_tilt_, M_PI / 2);
  wb.param("thrust_regularization", thrust_reg_, 1e-3);
  wb.param("contact_regularization", contact_reg_, 1e-4);
  // the regularisations are small next to the wrench cost: the split between the rotors and the
  // contacts is decided by them, along a direction of little curvature, so it needs a tight
  // tolerance (with the default 1e-3 of OSQP it stops far from the optimum, the contacts carrying
  // little)
  wb.param("solver_tolerance", solver_tolerance_, 1e-6);
  wb.param("solver_max_iteration", solver_max_iteration_, 10000);
  std::vector<double> weight;
  wb.param("wrench_weight", weight, std::vector<double>{ 1.0, 1.0, 1.0, 10.0, 10.0, 10.0 });
  if (weight.size() != 6)
    throw std::runtime_error("[whole body] wrench_weight needs 6 values");
  wrench_weight_ = Eigen::Map<Eigen::VectorXd>(weight.data(), 6);
  std::vector<std::string> frozen;
  wb.param("freeze_integral_in_contact", frozen, std::vector<std::string>{ "x", "y", "z", "roll", "pitch", "yaw" });
  const std::vector<std::string> axis_names{ "x", "y", "z", "roll", "pitch", "yaw" };  // the order of X..YAW
  freeze_integral_.assign(6, false);
  for (const auto& name : frozen)
  {
    const auto it = std::find(axis_names.begin(), axis_names.end(), name);
    if (it == axis_names.end())
      throw std::runtime_error("[whole body] freeze_integral_in_contact: unknown axis " + name);
    freeze_integral_.at(it - axis_names.begin()) = true;
  }
  std::vector<std::string> gathered;
  wb.param("contact_integral", gathered, std::vector<std::string>{});
  contact_integral_.assign(6, false);
  for (const auto& name : gathered)
  {
    const auto it = std::find(axis_names.begin(), axis_names.end(), name);
    if (it == axis_names.end())
      throw std::runtime_error("[whole body] contact_integral: unknown axis " + name);
    contact_integral_.at(it - axis_names.begin()) = true;
  }
  wb.param("contact_integral_scale", contact_integral_scale_, 1.0);
  if (contact_integral_scale_ < 1.0)
    throw std::runtime_error("[whole body] contact_integral_scale must be at least 1");
  free_integral_.assign(6, 0.0);
  contact_excess_.assign(6, 0.0);
  double ratio;
  wb.param("torque_limit_ratio", ratio, 0.75);
  wb.param("torque_limited_joints", torque_limited_joints_, std::vector<std::string>{});
  for (const auto& name : torque_limited_joints_)
  {
    const auto joint = robot_model_->getUrdfModel().getJoint(name);
    if (!joint || !model_.existJointName(name) || !joint->limits)
      throw std::runtime_error("[whole body] torque limited joint " + name + " is not a limited joint of the model");
    torque_limits_.emplace_back(model_.joints[model_.getJointId(name)].idx_v(), ratio * joint->limits->effort);
  }

  // the contacts: frames added to the model
  XmlRpc::XmlRpcValue contacts;
  if (wb.getParam("contacts", contacts) && contacts.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    for (auto& item : contacts)
    {
      WholeBodyContact c;
      c.name = item.first;
      auto& v = item.second;
      c.link = static_cast<std::string>(v["link"]);
      if (!model_.existFrame(c.link))
        throw std::runtime_error("[whole body] contact " + c.name + ": no link " + c.link);
      const auto xyz = toVector(v["xyz"]), quat = toVector(v["quaternion"]);  // in the link, [x, y, z, w]
      const pinocchio::SE3 in_link(Eigen::Quaterniond(quat[3], quat[0], quat[1], quat[2]).toRotationMatrix(),
                                   Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));
      const auto& link = model_.frames[model_.getFrameId(c.link)];
      c.frame = model_.addFrame(pinocchio::Frame(c.name, link.parentJoint, model_.getFrameId(c.link),
                                                 link.placement * in_link, pinocchio::OP_FRAME));
      if (v.hasMember("friction"))  // one coefficient, or one for the x and the y of the contact frame
      {
        if (v["friction"].getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
          const auto mu = toVector(v["friction"]);
          if (mu.size() != 2)
            throw std::runtime_error("[whole body] contact " + c.name + ": friction is one value or two (x, y)");
          c.friction = Eigen::Vector2d(mu[0], mu[1]);
        }
        else
          c.friction.setConstant(toDouble(v["friction"]));
        if ((c.friction.array() < 0.0).any())
          throw std::runtime_error("[whole body] contact " + c.name + ": negative friction");
      }
      if (v.hasMember("min_normal"))
        c.min_normal = toDouble(v["min_normal"]);
      if (v.hasMember("max_normal"))
        c.max_normal = toDouble(v["max_normal"]);
      if (v.hasMember("ramp_time"))
        c.ramp_time = toDouble(v["ramp_time"]);
      if (v.hasMember("ramp"))
      {
        const std::string ramp = static_cast<std::string>(v["ramp"]);
        if (ramp != "relative" && ramp != "absolute")
          throw std::runtime_error("[whole body] contact " + c.name + ": ramp is relative or absolute, not " + ramp);
        c.relative_ramp = ramp == "relative";
      }
      c.prescribed = v.hasMember("mode") && static_cast<std::string>(v["mode"]) == "prescribed";
      c.force = Eigen::Vector3d::Zero();
      if (v.hasMember("force"))
      {
        const auto f = toVector(v["force"]);
        c.force = Eigen::Vector3d(f[0], f[1], f[2]);
      }
      contacts_.push_back(c);
    }
  }
  data_ = pinocchio::Data(model_);
  for (size_t i = 0; i < contacts_.size(); i++)
  {
    auto& c = contacts_[i];
    c.server = nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
        "whole_body/contacts/" + c.name + "/activate",
        boost::bind(&GimbalrotorWholeBodyController::activateContact, this, _1, _2, i));
    c.force_pub = nh_.advertise<geometry_msgs::WrenchStamped>("whole_body/contacts/" + c.name + "/force", 1);
  }
  wrench_error_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("whole_body/wrench_error", 1);
  joint_torque_pub_ = nh_.advertise<sensor_msgs::JointState>("whole_body/joint_torque", 1);
  ROS_INFO("[whole body] %d rotors, %zu contacts, %zu torque limited joints, min lift %.2f N, max tilt %.2f rad",
           motor_num_, contacts_.size(), torque_limits_.size(), min_lift_, max_tilt_);
}

bool GimbalrotorWholeBodyController::activateContact(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res,
                                                     size_t index)
{
  std::lock_guard<std::mutex> lock(contacts_mutex_);
  auto& c = contacts_.at(index);
  const ros::Time now = ros::Time::now();
  if (c.active != static_cast<bool>(req.data))
  {
    c.share_at_switch = loadShare(c, now);
    c.active = req.data;
    c.switched = now;
  }
  res.success = true;
  res.message = c.name + (c.active ? " on" : " off");
  ROS_INFO("[whole body] contact %s", res.message.c_str());
  return true;
}

void GimbalrotorWholeBodyController::reset()
{
  GimbalrotorController::reset();
  was_loaded_ = unloading_ = false;  // the PIDs are reset too
  // every contact off at once: called while arming (every loop), at the takeoff and after the
  // motors are off, so no contact is carried over to the next flight
  std::lock_guard<std::mutex> lock(contacts_mutex_);
  const ros::Time now = ros::Time::now();
  for (auto& c : contacts_)
  {
    if (c.active || c.share_at_switch > 0.0)
      ROS_INFO("[whole body] reset: contact %s off", c.name.c_str());
    c.active = false;
    c.share_at_switch = 0.0;
    c.switched = now;
    c.last_force.setZero();
  }
}

double GimbalrotorWholeBodyController::loadShare(const WholeBodyContact& c, const ros::Time& now) const
{
  // from the share at the switch, linearly to 1 (on) or 0 (off) over ramp_time
  const double s = c.ramp_time > 0 ? (now - c.switched).toSec() / c.ramp_time : 1.0;
  const double target = c.active ? 1.0 : 0.0;
  return c.share_at_switch + std::min(std::max(s, 0.0), 1.0) * (target - c.share_at_switch);
}

bool GimbalrotorWholeBodyController::updateModel(Eigen::VectorXd& q)
{
  // the joints from the robot model (the joint states); the root turned so that the baselink has the
  // estimated attitude, at the origin (the statics needs no position)
  q = pinocchio::neutral(model_);
  const auto& positions = robot_model_->getJointPositions();
  const auto& index = robot_model_->getJointIndexMap();
  for (pinocchio::JointIndex j = 2; j < static_cast<pinocchio::JointIndex>(model_.njoints); j++)
  {
    const auto it = index.find(model_.names[j]);
    if (it == index.end())
      continue;
    const double angle = positions(it->second);
    const auto& joint = model_.joints[j];
    if (joint.nq() == 1)
      q[joint.idx_q()] = angle;
    else if (joint.nq() == 2)  // continuous: cos, sin
      q.segment<2>(joint.idx_q()) << std::cos(angle), std::sin(angle);
  }
  pinocchio::framesForwardKinematics(model_, data_, q);
  const Eigen::Matrix3d root_to_baselink = data_.oMf[baselink_frame_].rotation();
  const Eigen::Matrix3d world_to_baselink = toEigen(estimator_->getOrientation(Frame::BASELINK, estimate_mode_));
  const Eigen::Quaterniond root(world_to_baselink * root_to_baselink.transpose());
  q.segment<4>(3) << root.x(), root.y(), root.z(), root.w();

  pinocchio::computeJointJacobians(model_, data_, q);  // and the forward kinematics
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::centerOfMass(model_, data_, q, false);
  pinocchio::computeGeneralizedGravity(model_, data_, q);
  return true;
}

void GimbalrotorWholeBodyController::controlCore()
{
  // the PIDs hold their integral on the frozen axes while a contact carries load
  bool loaded = false, active = false;
  double share = 0.0;
  {
    std::lock_guard<std::mutex> lock(contacts_mutex_);
    const ros::Time now = ros::Time::now();
    for (const auto& contact : contacts_)
    {
      share = std::max(share, loadShare(contact, now));
      active = active || contact.active;
    }
    loaded = share > 1e-6;
  }
  for (size_t axis = 0; axis < freeze_integral_.size(); axis++)
    integrate_.at(axis) = !(loaded && freeze_integral_[axis]);
  if (loaded && !was_loaded_)  // the integral the robot flies free with
  {
    for (size_t axis = 0; axis < contact_integral_.size(); axis++)
      free_integral_.at(axis) = pid_controllers_.at(axis).getErrI();
    was_loaded_ = true;
    unloading_ = false;
  }

  // the PIDs and the usual allocation (kept if the QP does not run or fails)
  GimbalrotorController::controlCore();
  updateContactIntegral(loaded, active, share);
  if (navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE)
    return;

  // the wrench of the PIDs (world axes, about the CoG): m a and I alpha + w x I w
  const double mass = robot_model_->getMass();
  const Eigen::Vector3d acc(pid_controllers_.at(X).result(), pid_controllers_.at(Y).result(),
                            pid_controllers_.at(Z).result());
  const Eigen::Vector3d ang_acc(pid_controllers_.at(ROLL).result(), pid_controllers_.at(PITCH).result(),
                                pid_controllers_.at(YAW).result());
  const Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::Vector3d omega;
  tf::vectorTFToEigen(omega_, omega);
  const Eigen::Matrix3d world_to_cog = toEigen(estimator_->getOrientation(Frame::COG, estimate_mode_));
  Eigen::Matrix<double, 6, 1> wrench;
  wrench.head<3>() = mass * acc;
  wrench.tail<3>() = world_to_cog * (inertia * ang_acc + omega.cross(inertia * omega));

  Eigen::VectorXd q;
  if (!updateModel(q) || !allocate(q, wrench, ros::Time::now()))
    ROS_ERROR_THROTTLE(1.0, "[whole body] the QP failed: the usual allocation");
}

void GimbalrotorWholeBodyController::updateContactIntegral(bool loaded, bool active, double share)
{
  // after the PIDs: their integrals for the next cycle
  if (!was_loaded_)
    return;
  if (!loaded)  // free again: the integral as it was before the load
  {
    for (size_t axis = 0; axis < contact_integral_.size(); axis++)
      if (contact_integral_[axis])
        pid_controllers_.at(axis).setErrI(free_integral_.at(axis));
    was_loaded_ = unloading_ = false;
    return;
  }
  for (size_t axis = 0; axis < contact_integral_.size(); axis++)
  {
    if (!contact_integral_[axis])
      continue;
    auto& pid = pid_controllers_.at(axis);
    if (active)  // the step of this cycle (0 if held), sped up
      pid.setErrI(pid.getErrI() + (contact_integral_scale_ - 1.0) * (pid.getErrI() - pid.getPrevErrI()));
    else  // every contact going off: what was gathered goes out with the load
    {
      if (!unloading_)
        contact_excess_.at(axis) = pid.getErrI() - free_integral_.at(axis);
      const double ratio = unloading_ ? share / unload_share_ : 1.0;
      pid.setErrI(free_integral_.at(axis) + ratio * contact_excess_.at(axis));
    }
  }
  if (active)
    unloading_ = false;
  else if (!unloading_)
  {
    unloading_ = true;
    unload_share_ = share;
  }
}

bool GimbalrotorWholeBodyController::allocate(const Eigen::VectorXd& q, const Eigen::Matrix<double, 6, 1>& wrench,
                                              const ros::Time& now)
{
  std::lock_guard<std::mutex> lock(contacts_mutex_);
  const Eigen::Vector3d com = data_.com[0], up_world = Eigen::Vector3d::UnitZ();
  const int n_rotor = motor_num_;
  std::vector<size_t> contacts;  // those with a load share (on, or ramping off)
  std::vector<double> shares;
  for (size_t k = 0; k < contacts_.size(); k++)
  {
    const double share = loadShare(contacts_[k], now);
    if (share > 1e-6)
    {
      contacts.push_back(k);
      shares.push_back(share);
    }
  }
  const int nx = 2 * n_rotor + 3 * static_cast<int>(contacts.size());

  // G: the wrench (world, about the CoG) of each variable; T: its torque on the limited joints
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(6, nx), T = Eigen::MatrixXd::Zero(torque_limits_.size(), nx);
  pinocchio::Data::Matrix6x J(6, model_.nv);
  auto add_torques = [&](int col, const Eigen::Vector3d& force, const Eigen::Vector3d& moment) {
    // the joint torques of the statics: tau = g(q) - J^T [f; m]
    for (size_t r = 0; r < torque_limits_.size(); r++)
    {
      const int v = torque_limits_[r].first;
      T(r, col) = -(J.col(v).head<3>().dot(force) + J.col(v).tail<3>().dot(moment));
    }
  };
  const auto& rotor_direction = gimbalrotor_robot_model_->getRotorDirection();
  const double m_f_rate = gimbalrotor_robot_model_->getMFRate();
  std::vector<Eigen::Vector3d> sides(n_rotor), ups(n_rotor), axes(n_rotor);
  for (int i = 0; i < n_rotor; i++)
  {
    const auto& frame = data_.oMf[thrust_frames_[i]];
    const Eigen::Vector3d p = frame.translation() - com;
    axes[i] = data_.oMi[gimbal_joints_[i]].rotation() * gimbal_axes_[i];
    ups[i] = (up_world - axes[i].dot(up_world) * axes[i]).normalized();
    sides[i] = ups[i].cross(axes[i]);
    const double drag = rotor_direction.at(i + 1) * m_f_rate;  // torque along the thrust per thrust
    J.setZero();
    pinocchio::getFrameJacobian(model_, data_, thrust_frames_[i], pinocchio::LOCAL_WORLD_ALIGNED, J);
    for (int k = 0; k < 2; k++)
    {
      const Eigen::Vector3d e = k == 0 ? sides[i] : ups[i];
      G.col(2 * i + k) << e, p.cross(e) + drag * e;
      add_torques(2 * i + k, e, drag * e);
    }
  }
  for (size_t c = 0; c < contacts.size(); c++)
  {
    const auto& contact = contacts_[contacts[c]];
    const Eigen::Vector3d p = data_.oMf[contact.frame].translation() - com;
    J.setZero();
    pinocchio::getFrameJacobian(model_, data_, contact.frame, pinocchio::LOCAL_WORLD_ALIGNED, J);
    for (int k = 0; k < 3; k++)
    {
      const int col = 2 * n_rotor + 3 * c + k;
      const Eigen::Vector3d e = Eigen::Vector3d::Unit(k);
      G.col(col) << e, p.cross(e);
      add_torques(col, e, Eigen::Vector3d::Zero());
    }
  }

  // cost: |G x - wrench|_W^2 + reg |x|^2
  const Eigen::MatrixXd W = wrench_weight_.asDiagonal();
  Eigen::VectorXd reg(nx);
  reg.head(2 * n_rotor).setConstant(thrust_reg_);
  reg.tail(nx - 2 * n_rotor).setConstant(contact_reg_);
  const Eigen::MatrixXd P = 2.0 * (G.transpose() * W * G + Eigen::MatrixXd(reg.asDiagonal()));
  Eigen::VectorXd gradient = -2.0 * G.transpose() * W * wrench;

  // constraints: l <= A x <= u
  std::vector<Eigen::VectorXd> rows;
  std::vector<double> lower, upper;
  auto add = [&](const Eigen::VectorXd& row, double l, double u) {
    rows.push_back(row);
    lower.push_back(l);
    upper.push_back(u);
  };
  const double tan_tilt = std::tan(std::min(max_tilt_, M_PI / 2 - 1e-3));
  for (int i = 0; i < n_rotor; i++)
  {
    Eigen::VectorXd row = Eigen::VectorXd::Zero(nx);
    row(2 * i + 1) = 1.0;  // t: the lift
    add(row, std::max(min_lift_, robot_model_->getThrustLowerLimit()), robot_model_->getThrustUpperLimit());
    if (max_tilt_ < M_PI / 2)
    {
      for (double sign : { 1.0, -1.0 })
      {
        row.setZero();
        row(2 * i) = sign;
        row(2 * i + 1) = -tan_tilt;
        add(row, -INF, 0.0);
      }
    }
  }
  // the rows of the normal forces of the support contacts that ramp relatively (their bounds set below)
  std::vector<std::pair<size_t, int>> ramping;  // (contact, row)
  for (size_t c = 0; c < contacts.size(); c++)
  {
    const auto& contact = contacts_[contacts[c]];
    const Eigen::Matrix3d R = data_.oMf[contact.frame].rotation();
    const int col = 2 * n_rotor + 3 * c;
    Eigen::VectorXd row = Eigen::VectorXd::Zero(nx);
    if (contact.prescribed)
    {
      const Eigen::Vector3d force = shares[c] * (R * contact.force);
      for (int k = 0; k < 3; k++)
      {
        row.setZero();
        row(col + k) = 1.0;
        add(row, force(k), force(k));
      }
      continue;
    }
    const Eigen::Vector3d normal = R.col(2);
    row.segment<3>(col) = normal;
    const bool relative = contact.relative_ramp && shares[c] < 1.0 - 1e-9;
    if (relative)
      ramping.emplace_back(c, static_cast<int>(rows.size()));
    const double share = relative ? 1.0 : shares[c];  // relative: fully loaded first
    add(row, share * contact.min_normal, share * contact.max_normal);
    for (int axis = 0; axis < 2; axis++)
    {
      for (double sign : { 1.0, -1.0 })
      {
        row.setZero();
        row.segment<3>(col) = contact.friction(axis) * normal + sign * R.col(axis);
        add(row, 0.0, INF);
      }
    }
  }
  for (size_t r = 0; r < torque_limits_.size(); r++)
  {
    const double g = data_.g(torque_limits_[r].first), limit = torque_limits_[r].second;
    add(T.row(r).transpose(), -limit - g, limit - g);
  }

  const int m = rows.size();
  Eigen::MatrixXd A(m, nx);
  for (int r = 0; r < m; r++)
    A.row(r) = rows[r].transpose();
  Eigen::VectorXd l = Eigen::Map<Eigen::VectorXd>(lower.data(), m), u = Eigen::Map<Eigen::VectorXd>(upper.data(), m);
  auto solve = [&](const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound, Eigen::VectorXd& x) {
    namespace pq = proxsuite::proxqp;
    pq::dense::QP<double> solver(nx, 0, m);
    solver.settings.eps_abs = solver_tolerance_;
    solver.settings.eps_rel = solver_tolerance_;
    solver.settings.max_iter = solver_max_iteration_;
    // the infeasibility checks at their defaults (1e-4) take this problem, whose regularisation is
    // small (1e-3, 1e-4), for infeasible within a few iterations while it is feasible (e.g. a contact
    // bounded by its share of the load): kept for the truly hopeless only
    solver.settings.eps_primal_inf = 1e-12;
    solver.settings.eps_dual_inf = 1e-12;
    solver.settings.verbose = false;
    solver.init(P, gradient, proxsuite::nullopt, proxsuite::nullopt, A, lower_bound, upper_bound);
    solver.solve();
    if (solver.results.info.status != pq::QPSolverOutput::PROXQP_SOLVED)
      return false;
    x = solver.results.x;
    if (!x.allFinite())
      return false;
    // a sanity check of the solution: the constraints met within what matters physically ([N], [Nm];
    // a solved one may miss them by about the solver tolerance)
    constexpr double violation = 1e-4;
    const Eigen::VectorXd constraint = A * x;
    return (lower_bound - constraint).maxCoeff() <= violation && (constraint - upper_bound).maxCoeff() <= violation;
  };

  // the relatively ramping contacts fully loaded first, then each bounded by its share of what it
  // carried (absolute bounds if that fails)
  Eigen::VectorXd x;
  const bool full = solve(l, u, x);
  if (ramping.empty())
  {
    if (!full)
      return false;
  }
  else
  {
    for (const auto& [c, row] : ramping)
    {
      const auto& contact = contacts_[contacts[c]];
      if (full)
      {
        const double carried = std::max(A.row(row).dot(x), 0.0);  // its normal force fully loaded
        u(row) = shares[c] * carried;
        l(row) = std::min(shares[c] * contact.min_normal, u(row));
      }
      else
      {
        l(row) = shares[c] * contact.min_normal;
        u(row) = shares[c] * contact.max_normal;
      }
    }
    if (!full)
      ROS_WARN_THROTTLE(1.0, "[whole body] the QP fully loaded failed: the absolute ramp for this cycle");
    if (!solve(l, u, x))
      return false;
  }

  // the thrust of each rotor: its magnitude, and the gimbal turned so that its thrust axis is along it
  for (int i = 0; i < n_rotor; i++)
  {
    const Eigen::Vector3d f = x(2 * i) * sides[i] + x(2 * i + 1) * ups[i];
    const Eigen::Vector3d axis = data_.oMf[thrust_frames_[i]].rotation().col(2);
    const double current = q[model_.joints[gimbal_joints_[i]].idx_q()];
    target_full_thrust_.at(i) = f.norm();
    target_gimbal_angles_.at(i) = current + std::atan2(axes[i].dot(axis.cross(f)), axis.dot(f));
    target_vectoring_f_trans_.segment<2>(2 * i) = x.segment<2>(2 * i);  // debug: (side, up)
    target_vectoring_f_rot_.segment<2>(2 * i).setZero();
  }

  // the contact forces of the QP (in the contact frame) and the wrench error
  const std::string world = "world";
  for (auto& contact : contacts_)
    contact.last_force.setZero();
  for (size_t c = 0; c < contacts.size(); c++)
    contacts_[contacts[c]].last_force = x.segment<3>(2 * n_rotor + 3 * c);
  for (auto& contact : contacts_)
  {
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = now;
    msg.header.frame_id = contact.name;
    const Eigen::Vector3d local = data_.oMf[contact.frame].rotation().transpose() * contact.last_force;
    tf::vectorEigenToMsg(local, msg.wrench.force);
    contact.force_pub.publish(msg);
  }
  geometry_msgs::WrenchStamped error;
  error.header.stamp = now;
  error.header.frame_id = world;
  const Eigen::Matrix<double, 6, 1> e = G * x - wrench;
  tf::vectorEigenToMsg(e.head<3>(), error.wrench.force);
  tf::vectorEigenToMsg(e.tail<3>(), error.wrench.torque);
  wrench_error_pub_.publish(error);

  sensor_msgs::JointState torques;
  torques.header.stamp = now;
  for (size_t r = 0; r < torque_limits_.size(); r++)
  {
    torques.name.push_back(torque_limited_joints_[r]);
    torques.position.push_back(torque_limits_[r].second);
    torques.effort.push_back(data_.g(torque_limits_[r].first) + T.row(r).dot(x));
  }
  joint_torque_pub_.publish(torques);
  return true;
}
}  // namespace aerial_robot_control

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorWholeBodyController, aerial_robot_control::ControlBase);
