// -*- mode: c++ -*-

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <aerial_robot_control/control/fwddyn_mpc_controller.hpp>

#include <tf/transform_datatypes.h>
#include <aerial_robot_estimation/state_estimation.h>

namespace aerial_robot_control
{

FwddynMpcController::FwddynMpcController()
  : ControlBase(), joint_state_received_(false), n_joints_(0), mpc_parameters_()
{
}

void FwddynMpcController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  pinocchio_robot_model_ = std::make_shared<aerial_robot_dynamics::PinocchioRobotModel>(true);
  pin_model_ = pinocchio_robot_model_->getModel();

  // load MPC parameters
  ros::NodeHandle mpc_nh(nh_, "mpc");
  mpc_nh.param<int>("num_nodes", mpc_parameters_.num_nodes, mpc_parameters_.num_nodes);
  mpc_nh.param<int>("max_iter", mpc_parameters_.max_iter, mpc_parameters_.max_iter);
  mpc_nh.param<int>("max_init_iter", mpc_parameters_.max_init_iter, mpc_parameters_.max_init_iter);
  mpc_nh.param<double>("dt", mpc_parameters_.dt, mpc_parameters_.dt);
  mpc_nh.param<double>("control_weight", mpc_parameters_.control_weight, mpc_parameters_.control_weight);
  mpc_nh.param<double>("thrust_reg_weight", mpc_parameters_.thrust_reg_weight, mpc_parameters_.thrust_reg_weight);
  mpc_nh.param<double>("thrust_barrier_weight", mpc_parameters_.thrust_barrier_weight,
                       mpc_parameters_.thrust_barrier_weight);
  mpc_nh.param<double>("delta_thrust_max", mpc_parameters_.delta_thrust_max, mpc_parameters_.delta_thrust_max);

  // CoM tracking weight (3 elements)
  std::vector<double> com_w;
  if (mpc_nh.getParam("com_track_weight", com_w))
  {
    if ((int)com_w.size() == 3)
    {
      Eigen::Vector3d com_track_weight = Eigen::Map<Eigen::Vector3d>(com_w.data());
      mpc_parameters_.com_track_weight = com_track_weight;
    }
    else
      ROS_ERROR_STREAM("[FwddynMpcController] com_track_weight size mismatch: expected 3, got " << com_w.size());
  }
  else
    ROS_ERROR_STREAM("[FwddynMpcController] Failed to get com_track_weight from parameter server.");

  // centroidal momentum weight (6 elements)
  std::vector<double> centroidal_momentum_w;
  if (mpc_nh.getParam("centroidal_momentum_weight", centroidal_momentum_w))
  {
    if ((int)centroidal_momentum_w.size() == 6)
    {
      Eigen::VectorXd centroidal_momentum_weight =
          Eigen::Map<Eigen::VectorXd>(centroidal_momentum_w.data(), centroidal_momentum_w.size());
      mpc_parameters_.centroidal_momentum_weight = centroidal_momentum_weight;
    }
    else
      ROS_ERROR_STREAM("[FwddynMpcController] centroidal_momentum_weight size mismatch: expected 6, got "
                       << centroidal_momentum_w.size());
  }
  else
    ROS_ERROR_STREAM("[FwddynMpcController] Failed to get centroidal_momentum_weight from parameter server.");

  // state weight (2*nv elements)
  std::vector<double> x_w;
  if (mpc_nh.getParam("x_state_weight", x_w))
  {
    if ((int)x_w.size() == 2 * pin_model_->nv)
    {
      Eigen::VectorXd x_state_weight = Eigen::Map<Eigen::VectorXd>(x_w.data(), x_w.size());
      mpc_parameters_.x_state_weight = x_state_weight;
    }
    else
      ROS_ERROR_STREAM("[FwddynMpcController] x_state_weight size mismatch: expected " << 2 * pin_model_->nv << ", got "
                                                                                       << x_w.size());
  }
  else
    ROS_ERROR_STREAM("[FwddynMpcController] Failed to get x_state_weight from parameter server.");

  // state reference
  std::vector<double> x_ref;
  if (mpc_nh.getParam("x_ref", x_ref))
  {
    if ((int)x_ref.size() == pin_model_->nq + pin_model_->nv)
      x_ref_ = Eigen::Map<Eigen::VectorXd>(x_ref.data(), x_ref.size());
    else
      ROS_ERROR_STREAM("[FwddynMpcController] x_ref size mismatch: expected " << pin_model_->nq + pin_model_->nv
                                                                              << ", got " << x_ref.size());
  }
  else
    ROS_ERROR_STREAM("[FwddynMpcController] Failed to get x_ref from parameter server.");

  x_ref_default_ = x_ref_;

  mpc_parameters_.print();

  mpc_nh.param<double>("default_joint_vel", default_joint_vel_, default_joint_vel_);

  // publishers
  four_axis_command_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  joints_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 10);
  target_cog_pos_pub_ = nh_.advertise<geometry_msgs::PointStamped>("mpc_target_cog_pos", 10);
  thrust_rate_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("mpc/thrust_rate", 10);

  // subscribers
  joint_state_sub_ = nh_.subscribe("joint_states", 10, &FwddynMpcController::jointStateCallback, this);
  target_joint_state_sub_ =
      nh_.subscribe("final_target_joint_state", 1, &FwddynMpcController::targetJointStateCallback, this);

  curr_q_ = Eigen::VectorXd::Zero(pin_model_->nq);
  curr_dq_ = Eigen::VectorXd::Zero(pin_model_->nv);
  curr_tau_ = Eigen::VectorXd::Zero(pin_model_->nv);

  const int rotor_num = pinocchio_robot_model_->getRotorNum();
  n_joints_ = pin_model_->nv - 6;

  // build joint-name -> pinocchio index maps (skip universe=0 and root_joint=1)
  for (pinocchio::JointIndex i = 2; i < (pinocchio::JointIndex)pin_model_->njoints; i++)
  {
    const std::string& joint_name = pin_model_->names[i];
    const int q_idx = pin_model_->joints[i].idx_q();
    const int v_idx = pin_model_->joints[i].idx_v();
    joint_name_to_q_idx_[joint_name] = q_idx;
    joint_name_to_v_idx_[joint_name] = v_idx;
    joint_targets_[joint_name] = { x_ref_default_(q_idx), 0.0 };
  }

  // initialize MPC problem
  mpc_problem_.initialize(pinocchio_robot_model_, mpc_parameters_);

  ROS_INFO("[FwddynMpcController] nq=%d nv=%d rotor_num=%d n_joints=%d nodes=%d dt=%.3f", pin_model_->nq,
           pin_model_->nv, rotor_num, n_joints_, mpc_parameters_.num_nodes, mpc_parameters_.dt);
}

void FwddynMpcController::activate()
{
  ControlBase::activate();

  Eigen::VectorXd x = buildCurrentState();
  tf::Vector3 root_rpy = estimator_->getEuler(Frame::ROOT, estimate_mode_);
  Eigen::Quaterniond root_quat_yaw(Eigen::AngleAxisd(root_rpy.z(), Eigen::Vector3d::UnitZ()));
  root_quat_yaw.normalize();
  x.segment(3, 4) << root_quat_yaw.x(), root_quat_yaw.y(), root_quat_yaw.z(), root_quat_yaw.w();
  x.segment(pin_model_->nq, pin_model_->nv) = Eigen::VectorXd::Zero(pin_model_->nv);  // zero joint velocity
  x.tail(motor_num_) = Eigen::VectorXd::Zero(motor_num_);

  std::cout << "[FwddynMpcController] Initial state (with yaw-only orientation): " << x.transpose() << std::endl;

  // update root link reference to current state
  x_ref_.head(7) = x.head(7);

  std::cout << "[FwddynMpcController] Initial reference state: " << x_ref_.transpose() << std::endl;

  // CoM reference
  const tf::Vector3 target_cog_pos = navigator_->getTargetPos();
  const Eigen::Vector3d com_target(target_cog_pos.x(), target_cog_pos.y(), target_cog_pos.z());
  std::cout << "[FwddynMpcController] Initial CoM target: " << com_target.transpose() << std::endl;

  // build and solveMPC
  mpc_problem_.buildMPCProblem(x, com_target, x_ref_);
  mpc_problem_.solveMPC(mpc_parameters_.max_init_iter, true, false);

  // Visualize the optimized root trajectory
  broadcastOptimizedRootTransforms();
}

bool FwddynMpcController::update()
{
  if (!ControlBase::update())
    return false;

  if (!robot_model_->initialized())
    return false;

  controlCore();
  sendCmd();
  return true;
}

void FwddynMpcController::reset()
{
  ControlBase::reset();

  ROS_INFO_STREAM("[FwddynMpcController] reset");
  mpc_elapsed_time_ = 0.0;

  // restore joint targets and x_ref joint part to the initial YAML values
  for (const auto& [name, q_idx] : joint_name_to_q_idx_)
  {
    const int v_idx = joint_name_to_v_idx_.at(name);
    joint_targets_[name] = { x_ref_default_(q_idx), 0.0 };
    x_ref_(q_idx) = x_ref_default_(q_idx);
    x_ref_(pin_model_->nq + v_idx) = 0.0;
  }
}

void FwddynMpcController::controlCore()
{
  const tf::Vector3 target_cog_pos = navigator_->getTargetPos();
  const Eigen::Vector3d com_target(target_cog_pos.x(), target_cog_pos.y(), target_cog_pos.z());

  Eigen::VectorXd x0 = buildCurrentState();

  mpc_problem_.setInitialState(x0);

  updateJointRefInterpolation();

  mpc_elapsed_time_ += ctrl_loop_du_;
  if (mpc_elapsed_time_ >= mpc_parameters_.dt - 1e-9)
  {
    mpc_problem_.slideHorizon(com_target, x_ref_);
    mpc_elapsed_time_ -= mpc_parameters_.dt;
  }

  mpc_problem_.solveMPC(mpc_parameters_.max_iter, false, true);

  control_timestamp_ = ros::Time::now().toSec();
}

void FwddynMpcController::sendCmd()
{
  const auto& xs = mpc_problem_.xs();
  const auto& us = mpc_problem_.us();
  if (xs.size() < 2 || us.empty())
    return;

  const int rotor_num = pinocchio_robot_model_->getRotorNum();
  const int nq = pin_model_->nq;
  const int nv = pin_model_->nv;

  // commanded thrust = thrust component of next predicted state
  const Eigen::VectorXd thrust = xs[1].segment(nq + nv, rotor_num);
  if (!thrust.array().isFinite().all())
  {
    ROS_WARN_THROTTLE(1.0, "[FwddynMpcController] NaN/Inf in thrust, skipping sendCmd");
    return;
  }

  spinal::FourAxisCommand cmd;
  cmd.angles[0] = 0.0f;
  cmd.angles[1] = 0.0f;
  cmd.angles[2] = 0.0f;
  cmd.base_thrust.resize(rotor_num);
  for (int i = 0; i < rotor_num; i++)
    cmd.base_thrust[i] = static_cast<float>(thrust(i));

  four_axis_command_pub_.publish(cmd);

  // optimized thrust rate
  {
    std_msgs::Float64MultiArray rate_msg;
    rate_msg.data.resize(rotor_num);
    for (int i = 0; i < rotor_num; i++)
      rate_msg.data[i] = us[0](i);
    thrust_rate_pub_.publish(rate_msg);
  }

  if (n_joints_ > 0)
    publishJointsCtrl();

  if (ros::Time::now().toSec() - last_tf_broadcast_time_ >= tf_broadcast_duration_)
  {
    broadcastOptimizedRootTransforms();
    last_tf_broadcast_time_ = ros::Time::now().toSec();
  }

  if (ros::Time::now().toSec() - last_target_cog_pos_pub_time_ >= target_cog_pos_pub_duration_)
  {
    // publish the target CoM position for visualization
    geometry_msgs::PointStamped target_cog_msg;
    const tf::Vector3 target_cog_pos = navigator_->getTargetPos();
    target_cog_msg.header.stamp = ros::Time::now();
    target_cog_msg.header.frame_id = "world";
    target_cog_msg.point.x = target_cog_pos.x();
    target_cog_msg.point.y = target_cog_pos.y();
    target_cog_msg.point.z = target_cog_pos.z();
    target_cog_pos_pub_.publish(target_cog_msg);
    last_target_cog_pos_pub_time_ = ros::Time::now().toSec();
  }
}

Eigen::VectorXd FwddynMpcController::buildCurrentState()
{
  const int nq = pin_model_->nq;
  const int nv = pin_model_->nv;
  const int rotor_num = pinocchio_robot_model_->getRotorNum();

  // root
  Eigen::VectorXd x = Eigen::VectorXd::Zero(nq + nv + rotor_num);
  tf::Vector3 root_pos = estimator_->getPos(Frame::ROOT, estimate_mode_);
  tf::Matrix3x3 root_rot = estimator_->getOrientation(Frame::ROOT, estimate_mode_);
  tf::Vector3 root_vel = estimator_->getVel(Frame::ROOT_LOCAL, estimate_mode_);
  tf::Vector3 root_angular_vel = estimator_->getAngularVel(Frame::ROOT, estimate_mode_);

  x.head(3) << root_pos.x(), root_pos.y(), root_pos.z();
  tf::Quaternion root_quat;
  root_rot.getRotation(root_quat);
  x.segment(3, 4) << root_quat.x(), root_quat.y(), root_quat.z(), root_quat.w();
  x.segment(nq, 3) << root_vel.x(), root_vel.y(), root_vel.z();
  x.segment(nq + 3, 3) << root_angular_vel.x(), root_angular_vel.y(), root_angular_vel.z();

  // joints
  if (n_joints_ > 0)
  {
    x.segment(7, nq - 7) = curr_q_.segment(7, nq - 7);
    x.segment(nq + 6, nv - 6) = curr_dq_.segment(6, nv - 6);
  }

  // thrusts
  const auto& xs = mpc_problem_.xs();
  if (!xs.empty() && xs[0].size() == x.size())
    x.tail(rotor_num) = xs[0].tail(rotor_num);
  else
    x.tail(rotor_num) = Eigen::VectorXd::Zero(rotor_num);

  return x;
}

void FwddynMpcController::broadcastOptimizedRootTransforms()
{
  const auto& xs = mpc_problem_.xs();
  if (xs.empty())
    return;

  const ros::Time stamp = ros::Time::now();
  for (size_t i = 0; i < xs.size(); ++i)
  {
    if ((int)xs[i].size() < 7 || !xs[i].head(7).array().isFinite().all())
      continue;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(xs[i](0), xs[i](1), xs[i](2)));
    tf::Quaternion quat(xs[i](3), xs[i](4), xs[i](5), xs[i](6));
    quat.normalize();
    transform.setRotation(quat);

    optimized_root_tf_broadcaster_.sendTransform(
        tf::StampedTransform(transform, stamp, "world", "mpc_optimized_root_" + std::to_string(i)));
  }
}

void FwddynMpcController::publishJointsCtrl()
{
  const int rotor_num = pinocchio_robot_model_->getRotorNum();
  const auto& xs = mpc_problem_.xs();
  const auto& us = mpc_problem_.us();

  if (xs.size() < 2 || !xs[1].array().isFinite().all())
    return;

  sensor_msgs::JointState joint_state;
  for (pinocchio::JointIndex i = 2; i < (pinocchio::JointIndex)pin_model_->njoints; i++)
  {
    const std::string& joint_name = pin_model_->names[i];
    const int q_idx = pin_model_->joints[pin_model_->getJointId(joint_name)].idx_q();
    const int v_idx = pin_model_->joints[pin_model_->getJointId(joint_name)].idx_v();

    joint_state.name.push_back(joint_name);
    joint_state.position.push_back(xs[1](q_idx));
    joint_state.velocity.push_back(xs[1](pin_model_->nq + v_idx));
    joint_state.effort.push_back(us[0](rotor_num + (v_idx - 6)));  // skip free-flyer DOFs
  }

  joints_ctrl_pub_.publish(joint_state);
}

void FwddynMpcController::targetJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if (msg->name.size() != msg->position.size())
  {
    ROS_WARN_STREAM("[FwddynMpcController] Received target_joint_state with name/position size mismatch: "
                    << msg->name.size() << " vs " << msg->position.size());
    return;
  }

  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    auto it = joint_targets_.find(msg->name[i]);
    if (it == joint_targets_.end())
      continue;

    // Update target position
    it->second.position = msg->position[i];

    // Update target velocity if provided and valid; otherwise, keep the previous velocity (which may be 0)
    if (msg->velocity.size() == msg->position.size())
    {
      it->second.velocity = (std::isfinite(msg->velocity[i]) && msg->velocity[i] > 0.0) ? msg->velocity[i] : 0.0;
    }
  }
}

void FwddynMpcController::updateJointRefInterpolation()
{
  const double dt = ctrl_loop_du_;

  for (const auto& [name, target] : joint_targets_)
  {
    const int q_idx = joint_name_to_q_idx_.at(name);
    const int v_idx = joint_name_to_v_idx_.at(name);
    const double vel = (target.velocity > 0.0) ? target.velocity : default_joint_vel_;
    const double error = target.position - x_ref_(q_idx);
    const double step = vel * dt;

    if (std::abs(error) <= step)
    {
      x_ref_(q_idx) = target.position;
      x_ref_(pin_model_->nq + v_idx) = 0.0;
    }
    else
    {
      const double sign = (error > 0.0) ? 1.0 : -1.0;
      x_ref_(q_idx) += sign * step;
      x_ref_(pin_model_->nq + v_idx) = sign * vel;
    }
  }
}

void FwddynMpcController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  for (size_t i = 0; i < msg->position.size(); i++)
  {
    int joint_id = pin_model_->getJointId(msg->name[i]);
    if (joint_id >= 0 && joint_id < pin_model_->njoints)
    {
      int joint_index_q = pin_model_->joints[joint_id].idx_q();
      int joint_index_v = pin_model_->joints[joint_id].idx_v();
      curr_q_[joint_index_q] = msg->position[i];
      curr_dq_[joint_index_v] = msg->velocity[i];
      curr_tau_[joint_index_v] = msg->effort[i];
    }
  }
}

}  // namespace aerial_robot_control

// plugin registration
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::FwddynMpcController, aerial_robot_control::ControlBase);
