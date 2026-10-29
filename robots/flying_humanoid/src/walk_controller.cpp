#include <flying_humanoid/walk_controller.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>
#include <spinal/DesireCoord.h>
#include <spinal/FlightConfigCmd.h>
#include <spinal/FourAxisCommand.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>

namespace flying_humanoid
{
void WalkController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                double ctrl_loop_du)
{
  ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  std::string robot_description;
  nh_.getParam("robot_description", robot_description);
  std::vector<RotorInfo> rotors;
  model_ = buildPinocchioModel(robot_description, &rotors);
  data_ = pinocchio::Data(model_);

  ros::NodeHandle walk_nh(nh_, "walk");
  walk_nh.param("gimbal_names", gimbal_names_, std::vector<std::string>{ "gimbal1", "gimbal2" });
  walk_nh.param("roll_p_gain", roll_p_gain_, roll_p_gain_);
  walk_nh.param("roll_d_gain", roll_d_gain_, roll_d_gain_);
  walk_nh.param("pitch_p_gain", pitch_p_gain_, pitch_p_gain_);
  walk_nh.param("pitch_d_gain", pitch_d_gain_, pitch_d_gain_);
  walk_nh.param("yaw_p_gain", yaw_p_gain_, yaw_p_gain_);
  walk_nh.param("yaw_d_gain", yaw_d_gain_, yaw_d_gain_);
  walk_nh.param("max_moment", max_moment_, max_moment_);
  walk_nh.param("sr_inverse_lambda", sr_inverse_lambda_, sr_inverse_lambda_);

  // servo_bridge takes each servo group on its own topic
  joints_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  gimbals_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  four_axis_command_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  desire_coord_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
  trajectory_loaded_pub_ = nh_.advertise<std_msgs::String>("walk/trajectory_loaded", 1, true);
  trajectory_sub_ = nh_.subscribe("walk/trajectory", 1, &WalkController::trajectoryCallback, this);
  start_sub_ = nh_.subscribe("walk/start", 1, &WalkController::startCallback, this);
}

void WalkController::trajectoryCallback(const std_msgs::String::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_trajectory_ = msg->data;
}

void WalkController::startCallback(const std_msgs::Bool::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  start_requested_ = msg->data;
}

bool WalkController::update()
{
  loadPendingTrajectory();

  if (!ControlBase::update())
    return false;

  if (first_run_)
  {
    spinal::FlightConfigCmd cmd;
    cmd.cmd = spinal::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
    navigator_->getFlightConfigPublisher().publish(cmd);
    first_run_ = false;
  }

  if (trajectory_.empty())
    return true;

  bool start_requested;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    start_requested = start_requested_;
  }
  if (!start_requested && replaying_)
    rewind();
  if (start_requested && !replaying_)
  {
    replaying_ = true;
    ROS_INFO("start walking");
  }

  const Sample& sample = trajectory_.at(index_);
  if (replaying_)
    publishJointCommand(sample);
  publishDesireCoord(sample);
  publishThrust(sample);

  // advance on the trajectory time; hold the last sample at the end
  if (replaying_ && index_ + 1 < trajectory_.size())
  {
    replay_time_ += ctrl_loop_du_;
    while (index_ + 1 < trajectory_.size() && trajectory_[index_ + 1].time <= replay_time_)
      ++index_;
    if (index_ + 1 == trajectory_.size())
      ROS_INFO("finished walking");
  }

  return true;
}

void WalkController::reset()
{
  ControlBase::reset();
  rewind();
  first_run_ = true;
}

void WalkController::rewind()
{
  replaying_ = false;
  index_ = 0;
  replay_time_ = 0.0;
}

void WalkController::loadPendingTrajectory()
{
  std::string filename;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // do not switch the trajectory in the middle of a walk
    const bool walking = replaying_ && index_ + 1 < trajectory_.size();
    if (pending_trajectory_.empty() || walking)
      return;
    filename = pending_trajectory_;
    pending_trajectory_.clear();
    start_requested_ = false;
  }

  if (!loadTrajectory(filename))
    return;

  rewind();
  publishJointCommand(trajectory_.front());
  ROS_INFO_STREAM("loaded the walk trajectory " << filename << " (" << trajectory_.size() << " samples)");

  std_msgs::String msg;
  msg.data = filename;
  trajectory_loaded_pub_.publish(msg);
}

bool WalkController::loadTrajectory(const std::string& filename)
{
  std::ifstream ifs(filename);
  std::string line;
  if (!ifs || !std::getline(ifs, line))
  {
    ROS_ERROR_STREAM("failed to read the walk trajectory " << filename);
    return false;
  }

  std::map<std::string, int> columns;
  {
    std::stringstream ss(line);
    std::string name;
    for (int i = 0; std::getline(ss, name, ','); ++i)
      columns[name] = i;
  }
  auto column = [&columns](const std::string& name) {
    auto it = columns.find(name);
    return it == columns.end() ? -1 : it->second;
  };
  std::vector<int> q_cols, v_cols, thrust_cols;
  for (int i = 0; i < model_.nq; ++i)
    q_cols.push_back(column("q" + std::to_string(i)));
  for (int i = 0; i < model_.nv; ++i)
    v_cols.push_back(column("v" + std::to_string(i)));
  for (int i = 0; i < motor_num_; ++i)
    thrust_cols.push_back(column("thrust" + std::to_string(i + 1)));
  for (const auto* cols : { &q_cols, &v_cols, &thrust_cols })
  {
    if (std::find(cols->begin(), cols->end(), -1) != cols->end() || column("time") < 0)
    {
      ROS_ERROR_STREAM("the walk trajectory " << filename << " lacks some columns (time, q*, v*, thrust*)");
      return false;
    }
  }

  std::vector<Sample> trajectory;
  while (std::getline(ifs, line))
  {
    std::vector<double> row;
    std::stringstream ss(line);
    std::string value;
    while (std::getline(ss, value, ','))
      row.push_back(std::stod(value));
    if (row.size() != columns.size())
      continue;

    Sample sample;
    sample.time = row[column("time")];
    sample.q.resize(model_.nq);
    sample.v.resize(model_.nv);
    sample.thrust.resize(motor_num_);
    for (int i = 0; i < model_.nq; ++i)
      sample.q(i) = row[q_cols[i]];
    for (int i = 0; i < model_.nv; ++i)
      sample.v(i) = row[v_cols[i]];
    for (int i = 0; i < motor_num_; ++i)
      sample.thrust(i) = row[thrust_cols[i]];
    trajectory.push_back(sample);
  }

  if (trajectory.empty())
  {
    ROS_ERROR_STREAM("the walk trajectory " << filename << " has no sample");
    return false;
  }
  trajectory_ = trajectory;
  return true;
}

void WalkController::publishJointCommand(const Sample& sample)
{
  sensor_msgs::JointState joints, gimbals;
  joints.header.stamp = gimbals.header.stamp = ros::Time::now();

  // joint 0 is "universe" and joint 1 is the floating base
  for (pinocchio::JointIndex i = 2; i < model_.njoints; ++i)
  {
    const std::string& name = model_.names[i];
    const bool is_gimbal = std::find(gimbal_names_.begin(), gimbal_names_.end(), name) != gimbal_names_.end();
    sensor_msgs::JointState& msg = is_gimbal ? gimbals : joints;
    msg.name.push_back(name);
    msg.position.push_back(sample.q(model_.joints[i].idx_q()));
  }

  joints_ctrl_pub_.publish(joints);
  gimbals_ctrl_pub_.publish(gimbals);
}

Eigen::Matrix3d WalkController::baselinkRotation(const Sample& sample)
{
  pinocchio::forwardKinematics(model_, data_, sample.q);
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[model_.getFrameId(robot_model_->getBaselinkName())].rotation();
}

void WalkController::publishDesireCoord(const Sample& sample)
{
  const Eigen::Quaterniond quat(baselinkRotation(sample));
  const KDL::Rotation rot = KDL::Rotation::Quaternion(quat.x(), quat.y(), quat.z(), quat.w());
  robot_model_->setCogDesireOrientation(rot);

  double roll, pitch, yaw;
  rot.GetRPY(roll, pitch, yaw);
  spinal::DesireCoord msg;
  msg.roll = roll;
  msg.pitch = pitch;
  msg.yaw = 0.0;
  desire_coord_pub_.publish(msg);
}

void WalkController::publishThrust(const Sample& sample)
{
  // attitude error of the baselink in its own frame
  const Eigen::Matrix3d R_ref = baselinkRotation(sample);
  const tf::Matrix3x3 R_tf = estimator_->getOrientation(Frame::BASELINK, estimate_mode_);
  Eigen::Matrix3d R;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      R(i, j) = R_tf[i][j];
  const Eigen::Vector3d e = pinocchio::log3(R.transpose() * R_ref);

  const tf::Vector3 omega_tf = estimator_->getAngularVel(Frame::BASELINK, estimate_mode_);
  Eigen::Vector3d omega_ref = Eigen::Vector3d::Zero();
  if (replaying_)
    omega_ref = sample.v.segment<3>(3);  // the root and the baselink share the orientation
  const Eigen::Vector3d omega_e = omega_ref - Eigen::Vector3d(omega_tf.x(), omega_tf.y(), omega_tf.z());

  Eigen::Vector3d moment(roll_p_gain_ * e.x() + roll_d_gain_ * omega_e.x(),
                         pitch_p_gain_ * e.y() + pitch_d_gain_ * omega_e.y(),
                         yaw_p_gain_ * e.z() + yaw_d_gain_ * omega_e.z());
  moment = moment.cwiseMax(-max_moment_).cwiseMin(max_moment_);

  // moment of each rotor per unit thrust, in the baselink frame
  const std::vector<Eigen::Vector3d> p = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> u = robot_model_->getRotorsNormalFromCog<Eigen::Vector3d>();
  const Eigen::Matrix3d R_baselink_cog = robot_model_->getCog2Baselink<Eigen::Affine3d>().linear().transpose();
  Eigen::MatrixXd Q(3, motor_num_);
  for (int i = 0; i < motor_num_; ++i)
    Q.col(i) = R_baselink_cog *
               (-robot_model_->getRotorDirection().at(i + 1) * robot_model_->getMFRate() * u[i] + p[i].cross(u[i]));
  const Eigen::MatrixXd Q_inv =
      Q.transpose() * (Q * Q.transpose() + sr_inverse_lambda_ * Eigen::Matrix3d::Identity()).inverse();

  const Eigen::VectorXd thrust = sample.thrust + Q_inv * moment;
  spinal::FourAxisCommand msg;
  for (int i = 0; i < motor_num_; ++i)
    msg.base_thrust.push_back(
        std::clamp(thrust(i), robot_model_->getThrustLowerLimit(), robot_model_->getThrustUpperLimit()));
  four_axis_command_pub_.publish(msg);
}

}  // namespace flying_humanoid

PLUGINLIB_EXPORT_CLASS(flying_humanoid::WalkController, aerial_robot_control::ControlBase);
