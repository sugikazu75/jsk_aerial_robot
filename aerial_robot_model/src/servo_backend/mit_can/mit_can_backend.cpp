// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <aerial_robot_model/servo_backend/mit_can/mit_can_backend.h>
#include <aerial_robot_model/servo_bridge.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace
{
/* parameter: servo specific param > group param */
XmlRpc::XmlRpcValue* findParam(XmlRpc::XmlRpcValue& servo_params, XmlRpc::XmlRpcValue& group_params,
                               const std::string& name)
{
  if (servo_params.getType() == XmlRpc::XmlRpcValue::TypeStruct && servo_params.hasMember(name))
    return &servo_params[name];
  if (group_params.getType() == XmlRpc::XmlRpcValue::TypeStruct && group_params.hasMember(name))
    return &group_params[name];
  return nullptr;
}

bool getParam(XmlRpc::XmlRpcValue& servo_params, XmlRpc::XmlRpcValue& group_params, const std::string& name,
              double& value)
{
  XmlRpc::XmlRpcValue* param = findParam(servo_params, group_params, name);
  if (!param)
    return false;
  if (param->getType() == XmlRpc::XmlRpcValue::TypeDouble)
    value = static_cast<double>(*param);
  else if (param->getType() == XmlRpc::XmlRpcValue::TypeInt)
    value = static_cast<int>(*param);
  else
    return false;
  return true;
}

bool getParam(XmlRpc::XmlRpcValue& servo_params, XmlRpc::XmlRpcValue& group_params, const std::string& name,
              bool& value)
{
  XmlRpc::XmlRpcValue* param = findParam(servo_params, group_params, name);
  if (!param || param->getType() != XmlRpc::XmlRpcValue::TypeBoolean)
    return false;
  value = static_cast<bool>(*param);
  return true;
}

bool getParam(XmlRpc::XmlRpcValue& servo_params, XmlRpc::XmlRpcValue& group_params, const std::string& name,
              std::string& value)
{
  XmlRpc::XmlRpcValue* param = findParam(servo_params, group_params, name);
  if (!param || param->getType() != XmlRpc::XmlRpcValue::TypeString)
    return false;
  value = static_cast<std::string>(*param);
  return true;
}
}  // namespace

MitCanBackend::MitCanBackend(std::mutex& state_mutex, bool use_can, double command_rate)
  : ServoBackend(ServoType::CUBEMARS), state_mutex_(state_mutex), use_can_(use_can), command_rate_(command_rate)
{
}

MitCanBackend::~MitCanBackend()
{
  stop();
}

void MitCanBackend::stop()
{
  running_ = false;
  if (command_thread_.joinable())
    command_thread_.join();

  for (auto& bus : buses_)
    bus.second->close();
}

bool MitCanBackend::addServo(SingleServoHandle* servo, XmlRpc::XmlRpcValue& group_params,
                             XmlRpc::XmlRpcValue& servo_params)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  const std::string& name = servo->getName();
  auto motor = std::make_shared<Motor>();
  motor->handle = servo;
  motor->id = servo->getId();

  if (motor->id < 0 || motor->id > 0xFF)
  {
    ROS_ERROR("[mit can backend]: invalid id %d for %s", motor->id, name.c_str());
    return false;
  }

  const auto& models = mitMotorModels();
  if (!getParam(servo_params, group_params, "model", motor->model_name) ||
      models.find(motor->model_name) == models.end())
  {
    std::string candidates;
    for (const auto& model : models)
      candidates += model.first + " ";
    ROS_ERROR("[mit can backend]: please set valid 'model' for %s, candidates: %s", name.c_str(), candidates.c_str());
    return false;
  }
  motor->model = models.at(motor->model_name);

  /* protocol is shared among the motors with the same firmware version */
  if (protocols_.find(motor->model.protocol) == protocols_.end())
  {
    MitProtocolPtr protocol = createMitProtocol(motor->model.protocol);
    if (!protocol)
    {
      ROS_ERROR("[mit can backend]: protocol %s of %s is not supported", motor->model.protocol.c_str(), name.c_str());
      return false;
    }
    protocols_.insert(std::make_pair(motor->model.protocol, protocol));
  }
  motor->protocol = protocols_.at(motor->model.protocol);

  motor->bus = "can0";
  getParam(servo_params, group_params, "bus", motor->bus);

  if (!getParam(servo_params, group_params, "kp", motor->kp) || !getParam(servo_params, group_params, "kd", motor->kd))
  {
    ROS_ERROR("[mit can backend]: please set 'kp' and 'kd' for %s", name.c_str());
    return false;
  }
  if (motor->kp < 0 || motor->kp > motor->model.kp_max || motor->kd < 0 || motor->kd > motor->model.kd_max)
  {
    ROS_WARN("[mit can backend]: kp %f, kd %f for %s are out of range [0, %f], [0, %f], clamp", motor->kp, motor->kd,
             name.c_str(), motor->model.kp_max, motor->model.kd_max);
    motor->kp = std::min(std::max(motor->kp, 0.0), motor->model.kp_max);
    motor->kd = std::min(std::max(motor->kd, 0.0), motor->model.kd_max);
  }

  motor->zero_point_offset = 0.0;
  getParam(servo_params, group_params, "zero_point_offset", motor->zero_point_offset);

  /* encoder on rotor side: the output angle at startup is ambiguous by 2pi / gear_ratio */
  motor->startup_angle_tolerance = M_PI / motor->model.gear_ratio;
  getParam(servo_params, group_params, "startup_angle_tolerance", motor->startup_angle_tolerance);

  motor->require_rough_calib = false;
  getParam(servo_params, group_params, "require_rough_calib", motor->require_rough_calib);

  const auto key = std::make_pair(motor->bus, motor->id);
  if (motors_.find(key) != motors_.end())
  {
    ROS_ERROR("[mit can backend]: id %d in %s is duplicated (%s and %s)", motor->id, motor->bus.c_str(),
              motors_.at(key)->handle->getName().c_str(), name.c_str());
    return false;
  }

  if (use_can_ && buses_.find(motor->bus) == buses_.end())
  {
    std::unique_ptr<CanBus> bus(new CanBus());
    const std::string bus_name = motor->bus;
    if (!bus->open(bus_name, [this, bus_name](const can_frame& frame) { canCallback(bus_name, frame); }))
      return false;
    buses_.insert(std::make_pair(bus_name, std::move(bus)));
  }

  motors_.insert(std::make_pair(key, motor));
  handle_motors_.insert(std::make_pair(servo, motor));

  /* start the periodic command only if there is a motor */
  if (use_can_ && command_rate_ > 0 && !running_)
  {
    running_ = true;
    command_thread_ = std::thread(&MitCanBackend::commandLoop, this);
  }

  ROS_INFO("[mit can backend]: %s, model: %s (protocol %s), bus: %s, id: %d, kp: %f, kd: %f", name.c_str(),
           motor->model_name.c_str(), motor->model.protocol.c_str(), motor->bus.c_str(), motor->id, motor->kp,
           motor->kd);
  return true;
}

MitCanBackend::MotorPtr MitCanBackend::findMotor(const SingleServoHandle& servo) const
{
  auto it = handle_motors_.find(&servo);
  if (it == handle_motors_.end())
    return MotorPtr();
  return it->second;
}

double MitCanBackend::motorToJoint(const Motor& motor, double motor_pos) const
{
  return motor.handle->getAngleSgn() * (motor_pos - motor.zero_point_offset - motor.turn_offset);
}

double MitCanBackend::jointToMotor(const Motor& motor, double joint_pos) const
{
  return motor.handle->getAngleSgn() * joint_pos + motor.zero_point_offset + motor.turn_offset;
}

void MitCanBackend::canCallback(const std::string& bus, const can_frame& frame)
{
  const ros::Time now = ros::Time::now();

  std::lock_guard<std::mutex> lock(state_mutex_);

  /* find the motor by the protocol which can parse this frame */
  MotorPtr motor_ptr;
  for (const auto& protocol : protocols_)
  {
    int id;
    if (!protocol.second->parseId(frame, id))
      continue;

    auto it = motors_.find(std::make_pair(bus, id));
    if (it != motors_.end() && it->second->protocol == protocol.second)
    {
      motor_ptr = it->second;
      break;
    }
  }
  if (!motor_ptr)
    return;

  Motor& motor = *motor_ptr;
  const std::string& name = motor.handle->getName();

  MitStatus status;
  if (!motor.protocol->parseStatus(frame, motor.model, status))
    return;

  if (status.disabled)
  {
    ROS_INFO("[mit can backend]: %s is disabled", name.c_str());
  }
  else
  {
    if (status.error_code != 0 && status.error_code != motor.error_code)
      ROS_ERROR("[mit can backend]: %s error %d (%s)", name.c_str(), status.error_code,
                motor.protocol->errorString(status.error_code).c_str());
    motor.error_code = status.error_code;
  }

  motor.motor_pos = status.position;

  if (!motor.state_received)
  {
    /* the output angle at startup is ambiguous by 2pi / gear_ratio due to the encoder on rotor side */
    const double joint_pos = motorToJoint(motor, status.position);
    const double period = 2.0 * M_PI / motor.model.gear_ratio;
    if (motor.require_rough_calib)
    {
      motor.calibrated = false;
      ROS_WARN(
          "[mit can backend]: %s requires rough calibration before torque enable. move the joint near zero "
          "(within +-%.1f deg), and call rough_calib service",
          name.c_str(), period / 2 * 180 / M_PI);
    }
    else if (std::fabs(joint_pos) > motor.startup_angle_tolerance)
    {
      motor.calibrated = false;
      ROS_ERROR(
          "[mit can backend]: %s starts at %.1f deg, which is far from zero (tolerance %.1f deg). the angle may be "
          "shifted by a multiple of %.1f deg. move the joint near zero, and call rough_calib service",
          name.c_str(), joint_pos * 180 / M_PI, motor.startup_angle_tolerance * 180 / M_PI, period * 180 / M_PI);
    }
    else
    {
      motor.calibrated = true;
    }
    motor.state_received = true;
  }

  motor.handle->setCurrAngleVal(motorToJoint(motor, status.position), ValueType::RADIAN);
  motor.handle->setCurrVelocityVal(motor.handle->getAngleSgn() * status.velocity);
  motor.handle->setCurrTorqueVal(status.torque);  // angle_sgn (and torque_scale) is applied in the handle
  motor.handle->markStateReceived(now);
}

void MitCanBackend::addFrame(const Motor& motor, const can_frame& frame)
{
  auto bus = buses_.find(motor.bus);
  if (bus == buses_.end())
    return;
  pending_.push_back(std::make_pair(bus->second.get(), frame));
}

void MitCanBackend::setMitCommand(Motor& motor, const MitCommand& cmd)
{
  const MitMotorModel& m = motor.model;
  if (std::fabs(cmd.position) > m.p_max)
    ROS_WARN_THROTTLE(1.0, "[mit can backend]: target position %f of %s is out of range +-%f, clamp", cmd.position,
                      motor.handle->getName().c_str(), m.p_max);
  if (std::fabs(cmd.torque) > m.t_max)
    ROS_WARN_THROTTLE(1.0, "[mit can backend]: target torque %f of %s is out of range +-%f, clamp", cmd.torque,
                      motor.handle->getName().c_str(), m.t_max);

  /* send immediately, and re-send in commandLoop() */
  motor.cmd = cmd;
  addFrame(motor, motor.protocol->makeMitFrame(motor.id, motor.model, motor.cmd));
}

void MitCanBackend::commandLoop()
{
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / command_rate_));
  auto next = std::chrono::steady_clock::now();

  while (running_)
  {
    {
      /*
        Send under the lock to keep the order with flush(): a frame built before disable must not be sent after the
        disable frame. The send is non-blocking.
      */
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (const auto& motor : motors_)
      {
        const Motor& m = *motor.second;
        auto bus = buses_.find(m.bus);
        if (m.enabled && bus != buses_.end())
          bus->second->send(m.protocol->makeMitFrame(m.id, m.model, m.cmd));
      }
    }

    next += period;
    const auto now = std::chrono::steady_clock::now();
    if (next < now)
      next = now;  // avoid burst after delay
    std::this_thread::sleep_until(next);
  }
}

void MitCanBackend::addCommand(const SingleServoHandle& servo, const ServoCommand& cmd)
{
  MotorPtr motor = findMotor(servo);
  if (!motor || !motor->enabled)
    return;

  const double sgn = servo.getAngleSgn();
  MitCommand mit_cmd;
  if (cmd.has_position)
  {
    /* position control with velocity and feed-forward torque */
    mit_cmd.position = jointToMotor(*motor, servo.getTargetAngleVal(ValueType::RADIAN));  // clamped by joint limits
    mit_cmd.velocity = cmd.has_velocity ? sgn * cmd.velocity : 0.0;
    mit_cmd.kp = motor->kp;
    mit_cmd.kd = motor->kd;
    mit_cmd.torque = cmd.has_effort ? sgn * cmd.effort : 0.0;
  }
  else if (cmd.has_effort)
  {
    /* pure torque control */
    mit_cmd.torque = sgn * cmd.effort;
  }
  else
  {
    return;
  }

  setMitCommand(*motor, mit_cmd);
}

bool MitCanBackend::addTorqueEnable(const SingleServoHandle& servo, bool enable)
{
  MotorPtr motor = findMotor(servo);
  if (!motor)
    return false;

  if (!use_can_)
  {
    motor->enabled = enable;
    return true;
  }

  if (!enable)
  {
    motor->enabled = false;
    for (const auto& frame : motor->protocol->makeDisableFrames(motor->id))
      addFrame(*motor, frame);
    return true;
  }

  if (!motor->state_received)
  {
    ROS_ERROR("[mit can backend]: can not enable %s, no state from motor", servo.getName().c_str());
    return false;
  }
  if (!motor->calibrated)
  {
    ROS_ERROR(
        "[mit can backend]: can not enable %s, the joint angle is not reliable. move the joint near zero, and "
        "call rough_calib service",
        servo.getName().c_str());
    return false;
  }
  if (motor->error_code != 0)
    ROS_WARN("[mit can backend]: enable %s with error %d (%s)", servo.getName().c_str(), motor->error_code,
             motor->protocol->errorString(motor->error_code).c_str());

  if (!motor->enabled)
  {
    motor->enabled = true;
    for (const auto& frame : motor->protocol->makeEnableFrames(motor->id))
      addFrame(*motor, frame);

    /* hold the current position */
    motor->handle->setTargetAngleVal(motorToJoint(*motor, motor->motor_pos), ValueType::RADIAN);
    MitCommand hold_cmd;
    hold_cmd.position = motor->motor_pos;
    hold_cmd.kp = motor->kp;
    hold_cmd.kd = motor->kd;
    setMitCommand(*motor, hold_cmd);
  }
  return true;
}

bool MitCanBackend::roughCalib(const SingleServoHandle& servo, std::string& message)
{
  MotorPtr motor = findMotor(servo);
  if (!motor)
  {
    message = servo.getName() + ": not managed by this backend";
    return false;
  }

  if (!use_can_)
  {
    message = servo.getName() + ": skip in simulation";
    return true;
  }

  if (!motor->state_received)
  {
    message = servo.getName() + ": no state from motor";
    return false;
  }

  if (motor->enabled)
  {
    message = servo.getName() + ": disable torque before rough calibration";
    return false;
  }

  /* assume the joint is near zero: remove the multiple of 2pi / gear_ratio from the current angle */
  const double period = 2.0 * M_PI / motor->model.gear_ratio;
  const double relative_pos = motor->motor_pos - motor->zero_point_offset - motor->turn_offset;
  const int n = static_cast<int>(std::round(relative_pos / period));
  motor->turn_offset += n * period;
  motor->calibrated = true;

  const double joint_pos = motorToJoint(*motor, motor->motor_pos);
  motor->handle->setCurrAngleVal(joint_pos, ValueType::RADIAN);
  motor->handle->setTargetAngleVal(joint_pos, ValueType::RADIAN);

  char buf[256];
  snprintf(buf, sizeof(buf), "%s: shift %.1f deg (%d x %.1f deg), joint angle %.1f deg", servo.getName().c_str(),
           -n * period * 180 / M_PI, -n, period * 180 / M_PI, joint_pos * 180 / M_PI);
  message = buf;
  ROS_INFO("[mit can backend]: rough calibration, %s", buf);
  return true;
}

void MitCanBackend::flush()
{
  for (auto& frame : pending_)
    frame.first->send(frame.second);
  pending_.clear();
}

void MitCanBackend::clear()
{
  pending_.clear();
}
