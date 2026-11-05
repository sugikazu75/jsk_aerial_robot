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

#ifndef SERVO_BACKEND_MIT_CAN_MIT_CAN_BACKEND_H
#define SERVO_BACKEND_MIT_CAN_MIT_CAN_BACKEND_H

#include <aerial_robot_model/servo_backend/can_bus.h>
#include <aerial_robot_model/servo_backend/mit_can/mit_protocol.h>
#include <aerial_robot_model/servo_backend/servo_backend.h>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

/*
  Motors with force control (MIT) mode via SocketCAN (e.g., CubeMars).
  The protocol difference of the vendor and firmware version is handled in MitProtocol,
  and this backend handles the joint space conversion, torque enable, rough calibration, and periodic command.
  - the status frames are received in the receive thread of each CAN bus,
    and the servo handlers are updated under the lock of ServoBridge (state_mutex).
  - the latest command of each enabled motor is re-sent periodically (command_rate),
    since the motor is disabled if the command stops (communication timeout of the motor).
*/
class MitCanBackend : public ServoBackend
{
public:
  MitCanBackend(std::mutex& state_mutex, bool use_can, double command_rate);
  ~MitCanBackend() override;

  /* note: called without the lock of ServoBridge */
  bool addServo(SingleServoHandle* servo, XmlRpc::XmlRpcValue& group_params,
                XmlRpc::XmlRpcValue& servo_params) override;

  void addCommand(const SingleServoHandle& servo, const ServoCommand& cmd) override;
  bool addTorqueEnable(const SingleServoHandle& servo, bool enable) override;
  void flush() override;
  void clear() override;
  bool roughCalib(const SingleServoHandle& servo, std::string& message) override;
  void stop() override;

private:
  struct Motor
  {
    SingleServoHandle* handle;
    std::string bus;
    int id;
    std::string model_name;
    MitMotorModel model;
    MitProtocolPtr protocol;
    double kp;
    double kd;
    double zero_point_offset;        // [rad], motor position at joint zero
    double turn_offset = 0;          // [rad], correction of the multi-turn ambiguity by rough calibration
    double startup_angle_tolerance;  // [rad]
    bool require_rough_calib;        // always require rough calibration before torque enable
    bool enabled = false;
    bool state_received = false;
    bool calibrated = false;  // the joint angle is reliable
    double motor_pos = 0;     // [rad], raw position from motor
    int error_code = 0;
    MitCommand cmd;  // latest command in motor side, re-sent periodically while enabled
  };
  using MotorPtr = std::shared_ptr<Motor>;
  using BusFrame = std::pair<CanBus*, can_frame>;

  void canCallback(const std::string& bus, const can_frame& frame);
  void commandLoop();
  void addFrame(const Motor& motor, const can_frame& frame);
  void setMitCommand(Motor& motor, const MitCommand& cmd);
  double motorToJoint(const Motor& motor, double motor_pos) const;
  double jointToMotor(const Motor& motor, double joint_pos) const;
  MotorPtr findMotor(const SingleServoHandle& servo) const;

  std::mutex& state_mutex_;
  bool use_can_;
  double command_rate_;  // [Hz]
  std::atomic<bool> running_{ false };
  std::thread command_thread_;
  std::map<std::string, MitProtocolPtr> protocols_;  // protocol name -> protocol
  std::map<std::string, std::unique_ptr<CanBus>> buses_;
  std::map<std::pair<std::string, int>, MotorPtr> motors_;  // (bus, id) -> motor
  std::map<const SingleServoHandle*, MotorPtr> handle_motors_;
  std::vector<BusFrame> pending_;
};

#endif
