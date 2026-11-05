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

#ifndef SERVO_BACKEND_MIT_CAN_CUBEMARS_V3_PROTOCOL_H
#define SERVO_BACKEND_MIT_CAN_CUBEMARS_V3_PROTOCOL_H

#include <aerial_robot_model/servo_backend/mit_can/mit_protocol.h>

/*
  v3 protocol (e.g., AKE60-8, AK40-10 V3.0), extended frame id = (packet id << 8) | motor id
  - force control (MIT) command: packet 8, [kp(12), kd(12), position(16), velocity(12), torque(12)]
  - status: packet 0x29, periodic upload from motor (periodic feedback mode should be set by the upper computer)
    [int16 position (0.1 deg), int16 speed (10 ERPM), int16 current (0.01 A), int8 temperature, uint8 error]
  - disable: packet 15, the motor responds the status with error code 0x77
  - no enable command: the motor is enabled by the MIT command, and disabled if the command stops
*/
class CubemarsV3Protocol : public MitProtocol
{
public:
  std::vector<can_frame> makeEnableFrames(int id) const override;
  std::vector<can_frame> makeDisableFrames(int id) const override;
  can_frame makeMitFrame(int id, const MitMotorModel& model, const MitCommand& cmd) const override;

  bool parseId(const can_frame& frame, int& id) const override;
  bool parseStatus(const can_frame& frame, const MitMotorModel& model, MitStatus& status) const override;

  std::string errorString(int error_code) const override;

private:
  static can_frame makeFrame(uint32_t packet_id, int id, const uint8_t* data, uint8_t len);
};

#endif
