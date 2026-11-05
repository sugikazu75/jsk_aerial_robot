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

#ifndef SERVO_BACKEND_MIT_CAN_XIAOMI_PROTOCOL_H
#define SERVO_BACKEND_MIT_CAN_XIAOMI_PROTOCOL_H

#include <aerial_robot_model/servo_backend/mit_can/mit_protocol.h>

/*
  Protocol of Xiaomi CyberGear and RobStride motors, extended frame.
  - operation control (MIT) command: type 1, id = (1 << 24) | (torque(16) << 8) | motor id
    [position(16), velocity(16), kp(16), kd(16)]
  - enable: type 3, stop: type 4, write parameter: type 18 (run_mode: 0x7005, 0 is operation control mode)
  - feedback: type 2, id = (2 << 24) | (mode(2) << 22) | (fault(6) << 16) | (motor id << 8) | host id
    [position(16), velocity(16), torque(16), temperature(16, 0.1 deg C)]
    The feedback is sent only as the reply to the frames above (there is no periodic upload),
    so the stop frame is sent to get the state while the motor is disabled.
    The motor does not need the continuous command to stay enabled unless CAN_TIMEOUT is set (default: 0).
*/
class XiaomiProtocol : public MitProtocol
{
public:
  std::vector<can_frame> makeEnableFrames(int id) const override;
  std::vector<can_frame> makeDisableFrames(int id) const override;
  std::vector<can_frame> makeStatusRequestFrames(int id) const override;
  can_frame makeMitFrame(int id, const MitMotorModel& model, const MitCommand& cmd) const override;

  bool parseId(const can_frame& frame, int& id) const override;
  bool parseStatus(const can_frame& frame, const MitMotorModel& model, MitStatus& status) const override;

  std::string errorString(int error_code) const override;

private:
  static can_frame makeFrame(uint32_t type, uint32_t data, int id);
};

#endif
