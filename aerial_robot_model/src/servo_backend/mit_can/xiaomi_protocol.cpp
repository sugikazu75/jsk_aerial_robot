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

#include <aerial_robot_model/servo_backend/mit_can/xiaomi_protocol.h>

#include <cstring>

namespace
{
const uint32_t TYPE_MOTION_CONTROL = 1;
const uint32_t TYPE_FEEDBACK = 2;
const uint32_t TYPE_ENABLE = 3;
const uint32_t TYPE_STOP = 4;
const uint32_t TYPE_WRITE_PARAM = 18;

const uint32_t HOST_ID = 0xFE;
const uint16_t PARAM_RUN_MODE = 0x7005;
const uint8_t RUN_MODE_OPERATION_CONTROL = 0;
const uint32_t MODE_MOTOR = 2;  // mode in the feedback, 0: reset, 1: calibration, 2: motor (running)
}  // namespace

can_frame XiaomiProtocol::makeFrame(uint32_t type, uint32_t data, int id)
{
  can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = CAN_EFF_FLAG | (type << 24) | ((data & 0xFFFF) << 8) | (static_cast<uint32_t>(id) & 0xFF);
  frame.can_dlc = 8;
  return frame;
}

std::vector<can_frame> XiaomiProtocol::makeEnableFrames(int id) const
{
  /* make sure the operation control mode. the manual writes run_mode before the enable frame */
  can_frame run_mode = makeFrame(TYPE_WRITE_PARAM, HOST_ID, id);
  run_mode.data[0] = PARAM_RUN_MODE & 0xFF;
  run_mode.data[1] = PARAM_RUN_MODE >> 8;
  run_mode.data[4] = RUN_MODE_OPERATION_CONTROL;

  return std::vector<can_frame>{ run_mode, makeFrame(TYPE_ENABLE, HOST_ID, id) };
}

std::vector<can_frame> XiaomiProtocol::makeDisableFrames(int id) const
{
  return std::vector<can_frame>{ makeFrame(TYPE_STOP, HOST_ID, id) };
}

std::vector<can_frame> XiaomiProtocol::makeStatusRequestFrames(int id) const
{
  /* the stop frame with zero data does not clear the fault, and only replies the feedback in the stopped motor */
  return std::vector<can_frame>{ makeFrame(TYPE_STOP, HOST_ID, id) };
}

can_frame XiaomiProtocol::makeMitFrame(int id, const MitMotorModel& model, const MitCommand& cmd) const
{
  const int p_int = floatToUint(cmd.position, -model.p_max, model.p_max, 16);
  const int v_int = floatToUint(cmd.velocity, -model.v_max, model.v_max, 16);
  const int kp_int = floatToUint(cmd.kp, 0.0, model.kp_max, 16);
  const int kd_int = floatToUint(cmd.kd, 0.0, model.kd_max, 16);
  const int t_int = floatToUint(cmd.torque, -model.t_max, model.t_max, 16);

  can_frame frame = makeFrame(TYPE_MOTION_CONTROL, t_int, id);
  frame.data[0] = p_int >> 8;
  frame.data[1] = p_int & 0xFF;
  frame.data[2] = v_int >> 8;
  frame.data[3] = v_int & 0xFF;
  frame.data[4] = kp_int >> 8;
  frame.data[5] = kp_int & 0xFF;
  frame.data[6] = kd_int >> 8;
  frame.data[7] = kd_int & 0xFF;
  return frame;
}

bool XiaomiProtocol::parseId(const can_frame& frame, int& id) const
{
  if (!(frame.can_id & CAN_EFF_FLAG) || frame.can_dlc < 8)
    return false;

  const uint32_t eid = frame.can_id & CAN_EFF_MASK;
  if ((eid >> 24) != TYPE_FEEDBACK)
    return false;

  id = (eid >> 8) & 0xFF;
  return true;
}

bool XiaomiProtocol::parseStatus(const can_frame& frame, const MitMotorModel& model, MitStatus& status) const
{
  const uint32_t eid = frame.can_id & CAN_EFF_MASK;
  const int pos_raw = (frame.data[0] << 8) | frame.data[1];
  const int vel_raw = (frame.data[2] << 8) | frame.data[3];
  const int tau_raw = (frame.data[4] << 8) | frame.data[5];
  const int temp_raw = (frame.data[6] << 8) | frame.data[7];

  status.position = uintToFloat(pos_raw, -model.p_max, model.p_max, 16);
  status.velocity = uintToFloat(vel_raw, -model.v_max, model.v_max, 16);
  status.torque = uintToFloat(tau_raw, -model.t_max, model.t_max, 16);
  status.temperature = temp_raw * 0.1;
  status.error_code = (eid >> 16) & 0x3F;
  status.disabled = ((eid >> 22) & 0x3) != MODE_MOTOR;
  return true;
}

std::string XiaomiProtocol::errorString(int error_code) const
{
  static const char* const faults[] = { "under-voltage",          "over-current",       "over-temperature",
                                        "magnetic encoder fault", "HALL encoder fault", "not calibrated" };
  std::string message;
  for (int i = 0; i < 6; i++)
  {
    if (error_code & (1 << i))
      message += (message.empty() ? "" : ", ") + std::string(faults[i]);
  }
  return message.empty() ? "no error" : message;
}
