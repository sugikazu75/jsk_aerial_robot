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

#include <aerial_robot_model/servo_backend/mit_can/cubemars_v3_protocol.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace
{
const uint32_t CAN_PACKET_SET_MIT = 8;
const uint32_t CAN_PACKET_DISABLE = 15;
const uint32_t CAN_PACKET_STATUS = 0x29;
const uint8_t DISABLE_ACK = 0x77;  // error code in the status frame after disable
}  // namespace

can_frame CubemarsV3Protocol::makeFrame(uint32_t packet_id, int id, const uint8_t* data, uint8_t len)
{
  can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = CAN_EFF_FLAG | (packet_id << 8) | (static_cast<uint32_t>(id) & 0xFF);
  frame.can_dlc = std::min<uint8_t>(len, CAN_MAX_DLEN);
  for (int i = 0; i < frame.can_dlc; i++)
    frame.data[i] = data[i];
  return frame;
}

std::vector<can_frame> CubemarsV3Protocol::makeEnableFrames(int id) const
{
  return std::vector<can_frame>();  // enabled by the MIT command
}

std::vector<can_frame> CubemarsV3Protocol::makeDisableFrames(int id) const
{
  return std::vector<can_frame>{ makeFrame(CAN_PACKET_DISABLE, id, nullptr, 0) };
}

can_frame CubemarsV3Protocol::makeMitFrame(int id, const MitMotorModel& model, const MitCommand& cmd) const
{
  const int p_int = floatToUint(cmd.position, -model.p_max, model.p_max, 16);
  const int v_int = floatToUint(cmd.velocity, -model.v_max, model.v_max, 12);
  const int kp_int = floatToUint(cmd.kp, 0.0, model.kp_max, 12);
  const int kd_int = floatToUint(cmd.kd, 0.0, model.kd_max, 12);
  const int t_int = floatToUint(cmd.torque, -model.t_max, model.t_max, 12);

  uint8_t data[8];
  data[0] = kp_int >> 4;                            // kp high 8 bits
  data[1] = ((kp_int & 0xF) << 4) | (kd_int >> 8);  // kp low 4 bits, kd high 4 bits
  data[2] = kd_int & 0xFF;                          // kd low 8 bits
  data[3] = p_int >> 8;                             // position high 8 bits
  data[4] = p_int & 0xFF;                           // position low 8 bits
  data[5] = v_int >> 4;                             // speed high 8 bits
  data[6] = ((v_int & 0xF) << 4) | (t_int >> 8);    // speed low 4 bits, torque high 4 bits
  data[7] = t_int & 0xFF;                           // torque low 8 bits

  return makeFrame(CAN_PACKET_SET_MIT, id, data, 8);
}

bool CubemarsV3Protocol::parseId(const can_frame& frame, int& id) const
{
  if (!(frame.can_id & CAN_EFF_FLAG) || frame.can_dlc < 8)
    return false;

  const uint32_t eid = frame.can_id & CAN_EFF_MASK;
  if ((eid >> 8) != CAN_PACKET_STATUS)
    return false;

  id = eid & 0xFF;
  return true;
}

bool CubemarsV3Protocol::parseStatus(const can_frame& frame, const MitMotorModel& model, MitStatus& status) const
{
  const int16_t pos_raw = static_cast<int16_t>((frame.data[0] << 8) | frame.data[1]);
  const int16_t spd_raw = static_cast<int16_t>((frame.data[2] << 8) | frame.data[3]);
  const int16_t cur_raw = static_cast<int16_t>((frame.data[4] << 8) | frame.data[5]);
  const int8_t temp_raw = static_cast<int8_t>(frame.data[6]);
  const uint8_t error_code = frame.data[7];

  status.position = pos_raw * 0.1 * M_PI / 180.0;  // [rad]
  // output shaft speed = ERPM / pole pairs / gear ratio
  status.velocity = spd_raw * 10.0 / model.pole_pairs / model.gear_ratio * 2.0 * M_PI / 60.0;  // [rad/s]
  status.torque = cur_raw * 0.01 * model.torque_constant;                                      // [Nm]
  status.temperature = temp_raw;
  status.disabled = (error_code == DISABLE_ACK);
  status.error_code = status.disabled ? 0 : error_code;
  return true;
}

std::string CubemarsV3Protocol::errorString(int error_code) const
{
  switch (error_code)
  {
    case 0:
      return "no error";
    case 1:
      return "motor over-temperature";
    case 2:
      return "over-current";
    case 3:
      return "over-voltage";
    case 4:
      return "under-voltage";
    case 5:
      return "encoder fault";
    case 6:
      return "MOSFET over-temperature";
    case 7:
      return "motor stall";
    default:
      return "unknown error";
  }
}
