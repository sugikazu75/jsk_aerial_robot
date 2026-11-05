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

#include <aerial_robot_model/servo_backend/mit_can/mit_protocol.h>
#include <aerial_robot_model/servo_backend/mit_can/cubemars_v3_protocol.h>

#include <algorithm>

int MitProtocol::floatToUint(double x, double x_min, double x_max, int bits)
{
  const double span = x_max - x_min;
  x = std::min(std::max(x, x_min), x_max);
  return static_cast<int>((x - x_min) * static_cast<double>((1 << bits) - 1) / span);
}

double MitProtocol::uintToFloat(int x, double x_min, double x_max, int bits)
{
  const double span = x_max - x_min;
  return static_cast<double>(x) * span / static_cast<double>((1 << bits) - 1) + x_min;
}

const std::map<std::string, MitMotorModel>& mitMotorModels()
{
  static const std::map<std::string, MitMotorModel> models = {
    /* v3: AK Series Motor Module V3 Driver Software User Manual (V1.0.1), force control mode parameter range */
    // protocol, p_max, v_max, t_max, kp_max, kd_max, torque_constant, pole_pairs, gear_ratio
    { "AKE60_8", { "cubemars_v3", 12.56, 40.0, 15.0, 500.0, 5.0, 0.7382, 14, 8 } },
    { "AK40_10_V3", { "cubemars_v3", 12.56, 45.0, 5.0, 500.0, 5.0, 0.4894, 14, 10 } },
  };
  return models;
}

MitProtocolPtr createMitProtocol(const std::string& protocol)
{
  if (protocol == "cubemars_v3")
    return std::make_shared<CubemarsV3Protocol>();
  return MitProtocolPtr();
}
