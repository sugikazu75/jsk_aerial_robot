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

#ifndef SERVO_BACKEND_CUBEMARS_CUBEMARS_PROTOCOL_H
#define SERVO_BACKEND_CUBEMARS_CUBEMARS_PROTOCOL_H

#include <linux/can.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

/* parameters of CubeMars AK series motor in force control (MIT) mode */
struct CubemarsModel
{
  std::string protocol;    // communication protocol (e.g., "v3")
  double p_max;            // [rad]
  double v_max;            // [rad/s]
  double t_max;            // [Nm]
  double kp_max;           // [Nm/rad]
  double kd_max;           // [Nm/(rad/s)]
  double torque_constant;  // [Nm/A] at output shaft
  double pole_pairs;
  double gear_ratio;
};

/* MIT command in the motor side (output shaft) */
struct CubemarsMitCommand
{
  double position = 0;  // [rad]
  double velocity = 0;  // [rad/s]
  double kp = 0;        // [Nm/rad]
  double kd = 0;        // [Nm/(rad/s)]
  double torque = 0;    // [Nm], feed-forward torque
};

/* motor status in the motor side (output shaft) */
struct CubemarsStatus
{
  double position = 0;     // [rad]
  double velocity = 0;     // [rad/s]
  double torque = 0;       // [Nm]
  double temperature = 0;  // [deg C]
  int error_code = 0;      // 0: no error
  bool disabled = false;   // response to the disable command
};

/*
  Communication protocol of CubeMars motors (difference of firmware version, e.g., v2 and v3).
  The protocol only converts between CAN frames and physical values in the motor side.
*/
class CubemarsProtocol
{
public:
  virtual ~CubemarsProtocol() = default;

  /* frames to enable / disable the motor (e.g., enter / exit motor control mode) */
  virtual std::vector<can_frame> makeEnableFrames(int id) const = 0;
  virtual std::vector<can_frame> makeDisableFrames(int id) const = 0;
  virtual can_frame makeMitFrame(int id, const CubemarsModel& model, const CubemarsMitCommand& cmd) const = 0;

  /* return false if the frame is not the status frame of this protocol */
  virtual bool parseId(const can_frame& frame, int& id) const = 0;
  virtual bool parseStatus(const can_frame& frame, const CubemarsModel& model, CubemarsStatus& status) const = 0;

  virtual std::string errorString(int error_code) const = 0;

protected:
  static int floatToUint(double x, double x_min, double x_max, int bits);
  static double uintToFloat(int x, double x_min, double x_max, int bits);
};

using CubemarsProtocolPtr = std::shared_ptr<CubemarsProtocol>;

/* supported motor models (name -> parameters) */
const std::map<std::string, CubemarsModel>& cubemarsModels();

/* create protocol by name (e.g., "v3"), return nullptr if not supported */
CubemarsProtocolPtr createCubemarsProtocol(const std::string& protocol);

#endif
