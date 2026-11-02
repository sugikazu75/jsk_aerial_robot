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

#ifndef SERVO_BACKEND_SERVO_BACKEND_H
#define SERVO_BACKEND_SERVO_BACKEND_H

#include <memory>
#include <string>

class SingleServoHandle;

namespace ServoType
{
const std::string DYNAMIXEL = "dynamixel";
};

/* command for single servo in the joint space (radian, rad/s, Nm) */
struct ServoCommand
{
  bool has_position = false;
  bool has_velocity = false;
  bool has_effort = false;
  double position = 0;
  double velocity = 0;
  double effort = 0;
};

/*
  Interface to send the servo commands to the real machine.
  The commands are buffered by add*() and sent at once by flush().
  The target values in the servo handle are already updated (and clamped) before add*() is called.
  Note: all methods are called under the lock of ServoBridge.
*/
class ServoBackend
{
public:
  explicit ServoBackend(const std::string& type) : type_(type)
  {
  }
  virtual ~ServoBackend() = default;

  const std::string& getType() const
  {
    return type_;
  }

  virtual void addCommand(const SingleServoHandle& servo, const ServoCommand& cmd) = 0;
  virtual void addTorqueEnable(const SingleServoHandle& servo, bool enable) = 0;
  virtual void flush() = 0;
  virtual void clear() = 0;

protected:
  std::string type_;
};

using ServoBackendPtr = std::shared_ptr<ServoBackend>;

#endif
