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

#ifndef SERVO_BACKEND_DYNAMIXEL_BACKEND_H
#define SERVO_BACKEND_DYNAMIXEL_BACKEND_H

#include <aerial_robot_model/servo_backend/servo_backend.h>

#include <ros/ros.h>
#include <spinal/ServoControlCmd.h>
#include <spinal/ServoTorqueCmd.h>

/* Dynamixel via spinal (rosserial): convert to the servo unit (BIT) and publish spinal messages */
class DynamixelBackend : public ServoBackend
{
public:
  DynamixelBackend(const ros::Publisher& pos_pub, const ros::Publisher& torque_pub, const ros::Publisher& enable_pub);

  void addCommand(const SingleServoHandle& servo, const ServoCommand& cmd) override;
  void addTorqueEnable(const SingleServoHandle& servo, bool enable) override;
  void flush() override;
  void clear() override;

private:
  ros::Publisher pos_pub_, torque_pub_, enable_pub_;
  spinal::ServoControlCmd pos_msg_, torque_msg_;
  spinal::ServoTorqueCmd enable_msg_;
  bool pos_pending_ = false;
};

#endif
