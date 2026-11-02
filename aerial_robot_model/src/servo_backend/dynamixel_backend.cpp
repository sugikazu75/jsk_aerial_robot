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

#include <aerial_robot_model/servo_backend/dynamixel_backend.h>
#include <aerial_robot_model/servo_bridge.h>

DynamixelBackend::DynamixelBackend(const ros::Publisher& pos_pub, const ros::Publisher& torque_pub,
                                   const ros::Publisher& enable_pub)
  : ServoBackend(ServoType::DYNAMIXEL), pos_pub_(pos_pub), torque_pub_(torque_pub), enable_pub_(enable_pub)
{
}

void DynamixelBackend::addCommand(const SingleServoHandle& servo, const ServoCommand& cmd)
{
  if (cmd.has_position)
  {
    pos_msg_.index.push_back(servo.getId());
    pos_msg_.angles.push_back(servo.getTargetAngleVal(ValueType::BIT));
    pos_pending_ = true;
  }

  if (cmd.has_effort)
  {
    torque_msg_.index.push_back(servo.getId());
    torque_msg_.angles.push_back(servo.getTargetTorqueVal(ValueType::BIT));
  }
}

void DynamixelBackend::addTorqueEnable(const SingleServoHandle& servo, bool enable)
{
  enable_msg_.index.push_back(servo.getId());
  enable_msg_.torque_enable.push_back(enable);
}

void DynamixelBackend::flush()
{
  if (pos_pending_)
    pos_pub_.publish(pos_msg_);
  if (torque_msg_.index.size() > 0)
    torque_pub_.publish(torque_msg_);
  if (enable_msg_.index.size() > 0)
    enable_pub_.publish(enable_msg_);
  clear();
}

void DynamixelBackend::clear()
{
  pos_msg_ = spinal::ServoControlCmd();
  torque_msg_ = spinal::ServoControlCmd();
  enable_msg_ = spinal::ServoTorqueCmd();
  pos_pending_ = false;
}
