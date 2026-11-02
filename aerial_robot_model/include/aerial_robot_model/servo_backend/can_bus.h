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

#ifndef SERVO_BACKEND_CAN_BUS_H
#define SERVO_BACKEND_CAN_BUS_H

#include <linux/can.h>

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

/*
  Simple wrapper of SocketCAN (raw socket) for one CAN interface (e.g., can0).
  - send(): write one frame to the bus without blocking
  - received frames are passed to the callback in the dedicated receive thread
  Note: the CAN interface should be brought up beforehand (e.g., ip link set can0 up type can bitrate 1000000)
*/
class CanBus
{
public:
  using RxCallback = std::function<void(const can_frame&)>;

  CanBus() = default;
  ~CanBus();
  CanBus(const CanBus&) = delete;
  CanBus& operator=(const CanBus&) = delete;

  bool open(const std::string& interface_name, const RxCallback& callback);
  void close();
  bool send(const can_frame& frame);

  const std::string& getName() const
  {
    return interface_name_;
  }

private:
  void rxLoop();

  std::string interface_name_;
  int fd_ = -1;
  std::atomic<bool> running_{ false };
  std::thread rx_thread_;
  std::mutex tx_mutex_;
  RxCallback callback_;
};

#endif
