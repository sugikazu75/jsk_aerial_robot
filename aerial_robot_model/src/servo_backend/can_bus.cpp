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

#include <aerial_robot_model/servo_backend/can_bus.h>

#include <ros/ros.h>

#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>

CanBus::~CanBus()
{
  close();
}

bool CanBus::open(const std::string& interface_name, const RxCallback& callback)
{
  interface_name_ = interface_name;
  callback_ = callback;

  fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0)
  {
    ROS_ERROR("[can bus]: failed to create socket for %s: %s", interface_name.c_str(), strerror(errno));
    return false;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0)
  {
    ROS_ERROR("[can bus]: can not find CAN interface %s: %s", interface_name.c_str(), strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
  {
    ROS_ERROR("[can bus]: failed to bind socket to %s: %s", interface_name.c_str(), strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  running_ = true;
  rx_thread_ = std::thread(&CanBus::rxLoop, this);

  ROS_INFO("[can bus]: open %s", interface_name.c_str());
  return true;
}

void CanBus::close()
{
  running_ = false;
  if (rx_thread_.joinable())
    rx_thread_.join();
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

bool CanBus::send(const can_frame& frame)
{
  if (fd_ < 0)
    return false;

  std::lock_guard<std::mutex> lock(tx_mutex_);
  // do not block the caller (e.g., ros callback) even if the tx queue is full
  ssize_t n = ::send(fd_, &frame, sizeof(frame), MSG_DONTWAIT);
  if (n != sizeof(frame))
  {
    ROS_WARN_THROTTLE(1.0, "[can bus]: failed to send frame to %s: %s", interface_name_.c_str(), strerror(errno));
    return false;
  }
  return true;
}

void CanBus::rxLoop()
{
  struct pollfd pfd;
  pfd.fd = fd_;
  pfd.events = POLLIN;

  while (running_)
  {
    // check the stop flag periodically
    if (poll(&pfd, 1, 100) <= 0)
      continue;

    can_frame frame;
    ssize_t n = ::read(fd_, &frame, sizeof(frame));
    if (n != sizeof(frame))
    {
      ROS_WARN_THROTTLE(1.0, "[can bus]: failed to read frame from %s: %s", interface_name_.c_str(), strerror(errno));
      // e.g., the interface is down: avoid busy loop, and wait for the recovery
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    if (frame.can_id & CAN_ERR_FLAG)
      continue;

    callback_(frame);
  }
}
