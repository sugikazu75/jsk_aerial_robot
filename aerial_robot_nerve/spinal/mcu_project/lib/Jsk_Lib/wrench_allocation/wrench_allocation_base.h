/*
******************************************************************************
* File Name          : wrench_allocation_base.h
* Description        : base class for wrench allocation
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __WRENCH_ALLOCATION_BASE_H
#define __WRENCH_ALLOCATION_BASE_H

#ifndef SIMULATION
#include "config.h"
#include <ros.h>
#else
#include <ros/ros.h>
#endif

#include <math/AP_Math.h>
#include <vector>

class WrenchAllocationBase
{
public:
  WrenchAllocationBase();
  ~WrenchAllocationBase(){}

  void init(ros::NodeHandle* nh);
  void setMotorNumber(int motor_num);

private:
#ifdef SIMULATION
  ros::Subscriber torque_allocation_matrix_inv_sub_;
#else
  ros::Subscriber<spinal::TorqueAllocationMatrixInv, AttitudeController> torque_allocation_matrix_inv_sub_;
#endif
  int motor_num_;
}
