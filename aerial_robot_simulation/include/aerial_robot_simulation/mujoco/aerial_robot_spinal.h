// -*- mode: c++ -*-
//
// The simulated flight controller, for the ROS2 MuJoCo hardware component.
//
// Under ROS1 this lived in two places: hardware_interface::MujocoSpinalInterface
// held the attitude estimator, and a separate ros_control controller plugin
// (MujocoAttitudeController) held the flight control core and reached into the
// interface through a raw pointer.
//
// That split cannot be reproduced under ros2_control, which passes only named
// doubles between a hardware component and its controllers. The ROS1 controller
// receives a StateEstimate* and writes back into the hardware
// (spinal_interface_->onGround(...)); neither has a named-double equivalent. So
// estimator and controller are folded together here and owned by the hardware
// component. See docs/ros2_migration.md.

#pragma once

#include <aerial_robot_ros_compat/ros_compat.h>

#include <flight_control/flight_control.h>
#include <state_estimate/state_estimate.h>

#include <vector>

namespace aerial_robot_simulation
{

/**
 * Owns the spinal attitude estimator and flight control core, and drives them
 * from MuJoCo's sensor and actuator data.
 *
 * This is the firmware's own code, compiled with -DSIMULATION, so the control
 * law in simulation is the same one that runs on the board.
 */
class AerialRobotSpinal
{
public:
  bool init(ros_compat::NodeHandle nh, int motor_num);

  /** Feed one IMU sample and advance the attitude estimate. */
  void setImuValue(double acc_x, double acc_y, double acc_z, double gyro_x, double gyro_y, double gyro_z);
  void setMagValue(double mag_x, double mag_y, double mag_z);
  void stateEstimate();

  /** Overwrite the estimate with simulation truth. */
  void setGroundTruthStates(double q_x, double q_y, double q_z, double q_w, double w_x, double w_y, double w_z);

  /**
   * Freeze the attitude estimator while the robot is on the ground.
   *
   * The ROS1 controller called this on the hardware every cycle. Here both sides
   * are in one object, so update() applies it directly.
   */
  void onGround(bool flag)
  {
    on_ground_ = flag;
  }

  /** Run one flight control step and latch the resulting rotor forces. */
  void update();

  int getMotorNum() const
  {
    return motor_num_;
  }
  double getForce(int index) const
  {
    return index >= 0 && index < static_cast<int>(force_.size()) ? force_[index] : 0.0;
  }

private:
  bool on_ground_ = true;
  int motor_num_ = 0;
  std::vector<double> force_;

  StateEstimate spinal_state_estimator_;
  FlightControl controller_core_;
  ros_compat::NodeHandle nh_;
};

}  // namespace aerial_robot_simulation
