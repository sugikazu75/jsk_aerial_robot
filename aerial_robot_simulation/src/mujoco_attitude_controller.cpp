#include <aerial_robot_simulation/mujoco_attitude_controller.h>

MujocoAttitudeController::MujocoAttitudeController():
  controller_core_(new FlightControl())
{
}

bool MujocoAttitudeController::init(MujocoSpinalInterface* spinal_interface)
{
  spinal_interface_ = spinal_interface;
}

void MujocoAttitudeController::starting(const ros::Time& time)
{
}

void MujocoAttitudeController::update(const ros::Time& time, const ros::Duration& period)
{
  /* set ground truth value for control */
  auto true_cog_rpy = spinal_interface_->getTrueCogRPY();
  controller_core_->getAttController().setTrueRPY(true_cog_rpy.x(), true_cog_rpy.y(), true_cog_rpy.z());
  auto true_cog_angular = spinal_interface_->getTrueCogAngular();
  controller_core_->getAttController().setTrueAngular(true_cog_angular.x(), true_cog_angular.y(), true_cog_angular.z());

  /* freeze the attitude estimator while touching the ground, since the bad contact simulation performance in gazebo */
  spinal_interface_->onGround(!controller_core_->getAttController().getIntegrateFlag());

  /* update the controller */
  controller_core_->update();
}