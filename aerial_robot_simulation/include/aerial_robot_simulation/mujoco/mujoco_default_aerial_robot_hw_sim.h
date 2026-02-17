#pragma once

#include <mujoco_ros_control/default_robot_hw_sim.h>
#include <aerial_robot_simulation/rotor_interface.h>

namespace mujoco_ros_control
{
class DefaultAerialRobotHWSim : public mujoco_ros::control::DefaultRobotHWSim
{
public:
  DefaultAerialRobotHWSim(){};
  ~DefaultAerialRobotHWSim()
  {
  }

  bool initSim(const mjModel* m_ptr, mjData* d_ptr, mujoco_ros::MujocoEnv* mujoco_env_ptr,
               const std::string& robot_namespace, ros::NodeHandle model_nh, const urdf::Model* const urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions) override;

  void readSim(ros::Time time, ros::Duration period) override;

  void writeSim(ros::Time time, ros::Duration period) override;

private:
  hardware_interface::RotorInterface rotor_interface_;

  std::vector<std::string> rotor_names_;
  std::vector<double> rotor_cmd_;
  std::vector<double> rotor_pos_;
  std::vector<double> rotor_vel_;
  std::vector<double> rotor_eff_;
};
}  // namespace mujoco_ros_control
