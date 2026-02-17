#include <aerial_robot_simulation/mujoco/mujoco_default_aerial_robot_hw_sim.h>

namespace mujoco_ros_control
{

bool DefaultAerialRobotHWSim::initSim(const mjModel* m_ptr, mjData* d_ptr, mujoco_ros::MujocoEnv* mujoco_env_ptr,
                                      const std::string& robot_namespace, ros::NodeHandle model_nh,
                                      const urdf::Model* const urdf_model,
                                      std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  std::vector<transmission_interface::TransmissionInfo> standard_transmissions;
  std::vector<transmission_interface::TransmissionInfo> rotor_transmissions;

  /* separate rotor transmissions and standard transmissions */
  for (const auto& trans : transmissions)
  {
    bool is_rotor = false;
    for (const auto& hw_iface : trans.actuators_[0].hardware_interfaces_)
    {
      if (hw_iface == "RotorInterface" || hw_iface == "hardware_interface/RotorInterface")
      {
        is_rotor = true;
        break;
      }
    }
    if (is_rotor)
    {
      rotor_transmissions.push_back(trans);
    }
    else
      standard_transmissions.push_back(trans);
  }

  /* Initialize standard joint handlers */
  DefaultRobotHWSim::initSim(m_ptr, d_ptr, mujoco_env_ptr, robot_namespace, model_nh, urdf_model,
                             standard_transmissions);

  /* Initialize rotor handlers */
  size_t num_rotors = rotor_transmissions.size();
  rotor_names_.resize(num_rotors);
  rotor_cmd_.resize(num_rotors);
  rotor_pos_.resize(num_rotors);
  rotor_vel_.resize(num_rotors);
  rotor_eff_.resize(num_rotors);
  for (size_t i = 0; i < num_rotors; i++)
  {
    const auto& transmission = rotor_transmissions[i];

    std::string joint_name = transmission.joints_[0].name_;

    rotor_names_.at(i) = joint_name;
    rotor_cmd_.at(i) = 0.0;
    rotor_pos_.at(i) = 0.0;
    rotor_vel_.at(i) = 0.0;
    rotor_eff_.at(i) = 0.0;

    /* create handle for each rotor */
    hardware_interface::JointStateHandle state_handle(rotor_names_.at(i), &rotor_pos_.at(i), &rotor_vel_.at(i),
                                                      &rotor_eff_.at(i));
    hardware_interface::JointHandle rotor_handle(state_handle, &rotor_cmd_.at(i));

    rotor_interface_.registerHandle(rotor_handle);
  }

  registerInterface(&rotor_interface_);

  return true;
}

void DefaultAerialRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  DefaultRobotHWSim::readSim(time, period);
}

void DefaultAerialRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
  DefaultRobotHWSim::writeSim(time, period);

  /* rotor part */
  for (size_t i = 0; i < rotor_names_.size(); i++)
  {
    int rotor_id = mj_name2id(m_ptr_, mjtObj_::mjOBJ_ACTUATOR, rotor_names_[i].c_str());
    d_ptr_->ctrl[rotor_id] = rotor_cmd_.at(i);
  }
}

}  // namespace mujoco_ros_control

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::DefaultAerialRobotHWSim, mujoco_ros::control::RobotHWSim)
