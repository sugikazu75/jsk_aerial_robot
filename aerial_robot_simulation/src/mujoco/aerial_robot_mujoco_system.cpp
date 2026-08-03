#include <aerial_robot_simulation/mujoco/aerial_robot_mujoco_system.h>

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace aerial_robot_simulation
{

namespace
{
constexpr char kThrustInterface[] = "thrust";

bool isSupportedStateInterface(const std::string& interface_name)
{
  return interface_name == hardware_interface::HW_IF_EFFORT || interface_name == kThrustInterface;
}

bool isSupportedCommandInterface(const std::string& interface_name)
{
  return interface_name == hardware_interface::HW_IF_EFFORT || interface_name == kThrustInterface;
}

double clampToRange(const double value, const double lower, const double upper)
{
  return std::min(std::max(value, lower), upper);
}
}  // namespace

class AerialRobotMujocoSystem::Private
{
public:
  struct RotorData
  {
    std::string name;
    std::string actuator_name;
    int actuator_id = -1;
    double lower_command = -std::numeric_limits<double>::infinity();
    double upper_command = std::numeric_limits<double>::infinity();
    double command = 0.0;
    double state = 0.0;
  };

  std::vector<RotorData> rotors;
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  mjData* data = nullptr;
  const mjModel* model = nullptr;
  unsigned int* update_rate = nullptr;
  rclcpp::Time last_update_sim_time = rclcpp::Time(0L, RCL_STEADY_TIME);
};

bool AerialRobotMujocoSystem::initSim(rclcpp_lifecycle::LifecycleNode::SharedPtr& model_nh,
                                      const hardware_interface::HardwareInfo& hardware_info, const mjModel* m,
                                      mjData* d, unsigned int& update_rate)
{
  nh_ = model_nh;
  data_ = std::make_unique<Private>();
  data_->model = m;
  data_->data = d;
  data_->update_rate = &update_rate;
  data_->rotors.resize(hardware_info.joints.size());

  for (std::size_t i = 0; i < hardware_info.joints.size(); ++i)
  {
    const auto& joint_info = hardware_info.joints[i];
    auto& rotor = data_->rotors[i];
    rotor.name = joint_info.name;

    const auto actuator_name_it = joint_info.parameters.find("actuator_name");
    rotor.actuator_name =
        actuator_name_it == joint_info.parameters.end() ? joint_info.name : actuator_name_it->second;
    rotor.actuator_id = mj_name2id(m, mjOBJ_ACTUATOR, rotor.actuator_name.c_str());
    if (rotor.actuator_id < 0)
    {
      RCLCPP_ERROR(nh_->get_logger(), "MuJoCo actuator `%s` for `%s` was not found.",
                   rotor.actuator_name.c_str(), rotor.name.c_str());
      return false;
    }

    if (m->actuator_ctrllimited[rotor.actuator_id])
    {
      rotor.lower_command = m->actuator_ctrlrange[2 * rotor.actuator_id];
      rotor.upper_command = m->actuator_ctrlrange[2 * rotor.actuator_id + 1];
    }

    for (const auto& state_interface : joint_info.state_interfaces)
    {
      if (!isSupportedStateInterface(state_interface.name))
      {
        RCLCPP_ERROR(nh_->get_logger(),
                     "Unsupported state interface `%s` on `%s`. Use `effort` or `thrust` for rotor actuators.",
                     state_interface.name.c_str(), rotor.name.c_str());
        return false;
      }

      data_->state_interfaces.emplace_back(rotor.name, state_interface.name, &rotor.state);
    }

    bool has_command_interface = false;
    for (const auto& command_interface : joint_info.command_interfaces)
    {
      if (!isSupportedCommandInterface(command_interface.name))
      {
        RCLCPP_ERROR(
            nh_->get_logger(),
            "Unsupported command interface `%s` on `%s`. Use `effort` or `thrust` for rotor actuators.",
            command_interface.name.c_str(), rotor.name.c_str());
        return false;
      }

      if (has_command_interface)
      {
        RCLCPP_ERROR(nh_->get_logger(),
                     "Rotor actuator `%s` exposes more than one command interface. Keep exactly one of `effort` or "
                     "`thrust`.",
                     rotor.name.c_str());
        return false;
      }

      data_->command_interfaces.emplace_back(rotor.name, command_interface.name, &rotor.command);
      has_command_interface = true;
    }

    RCLCPP_INFO(nh_->get_logger(), "Mapped rotor interface `%s` to MuJoCo actuator `%s`.",
                rotor.name.c_str(), rotor.actuator_name.c_str());
  }

  return true;
}

CallbackReturn AerialRobotMujocoSystem::on_init(const hardware_interface::HardwareInfo& system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn AerialRobotMujocoSystem::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AerialRobotMujocoSystem::export_state_interfaces()
{
  return std::move(data_->state_interfaces);
}

std::vector<hardware_interface::CommandInterface> AerialRobotMujocoSystem::export_command_interfaces()
{
  return std::move(data_->command_interfaces);
}

CallbackReturn AerialRobotMujocoSystem::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn AerialRobotMujocoSystem::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (data_ != nullptr && data_->data != nullptr)
  {
    for (auto& rotor : data_->rotors)
    {
      rotor.command = 0.0;
      if (rotor.actuator_id >= 0)
      {
        data_->data->ctrl[rotor.actuator_id] = 0.0;
      }
    }
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type AerialRobotMujocoSystem::prepare_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/, const std::vector<std::string>& /*stop_interfaces*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AerialRobotMujocoSystem::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/, const std::vector<std::string>& /*stop_interfaces*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AerialRobotMujocoSystem::read(const rclcpp::Time& time,
                                                              const rclcpp::Duration& /*period*/)
{
  if (data_ == nullptr || data_->data == nullptr)
  {
    return hardware_interface::return_type::ERROR;
  }

  const bool should_update =
      data_->update_rate == nullptr || *data_->update_rate == 0 ||
      epsilonComp(time, data_->last_update_sim_time, static_cast<double>(*data_->update_rate));
  if (!should_update)
  {
    return hardware_interface::return_type::OK;
  }

  for (auto& rotor : data_->rotors)
  {
    rotor.state = rotor.actuator_id >= 0 ? data_->data->actuator_force[rotor.actuator_id] : 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AerialRobotMujocoSystem::write(const rclcpp::Time& time,
                                                               const rclcpp::Duration& /*period*/)
{
  if (data_ == nullptr || data_->data == nullptr)
  {
    return hardware_interface::return_type::ERROR;
  }

  const bool should_update =
      data_->update_rate == nullptr || *data_->update_rate == 0 ||
      epsilonComp(time, data_->last_update_sim_time, static_cast<double>(*data_->update_rate));
  if (!should_update)
  {
    return hardware_interface::return_type::OK;
  }

  for (auto& rotor : data_->rotors)
  {
    if (rotor.actuator_id < 0)
    {
      continue;
    }

    data_->data->ctrl[rotor.actuator_id] =
        clampToRange(rotor.command, rotor.lower_command, rotor.upper_command);
  }
  data_->last_update_sim_time = time;

  return hardware_interface::return_type::OK;
}

}  // namespace aerial_robot_simulation

PLUGINLIB_EXPORT_CLASS(aerial_robot_simulation::AerialRobotMujocoSystem,
                       mujoco_ros::control::MujocoRosSystemInterface)
