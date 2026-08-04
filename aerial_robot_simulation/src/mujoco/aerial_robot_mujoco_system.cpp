#include <aerial_robot_simulation/mujoco/aerial_robot_mujoco_system.h>

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <aerial_robot_ros_compat/tf_compat.h>
#include <aerial_robot_simulation/mujoco/aerial_robot_spinal.h>
#include <aerial_robot_simulation/noise_model.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

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

  /* The simulated flight controller: attitude estimator plus control core. */
  AerialRobotSpinal spinal;
  std::shared_ptr<rclcpp::Node> spinal_node;
  ros_compat::NodeHandle nh;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pub;

  double mocap_pub_rate = 0.01;
  double mocap_pos_noise = 0.001;
  double mocap_rot_noise = 0.001;
  rclcpp::Time last_mocap_time = rclcpp::Time(0L, RCL_ROS_TIME);

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

  /* The simulated flight controller. Under ROS1 this was split between the
     hardware interface and a ros_control controller plugin; ros2_control cannot
     carry an object across that boundary, so it lives here. */
  /* mujoco_ros_control hands us a LifecycleNode; the compat NodeHandle is built
     on rclcpp::Node, and the two are unrelated types in rclcpp. Rather than
     widen the compat layer for this one call site, give the flight controller
     its own node - carrying the lifecycle node's name, namespace and, crucially,
     its parameter overrides, so the spinal parameters resolve identically. */
  rclcpp::NodeOptions spinal_options;
  spinal_options.parameter_overrides(model_nh->get_node_options().parameter_overrides());
  spinal_options.automatically_declare_parameters_from_overrides(true);
  spinal_options.allow_undeclared_parameters(true);
  data_->spinal_node = std::make_shared<rclcpp::Node>(std::string(model_nh->get_name()) + "_spinal",
                                                      model_nh->get_namespace(), spinal_options);
  data_->nh = ros_compat::NodeHandle(data_->spinal_node);
  ros_compat::setGlobalNode(data_->nh);
  data_->spinal.init(data_->nh, static_cast<int>(data_->rotors.size()));

  ros_compat::NodeHandle simulation_nh(data_->nh, "simulation");
  ros_compat::getParam<double>(simulation_nh, "mocap_pub_rate", data_->mocap_pub_rate, 0.01);   // [sec]
  ros_compat::getParam<double>(simulation_nh, "mocap_pos_noise", data_->mocap_pos_noise, 0.001);  // m
  ros_compat::getParam<double>(simulation_nh, "mocap_rot_noise", data_->mocap_rot_noise, 0.001);  // rad

  /* The estimator subscribes to these: with sim_estimate_mode = GROUND_TRUTH,
     ground_truth is the only source of pose it has. */
  data_->ground_truth_pub = model_nh->create_publisher<nav_msgs::msg::Odometry>("ground_truth", 1);
  data_->mocap_pub = model_nh->create_publisher<geometry_msgs::msg::PoseStamped>("mocap/pose", 1);

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

  /* Pose of the `fc` site: simulation truth for the flight controller. */
  const int fc_id = mj_name2id(data_->model, mjOBJ_SITE, "fc");
  if (fc_id < 0)
  {
    RCLCPP_ERROR_ONCE(nh_->get_logger(),
                      "No MuJoCo site named `fc`. The flight controller has no pose to work from.");
    return hardware_interface::return_type::ERROR;
  }

  const mjtNum* site_xpos = data_->data->site_xpos;
  const mjtNum* site_xmat = data_->data->site_xmat;
  const tf2::Matrix3x3 fc_rot_mat(site_xmat[9 * fc_id + 0], site_xmat[9 * fc_id + 1], site_xmat[9 * fc_id + 2],
                                  site_xmat[9 * fc_id + 3], site_xmat[9 * fc_id + 4], site_xmat[9 * fc_id + 5],
                                  site_xmat[9 * fc_id + 6], site_xmat[9 * fc_id + 7], site_xmat[9 * fc_id + 8]);
  tf2::Quaternion fc_quat;
  fc_rot_mat.getRotation(fc_quat);

  /* IMU from the MuJoCo sensors, by name, as the ROS1 hardware sim did. */
  tf2::Vector3 acc(0, 0, 0), gyro(0, 0, 0), mag(0, 0, 0);
  for (int i = 0; i < data_->model->nsensor; ++i)
  {
    const char* raw_name = mj_id2name(data_->model, mjOBJ_SENSOR, i);
    if (raw_name == nullptr) continue;
    const std::string sensor_name(raw_name);

    tf2::Vector3* target = nullptr;
    if (sensor_name == "acc") target = &acc;
    else if (sensor_name == "gyro") target = &gyro;
    else if (sensor_name == "mag") target = &mag;
    if (target == nullptr) continue;

    for (int j = 0; j < data_->model->sensor_dim[i] && j < 3; ++j)
    {
      (*target)[j] = data_->data->sensordata[data_->model->sensor_adr[i] + j];
    }
  }

  data_->spinal.setImuValue(acc.x(), acc.y(), acc.z(), gyro.x(), gyro.y(), gyro.z());
  data_->spinal.setMagValue(mag.x(), mag.y(), mag.z());
  data_->spinal.stateEstimate();

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = time;
  odom_msg.pose.pose.position.x = site_xpos[3 * fc_id + 0];
  odom_msg.pose.pose.position.y = site_xpos[3 * fc_id + 1];
  odom_msg.pose.pose.position.z = site_xpos[3 * fc_id + 2];
  odom_msg.pose.pose.orientation.x = fc_quat.x();
  odom_msg.pose.pose.orientation.y = fc_quat.y();
  odom_msg.pose.pose.orientation.z = fc_quat.z();
  odom_msg.pose.pose.orientation.w = fc_quat.w();
  data_->ground_truth_pub->publish(odom_msg);

  data_->spinal.setGroundTruthStates(fc_quat.x(), fc_quat.y(), fc_quat.z(), fc_quat.w(), gyro.x(), gyro.y(), gyro.z());

  if ((ros_compat::Time(time) - ros_compat::Time(data_->last_mocap_time)).toSec() >= data_->mocap_pub_rate)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = time;
    pose_msg.pose.position.x = site_xpos[3 * fc_id + 0] + gazebo::gaussianKernel(data_->mocap_pos_noise);
    pose_msg.pose.position.y = site_xpos[3 * fc_id + 1] + gazebo::gaussianKernel(data_->mocap_pos_noise);
    pose_msg.pose.position.z = site_xpos[3 * fc_id + 2] + gazebo::gaussianKernel(data_->mocap_pos_noise);

    tf2::Quaternion q_delta;
    q_delta.setRPY(gazebo::gaussianKernel(data_->mocap_rot_noise), gazebo::gaussianKernel(data_->mocap_rot_noise),
                   gazebo::gaussianKernel(data_->mocap_rot_noise));
    const tf2::Quaternion q_noise = fc_quat * q_delta;
    pose_msg.pose.orientation.x = q_noise.x();
    pose_msg.pose.orientation.y = q_noise.y();
    pose_msg.pose.orientation.z = q_noise.z();
    pose_msg.pose.orientation.w = q_noise.w();

    data_->mocap_pub->publish(pose_msg);
    data_->last_mocap_time = time;
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

  /* Run the flight controller and take its rotor forces. The ros2_control
     command interfaces are still exported and still honoured, but in the spinal
     configuration the controller is the authority, exactly as on the board. */
  data_->spinal.update();

  for (std::size_t i = 0; i < data_->rotors.size(); ++i)
  {
    auto& rotor = data_->rotors[i];
    if (rotor.actuator_id < 0)
    {
      continue;
    }

    rotor.command = data_->spinal.getForce(static_cast<int>(i));
    data_->data->ctrl[rotor.actuator_id] =
        clampToRange(rotor.command, rotor.lower_command, rotor.upper_command);
  }
  data_->last_update_sim_time = time;

  return hardware_interface::return_type::OK;
}

}  // namespace aerial_robot_simulation

PLUGINLIB_EXPORT_CLASS(aerial_robot_simulation::AerialRobotMujocoSystem,
                       mujoco_ros::control::MujocoRosSystemInterface)
