#pragma once

#include <memory>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mujoco_ros_control/ros_two/mujoco_ros_system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace aerial_robot_simulation
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AerialRobotMujocoSystem : public mujoco_ros::control::MujocoRosSystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  bool initSim(rclcpp_lifecycle::LifecycleNode::SharedPtr& model_nh,
               const hardware_interface::HardwareInfo& hardware_info, const mjModel* m, mjData* d,
               unsigned int& update_rate) override;

private:
  /**
   * Position-controlled servo joints, driven from Servo.yaml's `simulation:`
   * block. Folded in here for the same reason the flight controller is: ROS1
   * gave each one an effort_controllers/JointPositionController, loaded through
   * servo_bridge, and ROS2 has no equivalent controller to load.
   */
  void initServos(const mjModel* m);
  void updateServos(double dt);
  void setServoTargets(const sensor_msgs::msg::JointState& msg);
  void publishJointStates(const rclcpp::Time& time);

  class Private;
  std::unique_ptr<Private> data_;
};

}  // namespace aerial_robot_simulation
