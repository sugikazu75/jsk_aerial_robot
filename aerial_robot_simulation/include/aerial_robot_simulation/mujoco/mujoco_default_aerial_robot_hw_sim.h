#pragma once

#include <aerial_robot_simulation/rotor_interface.h>
#include <mujoco_ros_control/default_robot_hw_sim.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

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

  double odom_pub_rate_;
  double odom_pub_last_time_;
  ros::Publisher odom_pub_;

  double tf_broadcast_rate_;
  double tf_broadcast_last_time_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string robot_ns_;

  std::vector<std::string> rotor_names_;
  std::vector<double> rotor_cmd_;
  std::vector<double> rotor_pos_;
  std::vector<double> rotor_vel_;
  std::vector<double> rotor_eff_;

  void publishOdometry(ros::Time time);
  void publishTF(ros::Time time);
};
}  // namespace mujoco_ros_control
