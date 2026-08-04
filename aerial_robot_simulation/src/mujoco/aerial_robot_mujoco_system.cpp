#include <aerial_robot_simulation/mujoco/aerial_robot_mujoco_system.h>

#include <algorithm>
#include <limits>
#include <string>
#include <thread>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <aerial_robot_ros_compat/tf_compat.h>
#include <aerial_robot_simulation/mujoco/aerial_robot_spinal.h>
#include <aerial_robot_simulation/noise_model.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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

/**
 * Every parameter override reaching a node, whatever route it came by.
 *
 * NodeOptions::parameter_overrides() holds only what was set programmatically
 * on the options object. Everything supplied by a `--params-file` - which is
 * how the whole simulation is configured - arrives through the node's
 * arguments instead and never appears there, so copying the options across
 * would hand the flight controller an empty parameter set and let every
 * getParam() below silently take its default.
 */
std::vector<rclcpp::Parameter> collectParameterOverrides(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& model_nh)
{
  std::vector<rclcpp::Parameter> overrides;
  for (const auto& entry : model_nh->get_node_parameters_interface()->get_parameter_overrides())
  {
    overrides.emplace_back(entry.first, entry.second);
  }
  return overrides;
}

/**
 * The `controllerN` keys under one servo group, in the order Servo.yaml lists
 * them - which the file itself says must follow the kinematic chain.
 *
 * The parameters keep the ROS1 file's shape rather than being flattened by the
 * launch, so Servo.yaml stays the single copy for both versions. rclcpp gives
 * them back dot-joined, so the group's children are found by prefix.
 */
std::vector<std::string> servoControllerKeys(const ros_compat::NodeHandle& nh, const std::string& group_prefix)
{
  std::vector<std::string> keys;
  const auto names = nh.node()->list_parameters({ group_prefix }, 0).names;
  for (const auto& name : names)
  {
    const std::string tail = name.substr(group_prefix.size() + 1);
    const std::string key = tail.substr(0, tail.find('.'));
    if (key.rfind("controller", 0) != 0)
      continue;
    if (std::find(keys.begin(), keys.end(), key) == keys.end())
      keys.push_back(key);
  }
  /* controller10 must not sort before controller2. */
  std::sort(keys.begin(), keys.end(), [](const std::string& a, const std::string& b) {
    const std::size_t prefix = std::string("controller").size();
    return std::stoi(a.substr(prefix)) < std::stoi(b.substr(prefix));
  });
  return keys;
}

/** The servo group names: the direct children of `servo_controller`. */
std::vector<std::string> servoGroupNames(const ros_compat::NodeHandle& nh)
{
  std::vector<std::string> groups;
  const auto names = nh.node()->list_parameters({ "servo_controller" }, 0).names;
  for (const auto& name : names)
  {
    const std::string tail = name.substr(std::string("servo_controller.").size());
    const std::string group = tail.substr(0, tail.find('.'));
    if (group.empty() || group == tail)
      continue;  // a scalar directly under servo_controller, not a group
    if (std::find(groups.begin(), groups.end(), group) == groups.end())
      groups.push_back(group);
  }
  return groups;
}

/**
 * A `simulation:` value for one servo, falling back to its group's.
 *
 * Servo.yaml puts the defaults on the group and lets a single controller
 * override them, and servo_bridge resolves it the same way round.
 */
template <class T>
bool servoSimParam(const ros_compat::NodeHandle& group_nh, const std::string& controller_key,
                   const std::string& name, T& value)
{
  ros_compat::NodeHandle controller_nh(group_nh, controller_key + "/simulation");
  if (controller_nh.getParam(name, value))
    return true;
  ros_compat::NodeHandle group_sim_nh(group_nh, "simulation");
  return group_sim_nh.getParam(name, value);
}

/**
 * The namespace the robot lives in, as an absolute ROS2 namespace.
 *
 * mujoco_ros gives every plugin a node under /<mujoco server>/<plugin>, which
 * is not where the robot is. mujoco_ros_control's own `namespace` parameter is
 * the robot namespace - it is what it prefixes onto robot_description_node and
 * what it hands the controller manager - so read the same one rather than
 * inventing a second knob that could disagree with it.
 */
std::string resolveRobotNamespace(const rclcpp_lifecycle::LifecycleNode::SharedPtr& model_nh)
{
  std::string ns;
  model_nh->get_parameter_or(std::string("namespace"), ns, std::string(""));
  if (ns.empty())
    return "/";
  return ns.front() == '/' ? ns : "/" + ns;
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

  /**
   * One position-controlled servo joint.
   *
   * Under ROS1 these were not the hardware sim's business at all: servo_bridge
   * - the interface the stack uses to reach the servos on both the real machine
   * and in simulation - loaded an effort_controllers/JointPositionController per
   * joint through the controller manager, and published the initial angle to it.
   * ROS2 has no such controller in the distributions this targets, so the PID
   * lives here, driven by the very same `simulation:` block of Servo.yaml. A
   * real-machine servo_bridge, which converts joint commands into servo units
   * for rosserial, is a separate job and still to do.
   */
  struct ServoData
  {
    std::string name;
    int joint_id = -1;
    int actuator_id = -1;
    int qpos_adr = -1;
    int qvel_adr = -1;
    double p_gain = 0.0;
    double i_gain = 0.0;
    double d_gain = 0.0;
    double i_clamp = 0.0;
    double target = 0.0;
    double error_integral = 0.0;
    double lower_command = -std::numeric_limits<double>::infinity();
    double upper_command = std::numeric_limits<double>::infinity();
  };

  ~Private()
  {
    stopSpinning();
  }

  /** Give the flight controller's node a thread of its own; see initSim(). */
  void startSpinning()
  {
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(spinal_node);
    spin_thread = std::thread([this]() { executor->spin(); });
  }

  void stopSpinning()
  {
    if (executor)
      executor->cancel();
    if (spin_thread.joinable())
      spin_thread.join();
  }

  /* The simulated flight controller: attitude estimator plus control core. */
  AerialRobotSpinal spinal;
  std::shared_ptr<rclcpp::Node> spinal_node;
  ros_compat::NodeHandle nh;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
  std::thread spin_thread;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> servo_ctrl_subs;

  std::vector<ServoData> servos;
  std::mutex servo_target_mutex;
  double joint_state_pub_rate = 0.02;
  double last_joint_state_time = 0.0;

  double mocap_pub_rate = 0.01;
  double mocap_pos_noise = 0.001;
  double mocap_rot_noise = 0.001;

  /* Ground truth noise and drift, the same model the ROS1 hardware sims use.
     mini_quadrotor sets every drift term to zero, so for that robot this
     reduces to gaussian noise - but the parameters are in its Simulation.yaml
     and a parameter that is read and ignored is worse than one that is not
     there. */
  double ground_truth_pub_rate = 0.01;
  double ground_truth_pos_noise = 0.0;
  double ground_truth_vel_noise = 0.0;
  double ground_truth_rot_noise = 0.0;
  double ground_truth_angular_noise = 0.0;
  double ground_truth_vel_drift = 0.0;
  double ground_truth_rot_drift = 0.0;
  double ground_truth_angular_drift = 0.0;
  double ground_truth_vel_drift_frequency = 0.0;
  double ground_truth_rot_drift_frequency = 0.0;
  double ground_truth_angular_drift_frequency = 0.0;
  double vel_curr_drift[3] = { 0.0, 0.0, 0.0 };
  double rot_curr_drift[3] = { 0.0, 0.0, 0.0 };
  double angular_curr_drift[3] = { 0.0, 0.0, 0.0 };
  double last_ground_truth_time = 0.0;
  /* Seconds rather than an rclcpp::Time. mujoco_ros_control times read()/write()
     off a clock it builds itself with RCL_STEADY_TIME, and rclcpp *throws* when
     two Times with different sources are subtracted - which is what this used to
     do against an RCL_ROS_TIME zero, taking the whole simulator down on the
     first control step. Only an elapsed interval is wanted here anyway. */
  double last_mocap_time = 0.0;

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
     its own node, carrying the lifecycle node's parameter overrides so the
     spinal parameters resolve from the same config.

     The namespace is *not* the lifecycle node's. mujoco_ros creates plugin
     nodes at /<mujoco server name>/<plugin name>, which is not where the robot
     lives, and the flight controller's topics - four_axes/command, rpy/gain,
     motor_info - have to meet aerial_robot_control inside the robot namespace.
     mujoco_ros_control already carries that namespace as its `namespace`
     parameter, the one it uses to find robot_state_publisher and to place the
     controller manager, so take it from there. */
  const std::string robot_ns = resolveRobotNamespace(model_nh);

  rclcpp::NodeOptions spinal_options;
  spinal_options.parameter_overrides(collectParameterOverrides(model_nh));
  spinal_options.automatically_declare_parameters_from_overrides(true);
  spinal_options.allow_undeclared_parameters(true);
  /* Global arguments carry the mujoco_server node's own remappings and
     parameter files; applying them to a node in the robot namespace would
     silently rewrite topics that were resolved for a different node. */
  spinal_options.use_global_arguments(false);
  data_->spinal_node = std::make_shared<rclcpp::Node>(std::string(model_nh->get_name()) + "_spinal", robot_ns,
                                                      spinal_options);
  data_->nh = ros_compat::NodeHandle(data_->spinal_node);
  ros_compat::setGlobalNode(data_->nh);
  data_->spinal.init(data_->nh, static_cast<int>(data_->rotors.size()));

  ros_compat::NodeHandle simulation_nh(data_->nh, "simulation");
  ros_compat::getParam<double>(simulation_nh, "mocap_pub_rate", data_->mocap_pub_rate, 0.01);   // [sec]
  ros_compat::getParam<double>(simulation_nh, "mocap_pos_noise", data_->mocap_pos_noise, 0.001);  // m
  ros_compat::getParam<double>(simulation_nh, "mocap_rot_noise", data_->mocap_rot_noise, 0.001);  // rad

  ros_compat::getParam<double>(simulation_nh, "ground_truth_pub_rate", data_->ground_truth_pub_rate, 0.01);  // [sec]
  ros_compat::getParam<double>(simulation_nh, "ground_truth_pos_noise", data_->ground_truth_pos_noise, 0.0);  // m
  ros_compat::getParam<double>(simulation_nh, "ground_truth_vel_noise", data_->ground_truth_vel_noise, 0.0);  // m/s
  ros_compat::getParam<double>(simulation_nh, "ground_truth_rot_noise", data_->ground_truth_rot_noise, 0.0);  // rad
  ros_compat::getParam<double>(simulation_nh, "ground_truth_angular_noise", data_->ground_truth_angular_noise,
                               0.0);  // rad/s
  ros_compat::getParam<double>(simulation_nh, "ground_truth_vel_drift", data_->ground_truth_vel_drift, 0.0);
  ros_compat::getParam<double>(simulation_nh, "ground_truth_rot_drift", data_->ground_truth_rot_drift, 0.0);
  ros_compat::getParam<double>(simulation_nh, "ground_truth_angular_drift", data_->ground_truth_angular_drift, 0.0);
  ros_compat::getParam<double>(simulation_nh, "ground_truth_vel_drift_frequency",
                               data_->ground_truth_vel_drift_frequency, 0.0);
  ros_compat::getParam<double>(simulation_nh, "ground_truth_rot_drift_frequency",
                               data_->ground_truth_rot_drift_frequency, 0.0);
  ros_compat::getParam<double>(simulation_nh, "ground_truth_angular_drift_frequency",
                               data_->ground_truth_angular_drift_frequency, 0.0);

  /* The estimator subscribes to these: with sim_estimate_mode = GROUND_TRUTH,
     ground_truth is the only source of pose it has. Published from the spinal
     node, so they land in the robot namespace where the estimator listens,
     rather than under the mujoco server's plugin node. */
  data_->ground_truth_pub = data_->spinal_node->create_publisher<nav_msgs::msg::Odometry>("ground_truth", 1);
  data_->mocap_pub = data_->spinal_node->create_publisher<geometry_msgs::msg::PoseStamped>("mocap/pose", 1);

  initServos(m);

  /* Nobody else spins this node. mujoco_ros_control adds its own node and the
     controller manager to the environment's executor, but it knows nothing
     about ours - and the firmware's subscriptions (four_axes/command, rpy/gain,
     motor_info) are what the flight control core is driven by, so unspun it
     holds attitude against a target that never arrives. */
  data_->startSpinning();

  RCLCPP_INFO(nh_->get_logger(), "Simulated flight controller running in namespace `%s`.",
              data_->spinal_node->get_namespace());

  return true;
}

void AerialRobotMujocoSystem::initServos(const mjModel* m)
{
  ros_compat::NodeHandle servo_nh(data_->nh, "servo_controller");

  for (const auto& group : servoGroupNames(data_->nh))
  {
    ros_compat::NodeHandle group_nh(servo_nh, group);

    for (const auto& key : servoControllerKeys(data_->nh, "servo_controller." + group))
    {
      ros_compat::NodeHandle controller_nh(group_nh, key);

      Private::ServoData servo;
      if (!controller_nh.getParam("name", servo.name))
      {
        RCLCPP_ERROR(nh_->get_logger(), "servo_controller/%s/%s has no `name`.", group.c_str(), key.c_str());
        continue;
      }

      servo.joint_id = mj_name2id(m, mjOBJ_JOINT, servo.name.c_str());
      servo.actuator_id = mj_name2id(m, mjOBJ_ACTUATOR, servo.name.c_str());
      if (servo.joint_id < 0 || servo.actuator_id < 0)
      {
        RCLCPP_ERROR(nh_->get_logger(),
                     "Servo `%s` needs a MuJoCo joint and an actuator of the same name; found joint %d, actuator %d.",
                     servo.name.c_str(), servo.joint_id, servo.actuator_id);
        continue;
      }
      servo.qpos_adr = m->jnt_qposadr[servo.joint_id];
      servo.qvel_adr = m->jnt_dofadr[servo.joint_id];

      servoSimParam(group_nh, key, "pid.p", servo.p_gain);
      servoSimParam(group_nh, key, "pid.i", servo.i_gain);
      servoSimParam(group_nh, key, "pid.d", servo.d_gain);
      /* i_clamp_max is what the gimbal configs spell; hydrus omits it. */
      if (!servoSimParam(group_nh, key, "pid.i_clamp_max", servo.i_clamp))
        servo.i_clamp = std::numeric_limits<double>::infinity();
      servoSimParam(group_nh, key, "init_value", servo.target);

      if (m->actuator_ctrllimited[servo.actuator_id])
      {
        servo.lower_command = m->actuator_ctrlrange[2 * servo.actuator_id];
        servo.upper_command = m->actuator_ctrlrange[2 * servo.actuator_id + 1];
      }

      RCLCPP_INFO(nh_->get_logger(), "Servo `%s`: holding %.3f rad with p=%.1f i=%.3f d=%.1f.",
                  servo.name.c_str(), servo.target, servo.p_gain, servo.i_gain, servo.d_gain);
      data_->servos.push_back(servo);
    }

    /* The topic servo_bridge takes joint commands on, so whatever drives the
       robot does not have to know it is talking to a simulator. */
    const std::string topic = group + "_ctrl";
    data_->servo_ctrl_subs.push_back(data_->spinal_node->create_subscription<sensor_msgs::msg::JointState>(
        topic, 1, [this](const sensor_msgs::msg::JointState::SharedPtr msg) { setServoTargets(*msg); }));
  }

  if (data_->servos.empty())
    return;

  /* The robot model tracks the configuration through this: without it a
     transformable robot's kinematics stay at the pose it was launched in, and
     its LQI gains are computed for a shape it is not in. Under ROS1 this came
     from joint_state_controller. */
  data_->joint_state_pub = data_->spinal_node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
}

void AerialRobotMujocoSystem::setServoTargets(const sensor_msgs::msg::JointState& msg)
{
  std::lock_guard<std::mutex> lock(data_->servo_target_mutex);
  for (std::size_t i = 0; i < msg.position.size(); ++i)
  {
    /* By name when the message carries them, by index otherwise - which is what
       the ROS1 servo command path allowed too. */
    if (i < msg.name.size())
    {
      auto it = std::find_if(data_->servos.begin(), data_->servos.end(),
                             [&](const Private::ServoData& s) { return s.name == msg.name[i]; });
      if (it != data_->servos.end())
        it->target = msg.position[i];
    }
    else if (i < data_->servos.size())
    {
      data_->servos[i].target = msg.position[i];
    }
  }
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
                                                              const rclcpp::Duration& period)
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

  /* Velocity of the `fc` site. mj_objectVelocity gives a spatial vector
     [angular; linear]; flg_local = 0 asks for it in the world frame, which is
     the frame ground_truth's linear velocity is defined in. */
  mjtNum fc_vel[6] = { 0 };
  mj_objectVelocity(data_->model, data_->data, mjOBJ_SITE, fc_id, fc_vel, 0);

  const double dt = period.seconds();

  /* The estimator takes velocity straight out of this message when it is in
     GROUND_TRUTH mode - `groundTruthCallback` reads twist.twist.linear with no
     differentiation of its own. Leaving the twist at zero therefore does not
     degrade the estimate, it pins the velocity at zero: the position gains then
     fly against a dead D term and the robot walks away. Filling it in is the
     contract the ROS1 Gazebo hardware sim publishes to; the ROS1 MuJoCo one
     advertised this topic and never published on it at all, which is why the
     gap did not show up there. */
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = time;
  odom_msg.pose.pose.position.x =
      site_xpos[3 * fc_id + 0] + gazebo::gaussianKernel(data_->ground_truth_pos_noise);
  odom_msg.pose.pose.position.y =
      site_xpos[3 * fc_id + 1] + gazebo::gaussianKernel(data_->ground_truth_pos_noise);
  odom_msg.pose.pose.position.z =
      site_xpos[3 * fc_id + 2] + gazebo::gaussianKernel(data_->ground_truth_pos_noise);

  tf2::Quaternion q_delta;
  q_delta.setRPY(gazebo::addNoise(data_->rot_curr_drift[0], data_->ground_truth_rot_drift,
                                  data_->ground_truth_rot_drift_frequency, 0, data_->ground_truth_rot_noise, dt),
                 gazebo::addNoise(data_->rot_curr_drift[1], data_->ground_truth_rot_drift,
                                  data_->ground_truth_rot_drift_frequency, 0, data_->ground_truth_rot_noise, dt),
                 gazebo::addNoise(data_->rot_curr_drift[2], data_->ground_truth_rot_drift,
                                  data_->ground_truth_rot_drift_frequency, 0, data_->ground_truth_rot_noise, dt));
  const tf2::Quaternion q_ground_truth = fc_quat * q_delta;
  odom_msg.pose.pose.orientation.x = q_ground_truth.x();
  odom_msg.pose.pose.orientation.y = q_ground_truth.y();
  odom_msg.pose.pose.orientation.z = q_ground_truth.z();
  odom_msg.pose.pose.orientation.w = q_ground_truth.w();

  odom_msg.twist.twist.linear.x =
      fc_vel[3] + gazebo::addNoise(data_->vel_curr_drift[0], data_->ground_truth_vel_drift,
                                   data_->ground_truth_vel_drift_frequency, 0, data_->ground_truth_vel_noise, dt);
  odom_msg.twist.twist.linear.y =
      fc_vel[4] + gazebo::addNoise(data_->vel_curr_drift[1], data_->ground_truth_vel_drift,
                                   data_->ground_truth_vel_drift_frequency, 0, data_->ground_truth_vel_noise, dt);
  odom_msg.twist.twist.linear.z =
      fc_vel[5] + gazebo::addNoise(data_->vel_curr_drift[2], data_->ground_truth_vel_drift,
                                   data_->ground_truth_vel_drift_frequency, 0, data_->ground_truth_vel_noise, dt);

  /* CAUTION: the angular velocity is in the fc frame, not the world frame -
     same as the ROS1 hardware sim, and why it is the gyro reading rather than
     the world-frame part of fc_vel. */
  odom_msg.twist.twist.angular.x =
      gyro.x() + gazebo::addNoise(data_->angular_curr_drift[0], data_->ground_truth_angular_drift,
                                  data_->ground_truth_angular_drift_frequency, 0,
                                  data_->ground_truth_angular_noise, dt);
  odom_msg.twist.twist.angular.y =
      gyro.y() + gazebo::addNoise(data_->angular_curr_drift[1], data_->ground_truth_angular_drift,
                                  data_->ground_truth_angular_drift_frequency, 0,
                                  data_->ground_truth_angular_noise, dt);
  odom_msg.twist.twist.angular.z =
      gyro.z() + gazebo::addNoise(data_->angular_curr_drift[2], data_->ground_truth_angular_drift,
                                  data_->ground_truth_angular_drift_frequency, 0,
                                  data_->ground_truth_angular_noise, dt);

  if (time.seconds() - data_->last_ground_truth_time >= data_->ground_truth_pub_rate)
  {
    data_->ground_truth_pub->publish(odom_msg);
    data_->last_ground_truth_time = time.seconds();
  }

  data_->spinal.setGroundTruthStates(fc_quat.x(), fc_quat.y(), fc_quat.z(), fc_quat.w(), gyro.x(), gyro.y(), gyro.z());

  if (time.seconds() - data_->last_mocap_time >= data_->mocap_pub_rate)
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
    data_->last_mocap_time = time.seconds();
  }

  publishJointStates(time);

  return hardware_interface::return_type::OK;
}

void AerialRobotMujocoSystem::publishJointStates(const rclcpp::Time& time)
{
  if (data_->joint_state_pub == nullptr ||
      time.seconds() - data_->last_joint_state_time < data_->joint_state_pub_rate)
  {
    return;
  }

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = time;
  for (const auto& servo : data_->servos)
  {
    msg.name.push_back(servo.name);
    msg.position.push_back(data_->data->qpos[servo.qpos_adr]);
    msg.velocity.push_back(data_->data->qvel[servo.qvel_adr]);
    msg.effort.push_back(data_->data->actuator_force[servo.actuator_id]);
  }
  data_->joint_state_pub->publish(msg);
  data_->last_joint_state_time = time.seconds();
}

hardware_interface::return_type AerialRobotMujocoSystem::write(const rclcpp::Time& time,
                                                               const rclcpp::Duration& period)
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

  updateServos(period.seconds());

  data_->last_update_sim_time = time;

  return hardware_interface::return_type::OK;
}

void AerialRobotMujocoSystem::updateServos(const double dt)
{
  if (data_->servos.empty())
    return;

  std::lock_guard<std::mutex> lock(data_->servo_target_mutex);
  for (auto& servo : data_->servos)
  {
    const double position = data_->data->qpos[servo.qpos_adr];
    const double velocity = data_->data->qvel[servo.qvel_adr];
    const double error = servo.target - position;

    if (dt > 0.0 && servo.i_gain != 0.0)
    {
      servo.error_integral =
          clampToRange(servo.error_integral + error * dt, -servo.i_clamp, servo.i_clamp);
    }

    /* The derivative term is on the measurement, not on the error: the target
       is a step when a new joint command arrives, and differentiating that
       kicks the servo hard enough to disturb a hovering robot. */
    const double effort =
        servo.p_gain * error + servo.i_gain * servo.error_integral - servo.d_gain * velocity;
    data_->data->ctrl[servo.actuator_id] =
        clampToRange(effort, servo.lower_command, servo.upper_command);
  }
}

}  // namespace aerial_robot_simulation

PLUGINLIB_EXPORT_CLASS(aerial_robot_simulation::AerialRobotMujocoSystem,
                       mujoco_ros::control::MujocoRosSystemInterface)
