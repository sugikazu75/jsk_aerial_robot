// Plans a walk when the number of steps is given on "walk/plan_request", saves the trajectory to a CSV file, and
// tells its path to the walk controller on "walk/trajectory".
// The walk starts from the measured state: the baselink pose ("uav/baselink/odom") and the joint positions
// ("joint_states"). With use_measured_state: false, it starts from initial_configuration at the origin instead.

#include <flying_humanoid/walk_planner.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <ctime>
#include <filesystem>
#include <memory>

namespace
{
template <class T>
T param(ros::NodeHandle& nh, const std::string& name, const T& default_value)
{
  T value;
  nh.param<T>(name, value, default_value);
  return value;
}

Eigen::VectorXd vectorParam(ros::NodeHandle& nh, const std::string& name, const Eigen::VectorXd& default_value)
{
  std::vector<double> value;
  if (!nh.getParam(name, value))
    return default_value;
  return Eigen::Map<Eigen::VectorXd>(value.data(), value.size());
}

flying_humanoid::WalkPlannerParams loadParams(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  flying_humanoid::WalkPlannerParams p;
  p.lfoot = param(pnh, "lfoot", p.lfoot);
  p.rfoot = param(pnh, "rfoot", p.rfoot);
  p.step_length = param(pnh, "step_length", p.step_length);
  p.step_height = param(pnh, "step_height", p.step_height);
  p.time_step = param(pnh, "time_step", p.time_step);
  p.step_knots = param(pnh, "step_knots", p.step_knots);
  p.support_knots = param(pnh, "support_knots", p.support_knots);
  p.lateral_shift_scale = param(pnh, "lateral_shift_scale", p.lateral_shift_scale);
  pnh.getParam("initial_configuration", p.initial_configuration);
  nh.getParam("motor_info/m_f_rate", p.m_f_rate);

  p.solver_type = param(pnh, "solver_type", p.solver_type);
  p.max_iter = param(pnh, "max_iterations", p.max_iter);
  p.num_threads = param(pnh, "num_threads", p.num_threads);

  p.x_weights = vectorParam(pnh, "x_weights", p.x_weights);
  p.state_weight = param(pnh, "state_weight", p.state_weight);
  p.control_weight = param(pnh, "control_weight", p.control_weight);
  p.thrust_reg_weight = vectorParam(pnh, "thrust_reg_weight", p.thrust_reg_weight);
  p.thrust_barrier_weight = param(pnh, "thrust_barrier_weight", p.thrust_barrier_weight);
  p.com_track_weight = vectorParam(pnh, "com_track_weight", p.com_track_weight);
  p.foot_track_weight = vectorParam(pnh, "foot_track_weight", p.foot_track_weight);
  p.centroidal_momentum_weight = vectorParam(pnh, "centroidal_momentum_weight", p.centroidal_momentum_weight);
  p.contact_wrench_weight = param(pnh, "contact_wrench_weight", p.contact_wrench_weight);
  p.contact_wrench_margin = param(pnh, "contact_wrench_margin", p.contact_wrench_margin);
  p.state_bound_weight = param(pnh, "state_bound_weight", p.state_bound_weight);
  p.state_bound_margin = param(pnh, "state_bound_margin", p.state_bound_margin);
  p.minimum_contact_force = param(pnh, "minimum_contact_force", p.minimum_contact_force);
  p.mu = param(pnh, "mu", p.mu);
  p.foot_size = vectorParam(pnh, "foot_size", p.foot_size);
  p.contact_gains = vectorParam(pnh, "contact_gains", p.contact_gains);
  p.contact_jacobian_damping = param(pnh, "contact_jacobian_damping", p.contact_jacobian_damping);
  p.touchdown_velocity_weight = param(pnh, "touchdown_velocity_weight", p.touchdown_velocity_weight);
  p.touchdown_constraint = param(pnh, "touchdown_constraint", p.touchdown_constraint);
  p.terminal_posture_constraint = param(pnh, "terminal_posture_constraint", p.terminal_posture_constraint);
  return p;
}
}  // namespace

class WalkPlannerNode
{
public:
  WalkPlannerNode() : pnh_("~")
  {
    std::string robot_description;
    while (!nh_.getParam("robot_description", robot_description) && ros::ok())
    {
      ROS_WARN_THROTTLE(1.0, "waiting for robot_description");
      ros::Duration(0.1).sleep();
    }
    planner_ = std::make_unique<flying_humanoid::WalkPlanner>(robot_description, loadParams(nh_, pnh_));

    result_dir_ = param<std::string>(pnh_, "result_dir", "/tmp/flying_humanoid_walk");
    diagnosis_ = param<bool>(pnh_, "diagnosis", false);
    use_measured_state_ = param<bool>(pnh_, "use_measured_state", true);
    baselink_ = param<std::string>(pnh_, "baselink", "fc");
    std::filesystem::create_directories(result_dir_);

    trajectory_pub_ = nh_.advertise<std_msgs::String>("walk/trajectory", 1);
    request_sub_ = nh_.subscribe("walk/plan_request", 1, &WalkPlannerNode::requestCallback, this);
    odom_sub_ = nh_.subscribe("uav/baselink/odom", 1, &WalkPlannerNode::odomCallback, this);
    joint_state_sub_ = nh_.subscribe("joint_states", 1, &WalkPlannerNode::jointStateCallback, this);
    ROS_INFO("walk planner is ready. publish the number of steps to %s", request_sub_.getTopic().c_str());
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    const auto& p = msg->pose.pose;
    baselink_pose_ = pinocchio::SE3(
        Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z).toRotationMatrix(),
        Eigen::Vector3d(p.position.x, p.position.y, p.position.z));
    odom_received_ = true;
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
      joint_positions_[msg->name[i]] = msg->position[i];
  }

  void requestCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    if (msg->data < 0)
    {
      ROS_ERROR("the number of steps must not be negative: %d", msg->data);
      return;
    }

    Eigen::VectorXd q_start;
    try
    {
      if (!use_measured_state_)
        q_start = planner_->defaultConfiguration();
      else if (!odom_received_)
        throw std::runtime_error("no odometry on " + odom_sub_.getTopic());
      else
        q_start = planner_->measuredConfiguration(baselink_pose_, joint_positions_, baselink_);
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("cannot get the state to start the walk from: %s", e.what());
      return;
    }

    ROS_INFO("planning a walk of %d steps...", msg->data);
    if (!planner_->plan(msg->data, q_start))
      ROS_WARN("the walk planning did not converge. check the trajectory before replaying it");

    char stamp[32];
    const std::time_t now = std::time(nullptr);
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
    const std::string filename = result_dir_ + "/walk_" + std::to_string(msg->data) + "steps_" + stamp + ".csv";
    if (!planner_->saveCsv(filename))
    {
      ROS_ERROR("failed to write %s", filename.c_str());
      return;
    }

    ROS_INFO("saved the walk trajectory to %s", filename.c_str());

    if (diagnosis_)
    {
      const std::string base = filename.substr(0, filename.size() - 4);
      planner_->saveSolverLog(base + "_solver.csv");
      planner_->saveNodeDiagnosis(base + "_nodes.csv");
      planner_->checkDerivatives();
    }
    std_msgs::String path;
    path.data = filename;
    trajectory_pub_.publish(path);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher trajectory_pub_;
  ros::Subscriber request_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber joint_state_sub_;
  std::unique_ptr<flying_humanoid::WalkPlanner> planner_;
  std::string result_dir_;
  bool diagnosis_;  // save the solver log and the node diagnosis, and check the derivatives
  bool use_measured_state_;
  std::string baselink_;  // frame of uav/baselink/odom
  pinocchio::SE3 baselink_pose_;
  bool odom_received_ = false;
  std::map<std::string, double> joint_positions_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "walk_planner");
  WalkPlannerNode node;
  ros::spin();
  return 0;
}
