// -*- mode: c++ -*-
// Standalone MPC node for gain tuning without simulation.
// Runs MPC in a closed loop using ideal (disturbance-free) dynamics.

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <aerial_robot_control/control/fwddyn_mpc_control_problem.hpp>
#include <aerial_robot_dynamics/robot_model.h>

#include <random>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_standalone_node");
  ros::NodeHandle nh;

  auto pinocchio_robot_model = std::make_shared<aerial_robot_dynamics::PinocchioRobotModel>(true);
  auto pin_model = pinocchio_robot_model->getModel();
  const int rotor_num = pinocchio_robot_model->getRotorNum();
  const int nq = pin_model->nq;
  const int nv = pin_model->nv;

  // --- Load MPC parameters ---
  aerial_robot_control::FwddynMpcControlProblem::Parameters mpc_params;
  ros::NodeHandle mpc_nh(nh, "mpc");
  mpc_nh.param<int>("num_nodes", mpc_params.num_nodes, mpc_params.num_nodes);
  mpc_nh.param<int>("max_iter", mpc_params.max_iter, mpc_params.max_iter);
  mpc_nh.param<int>("max_init_iter", mpc_params.max_init_iter, mpc_params.max_init_iter);
  mpc_nh.param<double>("dt", mpc_params.dt, mpc_params.dt);
  mpc_nh.param<double>("control_weight", mpc_params.control_weight, mpc_params.control_weight);
  mpc_nh.param<double>("thrust_reg_weight", mpc_params.thrust_reg_weight, mpc_params.thrust_reg_weight);
  mpc_nh.param<double>("thrust_barrier_weight", mpc_params.thrust_barrier_weight, mpc_params.thrust_barrier_weight);
  mpc_nh.param<double>("delta_thrust_max", mpc_params.delta_thrust_max, mpc_params.delta_thrust_max);

  // CoM tracking weight (3 elements)
  std::vector<double> com_w, x_w, x_ref_vec;
  if (mpc_nh.getParam("com_track_weight", com_w))
  {
    if ((int)com_w.size() == 3)
    {
      Eigen::Vector3d com_track_weight = Eigen::Map<Eigen::Vector3d>(com_w.data());
      mpc_params.com_track_weight = com_track_weight;
    }
    else
      ROS_ERROR_STREAM("[mpc_standalone] com_track_weight size mismatch: expected 3, got " << com_w.size());
  }
  else
    ROS_ERROR("[mpc_standalone] Failed to load com_track_weight");

  // state weight (2*nv elements)
  if (mpc_nh.getParam("x_state_weight", x_w))
  {
    if ((int)x_w.size() == 2 * nv)
    {
      Eigen::VectorXd x_state_weight = Eigen::Map<Eigen::VectorXd>(x_w.data(), x_w.size());
      mpc_params.x_state_weight = x_state_weight;
    }
    else
      ROS_ERROR_STREAM("[mpc_standalone] x_state_weight size mismatch: expected " << 2 * nv << ", got " << x_w.size());
  }
  else
    ROS_ERROR("[mpc_standalone] Failed to load x_state_weight (expected size %d)", 2 * nv);

  mpc_params.print();

  // --- Load x_ref ---
  Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(nq + nv);
  if (mpc_nh.getParam("x_ref", x_ref_vec))
  {
    if ((int)x_ref_vec.size() == nq + nv)
    {
      x_ref = Eigen::Map<Eigen::VectorXd>(x_ref_vec.data(), x_ref_vec.size());
    }
    else
      ROS_ERROR_STREAM("[mpc_standalone] x_ref size mismatch: expected " << nq + nv << ", got " << x_ref_vec.size());
  }
  else
    ROS_ERROR("[mpc_standalone] Failed to get x_ref from parameter server (expected size %d)", nq + nv);

  x_ref(3) = 0.0;  // identity quaternion x=0
  x_ref(4) = 0.0;  // identity quaternion y=0
  x_ref(5) = 0.0;  // identity quaternion z=0
  x_ref(6) = 1.0;  // identity quaternion w=1

  // --- Initial state: start on the ground (z=0), target hover at z=1.0 ---
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nq + nv + rotor_num);
  x0.head(nq) = x_ref.head(nq);

  const Eigen::Vector3d com_target(0.0, 0.0, 1.0);

  // --- Initialize and initial-solve MPC ---
  aerial_robot_control::FwddynMpcControlProblem mpc_problem;
  mpc_problem.initialize(pinocchio_robot_model, mpc_params);
  mpc_problem.buildMPCProblem(x0, com_target, x_ref);
  ROS_INFO("[mpc_standalone] Initial solve (%d iter)...", mpc_params.max_init_iter);
  mpc_problem.solveMPC(mpc_params.max_init_iter, true, false);
  ROS_INFO("[mpc_standalone] Initial solve done.");

  bool noise_enable;
  double noise_pos_std, noise_rpy_std, noise_vel_std, noise_angvel_std;
  double noise_joint_pos_std, noise_joint_vel_std;
  nh.param<bool>("mpc_standalone/noise/enable", noise_enable, false);
  nh.param<double>("mpc_standalone/noise/pos_std", noise_pos_std, 0.01);              // [m]
  nh.param<double>("mpc_standalone/noise/rpy_std", noise_rpy_std, 0.01);              // [rad]
  nh.param<double>("mpc_standalone/noise/vel_std", noise_vel_std, 0.01);              // [m/s]
  nh.param<double>("mpc_standalone/noise/angvel_std", noise_angvel_std, 0.01);        // [rad/s]
  nh.param<double>("mpc_standalone/noise/joint_pos_std", noise_joint_pos_std, 0.01);  // [rad]
  nh.param<double>("mpc_standalone/noise/joint_vel_std", noise_joint_vel_std, 0.01);  // [rad/s]

  std::default_random_engine rng(std::random_device{}());
  std::normal_distribution<double> ndist(0.0, 1.0);  // unit normal; scaled per component

  ROS_INFO("[mpc_standalone] noise: enable=%s  pos=%.4f rpy=%.4f vel=%.4f angvel=%.4f jpos=%.4f jvel=%.4f",
           noise_enable ? "true" : "false", noise_pos_std, noise_rpy_std, noise_vel_std, noise_angvel_std,
           noise_joint_pos_std, noise_joint_vel_std);

  std::string tf_ns = ros::this_node::getNamespace();
  if (!tf_ns.empty() && tf_ns[0] == '/')
    tf_ns = tf_ns.substr(1);

  // local pinocchio Data for CoM computation (separate from the shared data in robot_model)
  pinocchio::Data pin_data_local(*pin_model);

  tf::TransformBroadcaster tf_broadcaster;
  auto thrust_rate_pub = nh.advertise<std_msgs::Float64MultiArray>("mpc/thrust_rate", 10);
  auto target_cog_pos_pub = nh.advertise<geometry_msgs::PointStamped>("mpc_target_cog_pos", 10);
  auto traj_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("mpc/predicted_trajectory", 10);
  auto joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

  ROS_INFO("[mpc_standalone] MPC loop at %.2f Hz (dt=%.3f s)", 1.0 / mpc_params.dt, mpc_params.dt);

  ros::Rate rate(1.0 / mpc_params.dt);
  while (ros::ok())
  {
    // advance simulated state with ideal model dynamics
    const auto& xs = mpc_problem.xs();
    if (xs.size() >= 2)
    {
      if (!xs[1].array().isFinite().all())
        ROS_WARN_THROTTLE(1.0, "[mpc_standalone] NaN in xs[1], keeping previous x0");
      else
        x0 = xs[1];
    }

    // add zero-mean Gaussian noise to the state for robustness evaluation
    if (noise_enable)
    {
      // position
      for (int i = 0; i < 3; ++i)
        x0(i) += noise_pos_std * ndist(rng);
      // orientation: perturb as small rotation [droll, dpitch, dyaw], then re-normalise.
      // guard against zero-vector noise to avoid NaN from normalized()
      {
        Eigen::Vector3d angle_noise(noise_rpy_std * ndist(rng), noise_rpy_std * ndist(rng), noise_rpy_std * ndist(rng));
        const double angle = angle_noise.norm();
        Eigen::Quaterniond q(x0(6), x0(3), x0(4), x0(5));  // w, x, y, z
        if (angle > 1e-10)
          q = q * Eigen::Quaterniond(Eigen::AngleAxisd(angle, angle_noise / angle));
        q.normalize();
        x0(3) = q.x();
        x0(4) = q.y();
        x0(5) = q.z();
        x0(6) = q.w();
      }
      // linear velocity
      for (int i = 0; i < 3; ++i)
        x0(nq + i) += noise_vel_std * ndist(rng);
      // angular velocity
      for (int i = 0; i < 3; ++i)
        x0(nq + 3 + i) += noise_angvel_std * ndist(rng);
      // joint positions and velocities (skip free-flyer: q[7..nq), v[6..nv))
      for (int i = 7; i < nq; ++i)
        x0(i) += noise_joint_pos_std * ndist(rng);
      for (int i = 6; i < nv; ++i)
        x0(nq + i) += noise_joint_vel_std * ndist(rng);
    }

    mpc_problem.setInitialState(x0);
    mpc_problem.slideHorizon(com_target, x_ref);
    mpc_problem.solveMPC(mpc_params.max_iter, false, true);

    const ros::Time stamp = ros::Time::now();
    const auto& xs_new = mpc_problem.xs();
    const auto& us_new = mpc_problem.us();

    // TF: broadcast root frame (skip if x0 contains NaN)
    if (!x0.head(7).array().isFinite().all())
    {
      tf::Transform root_transform;
      root_transform.setOrigin(tf::Vector3(x0(0), x0(1), x0(2)));
      tf::Quaternion root_quat(x0(3), x0(4), x0(5), x0(6));
      root_quat.normalize();
      root_transform.setRotation(root_quat);
      tf_broadcaster.sendTransform(tf::StampedTransform(root_transform, stamp, "world", tf_ns + "/root"));
    }

    // TF: broadcast predicted trajectory frames (skip individual frames that contain NaN)
    for (size_t i = 0; i < xs_new.size(); ++i)
    {
      if ((int)xs_new[i].size() < 7 || !xs_new[i].head(7).array().isFinite().all())
        continue;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(xs_new[i](0), xs_new[i](1), xs_new[i](2)));
      tf::Quaternion quat(xs_new[i](3), xs_new[i](4), xs_new[i](5), xs_new[i](6));
      quat.normalize();
      transform.setRotation(quat);
      tf_broadcaster.sendTransform(
          tf::StampedTransform(transform, stamp, "world", tf_ns + "/mpc_optimized_root_" + std::to_string(i)));
    }

    // joint_states (skip if x0 contains NaN to avoid publishing NaN joint positions)
    if (!x0.array().isFinite().all())
    {
      sensor_msgs::JointState joint_state_msg;
      joint_state_msg.header.stamp = stamp;
      for (pinocchio::JointIndex i = 2; i < (pinocchio::JointIndex)pin_model->njoints; i++)
      {
        const std::string& joint_name = pin_model->names[i];
        const int q_idx = pin_model->joints[i].idx_q();
        joint_state_msg.name.push_back(joint_name);
        joint_state_msg.position.push_back(x0(q_idx));
      }
      joint_states_pub.publish(joint_state_msg);
    }

    // optimized thrust rates
    if (!us_new.empty())
    {
      std_msgs::Float64MultiArray rate_msg;
      rate_msg.data.resize(rotor_num);
      for (int i = 0; i < rotor_num; ++i)
        rate_msg.data[i] = us_new[0](i);
      thrust_rate_pub.publish(rate_msg);
    }

    // target CoM position
    {
      geometry_msgs::PointStamped pt;
      pt.header.stamp = stamp;
      pt.header.frame_id = "world";
      pt.point.x = com_target(0);
      pt.point.y = com_target(1);
      pt.point.z = com_target(2);
      target_cog_pos_pub.publish(pt);
    }

    // CoM trajectory: line strip of predicted CoM positions + sphere at current CoM.
    // Skip any state that contains NaN to prevent Ogre from crashing in rviz.
    if (!x0.head(nq).array().isFinite().all())
    {
      visualization_msgs::MarkerArray markers;

      // compute current CoM from x0
      pinocchio::centerOfMass(*pin_model, pin_data_local, x0.head(nq), false);
      const Eigen::Vector3d cur_com = pin_data_local.com[0];

      visualization_msgs::Marker line;
      line.header.frame_id = "world";
      line.header.stamp = stamp;
      line.ns = "mpc_com_traj";
      line.id = 0;
      line.type = visualization_msgs::Marker::LINE_STRIP;
      line.action = visualization_msgs::Marker::ADD;
      line.scale.x = 0.01;
      line.color.r = 0.0f;
      line.color.g = 1.0f;
      line.color.b = 0.0f;
      line.color.a = 1.0f;
      for (const auto& x : xs_new)
      {
        if (!x.head(nq).array().isFinite().all())
          continue;
        pinocchio::centerOfMass(*pin_model, pin_data_local, x.head(nq), false);
        geometry_msgs::Point p;
        p.x = pin_data_local.com[0](0);
        p.y = pin_data_local.com[0](1);
        p.z = pin_data_local.com[0](2);
        line.points.push_back(p);
      }
      markers.markers.push_back(line);

      // current CoM as a red sphere
      visualization_msgs::Marker cur;
      cur.header = line.header;
      cur.ns = "mpc_com_traj";
      cur.id = 1;
      cur.type = visualization_msgs::Marker::SPHERE;
      cur.action = visualization_msgs::Marker::ADD;
      cur.pose.position.x = cur_com(0);
      cur.pose.position.y = cur_com(1);
      cur.pose.position.z = cur_com(2);
      cur.pose.orientation.w = 1.0;
      cur.scale.x = cur.scale.y = cur.scale.z = 0.08;
      cur.color.r = 1.0f;
      cur.color.g = 0.0f;
      cur.color.b = 0.0f;
      cur.color.a = 1.0f;
      markers.markers.push_back(cur);

      traj_marker_pub.publish(markers);
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
