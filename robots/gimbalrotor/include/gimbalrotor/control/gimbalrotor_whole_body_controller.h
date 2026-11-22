// -*- mode: c++ -*-
/*
 * Whole-body controller of gimbalrotor with contacts.
 *
 * The CoG and attitude PIDs of PoseLinearController give the wrench the base needs (as in
 * GimbalrotorController); instead of its pseudo-inverse allocation, a QP of the whole-body statics
 * shares that wrench between the thrust of the rotors and the forces of the active contacts:
 *
 *   variables    per rotor (s, t): its thrust in its gimbal plane, t along the most upward direction
 *                of the plane, s along the other one; per active contact, its force (world)
 *   cost         |wrench of the rotors and the contacts - the PID wrench|^2 (weighted, soft) plus a
 *                regularisation of the thrust and of the contact forces
 *   constraints  per rotor: t in [min lift, max thrust], |s| <= tan(max tilt) t
 *                per support contact: inside its friction pyramid, its normal force within
 *                [min_normal, max_normal] when fully loaded; while it is loaded or unloaded, within
 *                the load share of that (see below)
 *                per prescribed contact: its force the one given (scaled by the load share)
 *                per actuated joint: the torque of the statics g(q) - sum J^T f within its limit
 *                (the joints are position controlled: their torque is only bounded)
 *
 * The load share of a contact grows from 0 to 1 over ramp_time after it is switched on (and goes
 * back to 0 when it is switched off). How it bounds the normal force of a support contact (ramp):
 *   relative  (default) share x the normal force the contact would carry fully loaded: that is
 *             first found by the same QP with every support contact fully loaded (the prescribed
 *             ones at their share), then the QP is solved again with these bounds. The load moves
 *             onto (off) the contact linearly over ramp_time whatever the posture and the task, as
 *             the planner (plan_perching_motion.py) moves it
 *   absolute  share x [min_normal, max_normal]: with a max_normal well above the load, the load
 *             moves within the first max_normal / load part of ramp_time
 * (the second QP only runs while a relative contact is ramping; if the first one fails, the
 * absolute bounds are used for that cycle)
 *
 * While a contact carries load, the PIDs of the axes in freeze_integral_in_contact do not integrate
 * (their integral is held: z keeps the share of the weight it has): a constrained axis would drift.
 *
 * The thrust of each rotor goes out as its thrust and gimbal angle (gimbal_calc_in_fc must be
 * false). The contacts, defined in the parameters, are switched by a service each:
 *   <ns>/whole_body/contacts/<name>/activate (std_srvs/SetBool)
 * and their forces of the QP are published on <ns>/whole_body/contacts/<name>/force; the torques of the
 * statics on the limited joints on <ns>/whole_body/joint_torque (sensor_msgs/JointState: effort the
 * torque, position its limit [Nm]).
 * A reset of the controller (the arming, the takeoff, the motors off after the landing) switches
 * every contact off at once.
 * Kinematics and statics: pinocchio, from robot_description and the joint states of the model.
 */

#pragma once

#include <gimbalrotor/control/gimbalrotor_controller.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <mutex>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <std_srvs/SetBool.h>

namespace aerial_robot_control
{
struct WholeBodyContact
{
  std::string name;
  std::string link;
  pinocchio::FrameIndex frame;  // the contact frame: z along the normal (the environment pushing the robot)
  bool prescribed = false;      // a task force instead of a supporting one
  Eigen::Vector3d force;        // prescribed: the force in the contact frame [N]
  double friction = 0.5;
  double min_normal = 0.0;    // [N] support: at least (when fully loaded)
  double max_normal = 50.0;   // [N] support: at most when fully loaded
  double ramp_time = 1.0;     // [s] of the load share from 0 to 1 (and back)
  bool relative_ramp = true;  // support: the share of the load it would carry (else of [min, max]_normal)
  bool active = false;
  ros::Time switched;  // when it was switched on or off
  double share_at_switch = 0.0;
  ros::ServiceServer server;
  ros::Publisher force_pub;
  Eigen::Vector3d last_force = Eigen::Vector3d::Zero();  // world, from the last QP
};

class GimbalrotorWholeBodyController : public GimbalrotorController
{
public:
  GimbalrotorWholeBodyController() = default;
  ~GimbalrotorWholeBodyController() = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate) override;

protected:
  void controlCore() override;
  void reset() override;  // also switches every contact off, at once (no ramp)

private:
  // model
  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<pinocchio::FrameIndex> thrust_frames_;
  std::vector<pinocchio::JointIndex> gimbal_joints_;
  std::vector<Eigen::Vector3d> gimbal_axes_;  // in the gimbal joint frame
  pinocchio::FrameIndex baselink_frame_;
  std::vector<std::pair<int, double>> torque_limits_;  // (velocity index, limit [Nm]) of the actuated joints
  std::vector<std::string> torque_limited_joints_;

  // contacts
  std::vector<WholeBodyContact> contacts_;
  std::mutex contacts_mutex_;
  bool activateContact(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res, size_t index);
  double loadShare(const WholeBodyContact& contact, const ros::Time& now) const;

  // parameters of the QP
  double min_lift_;    // [N] per rotor along up
  double max_tilt_;    // [rad] of the thrust from up
  double thrust_reg_;  // regularisation weights
  double contact_reg_;
  Eigen::VectorXd wrench_weight_;  // 6: force, torque
  double solver_tolerance_;        // ProxQP absolute and relative tolerance
  int solver_max_iteration_;
  std::vector<bool> freeze_integral_;  // per axis X..YAW, while a contact carries load
  ros::Publisher wrench_error_pub_;
  ros::Publisher joint_torque_pub_;  // the torques of the statics on the limited joints (and their limits)

  bool updateModel(Eigen::VectorXd& q);
  bool allocate(const Eigen::VectorXd& q, const Eigen::Matrix<double, 6, 1>& wrench, const ros::Time& now);
};
}  // namespace aerial_robot_control
