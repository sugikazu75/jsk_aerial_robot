#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <ros/ros.h>

#include <crocoddyl/core/diff-action-base.hpp>
#include <crocoddyl/core/state-base.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/floating-base-thrusters.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>

#include <vector>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "crocoddyl_test");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::string fileName = "/home/leus/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/mini_quadrotor/urdf/robot.urdf";
  
  boost::shared_ptr<pinocchio::Model> model;
  model = boost::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(fileName, pinocchio::JointModelFreeFlyer(), *model.get(), true);

  ROS_WARN_STREAM("[model][pinocchio] model name: " << model->name);
  ROS_WARN_STREAM("[model][pinocchio] model.nq: " << model->nq);
  ROS_WARN_STREAM("[model][pinocchio] model.nv: " << model->nv);
  ROS_WARN_STREAM("[model][pinocchio] model.njoints: " << model->njoints);

  std::vector<crocoddyl::Thruster> thrusters;
  double rotor_x = 0.085;
  double rotor_z = 0.026;
  double m_f_rate = 0.011;
  pinocchio::SE3 p1(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-rotor_x, -rotor_x, rotor_z));
  pinocchio::SE3 p2(Eigen::Matrix3d::Identity(), Eigen::Vector3d(rotor_x, -rotor_x, rotor_z));
  pinocchio::SE3 p3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(rotor_x, rotor_x, rotor_z));
  pinocchio::SE3 p4(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-rotor_x, rotor_x, rotor_z));
  thrusters.push_back(crocoddyl::Thruster(p1, m_f_rate, crocoddyl::ThrusterType::CCW));
  thrusters.push_back(crocoddyl::Thruster(p2, m_f_rate, crocoddyl::ThrusterType::CW));
  thrusters.push_back(crocoddyl::Thruster(p3, m_f_rate, crocoddyl::ThrusterType::CW));
  thrusters.push_back(crocoddyl::Thruster(p4, m_f_rate, crocoddyl::ThrusterType::CCW));

  boost::shared_ptr<crocoddyl::StateMultibody> state = boost::make_shared<crocoddyl::StateMultibody>(model);
  boost::shared_ptr<crocoddyl::ActuationModelFloatingBaseThrusters> actuation = boost::make_shared<crocoddyl::ActuationModelFloatingBaseThrusters>(state, thrusters);


}
