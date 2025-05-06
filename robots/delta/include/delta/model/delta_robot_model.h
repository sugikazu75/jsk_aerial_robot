// -*- mode: c++ -*-

#pragma once

#include <ros/ros.h>

#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <geometry_msgs/PoseArray.h>
#include <numeric>
#include <queue>
#include <std_msgs/Float32.h>

using namespace aerial_robot_model;

class RollingRobotModel : public aerial_robot_model::transformable::RobotModel {
public:
  RollingRobotModel(bool init_with_rosparam = true,
                    bool verbose = false,
                    double fc_f_min_thre = 0,
                    double fc_t_min_thre = 0,
                    double epsilon = 10);
  ~RollingRobotModel()
  {
    contact_point_calc_thread_.interrupt();
    contact_point_calc_thread_.join();
  }

  template <class T> T getContactPoint();
  template <class T> T getSecondContactPoint();
  template <class T> std::vector<T> getLinksRotationFromCog();
  template <class T> std::vector<T> getLinksRotationFromControlFrame();
  template <class T> std::vector<T> getRotorsOriginFromControlFrame();
  template <class T> std::vector<T> getRotorsNormalFromControlFrame();
  std::vector<KDL::Frame> getLinksCenterFrameFromCog()
  {
    std::lock_guard<std::mutex> lock(links_center_frame_mutex_);
    return links_center_frame_from_cog_;
  }
  void setCircleRadius(double radius) {circle_radius_ = radius;}
  double getCircleRadius() {return circle_radius_;}
  void calcContactPoint();
  int getContactingLink() {return contacting_link_;}
  const std::vector<double>& getContactingAnglesInLinks() {return contacting_angles_in_links_;}
  void setNominalContactPointFlag(bool flag) {nominal_contact_point_flag_ = flag;}
  bool getNonimalContactPointFlag() {return nominal_contact_point_flag_;}
  void setCommandedContactingLinkIndex(int index) {commanded_contacting_link_index_ = index;}
  int getCommandedContactingLinkIndex() {return commanded_contacting_link_index_;}
  void setControlFrame(KDL::Frame frame){control_frame_ = frame;};
  void setControlFrame(std::string frame_name);
  KDL::Frame getControlFrame(){return control_frame_;}
  std::string getControlFrameName() {return control_frame_name_;}
  Eigen::MatrixXd getFullWrenchAllocationMatrixFromControlFrame();
  Eigen::MatrixXd getFullWrenchAllocationMatrixFromControlFrame(std::string frame_name);
  Eigen::MatrixXd getPlannedWrenchAllocationMatrixFromControlFrame();
  const std::vector<double>& getCurrentJointAngles() {return current_joint_angles_;}
  const std::vector<double>& getCurrentGimbalAngles() {return current_gimbal_angles_;}
  void setGimbalPlanningFlag(int index, int flag) {gimbal_planning_flag_.at(index) = flag;}
  std::vector<int> getGimbalPlanningFlag() {return gimbal_planning_flag_;}
  void setCurrentGimbalPlanningAngle(int index, double angle) {current_gimbal_planning_angle_.at(index) = angle;}
  const std::vector<double>& getCurrentGimbalPlanningAngle() {return current_gimbal_planning_angle_;}
  void setFinalGimbalPlanningAngle(int index, double angle) {final_gimbal_planning_angle_.at(index) = angle;}
  const std::vector<double>& getFinalGimbalPlanningAngle() {return final_gimbal_planning_angle_;}
  template <class T> T getInertiaFromControlFrame();

private:
  boost::thread contact_point_calc_thread_;

  std::mutex links_rotation_mutex_;
  std::mutex links_center_frame_mutex_;
  std::mutex rotors_normal_control_frame_mutex_;
  std::mutex rotors_origin_control_frame_mutex_;
  std::mutex contact_point_mutex_;
  std::mutex current_joint_angles_mutex_;
  std::mutex current_gimbal_angles_mutex_;

  bool nominal_contact_point_flag_;
  int commanded_contacting_link_index_;
  KDL::Frame contact_point_;
  KDL::Frame second_contact_point_;
  int contacting_link_;
  std::vector<double> contacting_angles_in_links_;
  std::vector<int> lowest_point_ranks_;

  std::string thrust_link_;
  double circle_radius_;
  KDL::RigidBodyInertia link_inertia_; // from root
  KDL::Frame control_frame_;
  std::string control_frame_name_;
  std::map<std::string, KDL::Frame*> additional_frame_;

  std::vector<KDL::Rotation> links_rotation_from_cog_;
  std::vector<KDL::Rotation> links_rotation_from_control_frame_;
  std::vector<KDL::Vector> rotors_origin_from_control_frame_;
  std::vector<KDL::Vector> rotors_normal_from_control_frame_;
  std::vector<KDL::Frame> links_center_frame_from_cog_;
  std::vector<double> current_joint_angles_;
  std::vector<double> current_gimbal_angles_;

  /* gimbal planning */
  std::vector<int> gimbal_planning_flag_;
  std::vector<double> current_gimbal_planning_angle_;
  std::vector<double> final_gimbal_planning_angle_;

protected:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;

  void setContactPoint(KDL::Frame contact_point)
  {
    std::lock_guard<std::mutex> lock(contact_point_mutex_);
    contact_point_ = contact_point;
  }
  void setSecondContactPoint(KDL::Frame second_contact_point)
  {
    std::lock_guard<std::mutex> lock(contact_point_mutex_);
    second_contact_point_ = second_contact_point;
  }
};

template<> inline std::vector<KDL::Rotation> RollingRobotModel::getLinksRotationFromCog()
{
  std::lock_guard<std::mutex> lock(links_rotation_mutex_);
  return links_rotation_from_cog_;
}

template<> inline std::vector<Eigen::Matrix3d> RollingRobotModel::getLinksRotationFromCog()
{
  return aerial_robot_model::kdlToEigen(getLinksRotationFromCog<KDL::Rotation>());
}

template<> inline std::vector<KDL::Rotation> RollingRobotModel::getLinksRotationFromControlFrame()
{
  std::lock_guard<std::mutex> lock(links_rotation_mutex_);
  return links_rotation_from_control_frame_;
}

template<> inline std::vector<Eigen::Matrix3d> RollingRobotModel::getLinksRotationFromControlFrame()
{
  return aerial_robot_model::kdlToEigen(getLinksRotationFromControlFrame<KDL::Rotation>());
}

template<> inline std::vector<KDL::Vector> RollingRobotModel::getRotorsNormalFromControlFrame()
{
  std::lock_guard<std::mutex> lock(rotors_normal_control_frame_mutex_);
  return rotors_normal_from_control_frame_;
}

template<> inline std::vector<Eigen::Vector3d> RollingRobotModel::getRotorsNormalFromControlFrame()
{
  return aerial_robot_model::kdlToEigen(RollingRobotModel::getRotorsNormalFromControlFrame<KDL::Vector>());
}

template<> inline std::vector<KDL::Vector> RollingRobotModel::getRotorsOriginFromControlFrame()
{
  std::lock_guard<std::mutex> lock(rotors_origin_control_frame_mutex_);
  return rotors_origin_from_control_frame_;
}

template<> inline std::vector<Eigen::Vector3d> RollingRobotModel::getRotorsOriginFromControlFrame()
{
  return aerial_robot_model::kdlToEigen(RollingRobotModel::getRotorsOriginFromControlFrame<KDL::Vector>());
}

template<> inline KDL::Frame RollingRobotModel::getContactPoint()
{
  std::lock_guard<std::mutex> lock(contact_point_mutex_);
  return contact_point_;
}

template<> inline geometry_msgs::TransformStamped RollingRobotModel::getContactPoint()
{
  return aerial_robot_model::kdlToMsg(RollingRobotModel::getContactPoint<KDL::Frame>());
}

template<> inline KDL::Frame RollingRobotModel::getSecondContactPoint()
{
  std::lock_guard<std::mutex> lock(contact_point_mutex_);
  return second_contact_point_;
}

template<> inline geometry_msgs::TransformStamped RollingRobotModel::getSecondContactPoint()
{
  return aerial_robot_model::kdlToMsg(RollingRobotModel::getSecondContactPoint<KDL::Frame>());
}

template<> inline KDL::RotationalInertia RollingRobotModel::getInertiaFromControlFrame()
{
  KDL::Frame frame = getControlFrame();
  return (frame.Inverse() * link_inertia_).getRotationalInertia();
}

template<> inline Eigen::Matrix3d RollingRobotModel::getInertiaFromControlFrame()
{
  return aerial_robot_model::kdlToEigen(RollingRobotModel::getInertiaFromControlFrame<KDL::RotationalInertia>());
}
