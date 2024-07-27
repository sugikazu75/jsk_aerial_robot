#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "crocoddyl_tennis");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::string fileName = "/home/leus/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/dragon/robots/quad/robot.urdf";

  boost::shared_ptr<pinocchio::Model> model;
  model = boost::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(fileName, pinocchio::JointModelFreeFlyer(), *model.get(), true);
}
