#include "aerial_robot_base/aerial_robot_base.h"

int main (int argc, char **argv)
{
  ros_compat::NodeHandle nh = ros_compat::initNode(argc, argv, "aeria_robot_base");
  ros_compat::NodeHandle nh_private = ros_compat::privateNodeHandle(nh);
  AerialRobotBase*  aerialRobotBaseNode = new AerialRobotBase(nh, nh_private);
  ros_compat::waitForShutdown();

  delete aerialRobotBaseNode;
  return 0;
}





