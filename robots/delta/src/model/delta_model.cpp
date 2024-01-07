#include <delta/model/delta_model.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "delta_model");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  std::string model_path = "/home/sugihara/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/dragon/robots/quad/euclid_201709.urdf.urdf";
  // Load the URDF model
  pinocchio::Model model;
  pinocchio::urdf::buildModel(model_path, model);
  std::cout << "model name: " << model.name << std::endl;

  // Load the URDF model
  pinocchio::Data data(model);

  double start_t = ros::Time::now().toSec();

  // Sample a random joint configuration as well as random joint velocity and acceleration
  std::cout << "model nv: " << model.nv << std::endl;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(randomConfiguration(model).size());
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

  // Perform the forward kinematics over the kinematic tree
  forwardKinematics(model,data,q);

  std::cout << "\nJoint placements:" << std::endl;
  for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model.njoints; ++joint_id)
    std::cout << std::setw(24) << std::left
              << model.names[joint_id] << ": "
              << std::fixed << std::setprecision(4)
              << data.oMi[joint_id].translation().transpose()
              << std::endl;

  // Computes the kinematics derivatives for all the joints of the robot
  pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);

  // Retrieve the kinematics derivatives of a specific joint, expressed in the LOCAL frame of thise joints.
  pinocchio::JointIndex joint_id = (pinocchio::JointIndex)(model.njoints-1);
  pinocchio::Data::Matrix6x v_partial_dq(6, model.nv), a_partial_dq(6, model.nv), a_partial_dv(6, model.nv), a_partial_da(6, model.nv);
  v_partial_dq.setZero();
  a_partial_dq.setZero(); a_partial_dv.setZero(); a_partial_da.setZero();
  pinocchio::getJointAccelerationDerivatives(model,data,joint_id,pinocchio::LOCAL,v_partial_dq,
                                             a_partial_dq,a_partial_dv,a_partial_da);

  // Remark: we are not directly computing the quantity v_partial_dv as it is also equal to a_partial_da.

  // But we can also expressed the same quantities in the frame centered on the end-effector joint, but expressed in the axis aligned with the world frame.
  pinocchio::getJointAccelerationDerivatives(model,data,joint_id, pinocchio::WORLD,v_partial_dq,
                                             a_partial_dq,a_partial_dv,a_partial_da);

  double finish_t = ros::Time::now().toSec();
  std::cout << "finish - start: " << finish_t - start_t << std::endl;

  std::cout << q << std::endl;
  std::cout << std::endl;
  std::cout << v_partial_dq << std::endl;
  std::cout << std::endl;
  std::cout << a_partial_dq << std::endl;
  std::cout << std::endl;
  std::cout << a_partial_dv << std::endl;
  std::cout << std::endl;
  std::cout << a_partial_da << std::endl;

  ros::spin();
  return 0;

}
