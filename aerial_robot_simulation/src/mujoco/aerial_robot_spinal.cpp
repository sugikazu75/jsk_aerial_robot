#include <aerial_robot_simulation/mujoco/aerial_robot_spinal.h>

#include <aerial_robot_estimation/state_estimation.h>

namespace aerial_robot_simulation
{

bool AerialRobotSpinal::init(ros_compat::NodeHandle nh, int motor_num)
{
  nh_ = nh;
  motor_num_ = motor_num;
  force_.assign(motor_num_, 0.0);

  spinal_state_estimator_.init(&nh_);
  controller_core_.init(&nh_, &spinal_state_estimator_);

  return true;
}

void AerialRobotSpinal::setImuValue(double acc_x, double acc_y, double acc_z, double gyro_x, double gyro_y,
                                    double gyro_z)
{
  spinal_state_estimator_.getAttEstimator()->setAcc(acc_x, acc_y, acc_z);
  spinal_state_estimator_.getAttEstimator()->setGyro(gyro_x, gyro_y, gyro_z);
}

void AerialRobotSpinal::setMagValue(double mag_x, double mag_y, double mag_z)
{
  spinal_state_estimator_.getAttEstimator()->setMag(mag_x, mag_y, mag_z);
}

void AerialRobotSpinal::stateEstimate()
{
  if (on_ground_)
  {
    /* assume the robot is static, acc: [0, 0, g] */
    setImuValue(0, 0, aerial_robot_estimation::G, 0, 0, 0);
  }

  spinal_state_estimator_.update();
}

void AerialRobotSpinal::setGroundTruthStates(double q_x, double q_y, double q_z, double q_w, double w_x, double w_y,
                                             double w_z)
{
  /* directly overwrite the state in attitude estimation */
  ap::Quaternion q(q_w, q_x, q_y, q_z);
  ap::Matrix3f rot;
  q.rotation_matrix(rot);
  ap::Vector3f ang_vel(w_x, w_y, w_z);
  spinal_state_estimator_.getAttEstimator()->setGroundTruthStates(rot, ang_vel);
}

void AerialRobotSpinal::update()
{
  /* Freeze the attitude estimator while touching the ground: contact simulation
     is poor enough that integrating through it corrupts the estimate. The ROS1
     controller pushed this flag into the hardware; here it is one object. */
  onGround(!controller_core_.getAttController().getIntegrateFlag());

  controller_core_.update();

  for (int i = 0; i < motor_num_; ++i)
  {
    force_[i] = controller_core_.getAttController().getForce(i);
  }
}

}  // namespace aerial_robot_simulation
