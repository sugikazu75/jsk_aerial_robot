// -*- mode: c++ -*-
//
// tf1 replacements built on tf2.
//
// ROS2 has no `tf` package at all, so every tf:: name has to go. The datatypes
// are the easy part: tf::Vector3, tf::Matrix3x3, tf::Quaternion and
// tf::Transform are copies of the same bullet-derived code that tf2 carries, so
// they rename straight across to tf2:: and behave identically. Note they are
// separate classes, not aliases - a tf::Vector3 will not bind to a tf2::Vector3
// - which is why the rename has to happen everywhere at once.
//
// The conversion helpers are the awkward part. tf2 does offer toMsg/fromMsg
// overloads, but which types they cover differs between distributions, and
// picking the wrong overload silently converts through the wrong intermediate.
// These are written out by hand instead: a handful of field copies each, no
// dependency on an overload set that may or may not be there, and the same
// spelling under both ROS versions.
//
// Every function here is checked against its tf1 original in
// test/tf_compat_test.cpp.

#pragma once

#include <aerial_robot_ros_compat/message.h>
#include <aerial_robot_ros_compat/ros_compat.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <kdl/frames.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#if AERIAL_ROBOT_ROS_VERSION == 1
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#else
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#endif
AERIAL_ROBOT_MSG_NAMESPACE(geometry_msgs);

namespace ros_compat
{

// ---- Vector3 ---------------------------------------------------------------

/** tf::vector3TFToMsg */
inline void vector3TfToMsg(const tf2::Vector3& in, geometry_msgs_c::Vector3& out)
{
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
}

/** tf::vector3MsgToTF */
inline void vector3MsgToTf(const geometry_msgs_c::Vector3& in, tf2::Vector3& out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}

/** tf::pointTFToMsg */
inline void pointTfToMsg(const tf2::Vector3& in, geometry_msgs_c::Point& out)
{
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
}

/** tf::pointMsgToTF */
inline void pointMsgToTf(const geometry_msgs_c::Point& in, tf2::Vector3& out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}

// ---- Quaternion ------------------------------------------------------------

/** tf::quaternionTFToMsg */
inline void quaternionTfToMsg(const tf2::Quaternion& in, geometry_msgs_c::Quaternion& out)
{
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
  out.w = in.w();
}

/** tf::quaternionMsgToTF */
inline void quaternionMsgToTf(const geometry_msgs_c::Quaternion& in, tf2::Quaternion& out)
{
  out = tf2::Quaternion(in.x, in.y, in.z, in.w);
}

/** tf::createQuaternionFromYaw */
inline tf2::Quaternion createQuaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return q;
}

/** tf::createQuaternionMsgFromRollPitchYaw */
inline geometry_msgs_c::Quaternion createQuaternionMsgFromRollPitchYaw(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs_c::Quaternion msg;
  quaternionTfToMsg(q, msg);
  return msg;
}

// ---- Transform / Pose ------------------------------------------------------

/** tf::poseMsgToTF */
inline void poseMsgToTf(const geometry_msgs_c::Pose& in, tf2::Transform& out)
{
  tf2::Quaternion q;
  quaternionMsgToTf(in.orientation, q);
  // A pose carrying an all-zero quaternion is not a rotation. tf1 normalised
  // whatever it was given, which turns that case into a NaN; treat it as
  // identity so a default-constructed message does not poison the estimate.
  if (q.length2() == 0.0)
    q = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
  out = tf2::Transform(q, tf2::Vector3(in.position.x, in.position.y, in.position.z));
}

/** tf::transformTFToMsg */
inline void transformTfToMsg(const tf2::Transform& in, geometry_msgs_c::Transform& out)
{
  vector3TfToMsg(in.getOrigin(), out.translation);
  quaternionTfToMsg(in.getRotation(), out.rotation);
}

// ---- KDL -------------------------------------------------------------------

/** tf::transformKDLToTF */
inline void transformKdlToTf(const KDL::Frame& in, tf2::Transform& out)
{
  double x, y, z, w;
  in.M.GetQuaternion(x, y, z, w);
  out = tf2::Transform(tf2::Quaternion(x, y, z, w), tf2::Vector3(in.p.x(), in.p.y(), in.p.z()));
}

/** tf::quaternionKDLToTF */
inline void quaternionKdlToTf(const KDL::Rotation& in, tf2::Quaternion& out)
{
  double x, y, z, w;
  in.GetQuaternion(x, y, z, w);
  out = tf2::Quaternion(x, y, z, w);
}

/** tf::quaternionTFToKDL */
inline void quaternionTfToKdl(const tf2::Quaternion& in, KDL::Rotation& out)
{
  out = KDL::Rotation::Quaternion(in.x(), in.y(), in.z(), in.w());
}

/** tf::quaternionMsgToKDL */
inline void quaternionMsgToKdl(const geometry_msgs_c::Quaternion& in, KDL::Rotation& out)
{
  out = KDL::Rotation::Quaternion(in.x, in.y, in.z, in.w);
}

/** tf::pointMsgToKDL */
inline void pointMsgToKdl(const geometry_msgs_c::Point& in, KDL::Vector& out)
{
  out = KDL::Vector(in.x, in.y, in.z);
}

// ---- Eigen -----------------------------------------------------------------
//
// These came from tf_conversions and eigen_conversions, neither of which exists
// under ROS2. tf2_eigen covers some of the ground but only for the message
// types, not for the tf datatypes.

/** tf::vectorTFToEigen */
inline void vectorTfToEigen(const tf2::Vector3& in, Eigen::Vector3d& out)
{
  out = Eigen::Vector3d(in.x(), in.y(), in.z());
}

/** tf::vectorEigenToTF */
inline void vectorEigenToTf(const Eigen::Vector3d& in, tf2::Vector3& out)
{
  out = tf2::Vector3(in.x(), in.y(), in.z());
}

/** tf::matrixTFToEigen */
inline void matrixTfToEigen(const tf2::Matrix3x3& in, Eigen::Matrix3d& out)
{
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      out(i, j) = in[i][j];
}

/** tf::vectorEigenToMsg */
inline void vectorEigenToMsg(const Eigen::Vector3d& in, geometry_msgs_c::Vector3& out)
{
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
}

/** tf::vectorMsgToEigen */
inline void vectorMsgToEigen(const geometry_msgs_c::Vector3& in, Eigen::Vector3d& out)
{
  out = Eigen::Vector3d(in.x, in.y, in.z);
}

/** tf::pointEigenToMsg */
inline void pointEigenToMsg(const Eigen::Vector3d& in, geometry_msgs_c::Point& out)
{
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
}

/** tf::pointMsgToEigen */
inline void pointMsgToEigen(const geometry_msgs_c::Point& in, Eigen::Vector3d& out)
{
  out = Eigen::Vector3d(in.x, in.y, in.z);
}

/** tf::quaternionEigenToMsg */
inline void quaternionEigenToMsg(const Eigen::Quaterniond& in, geometry_msgs_c::Quaternion& out)
{
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
  out.w = in.w();
}

/** tf::quaternionMsgToEigen */
inline void quaternionMsgToEigen(const geometry_msgs_c::Quaternion& in, Eigen::Quaterniond& out)
{
  // Eigen's constructor takes w first; the message stores x,y,z,w. Getting this
  // the wrong way round is silent and rotates everything incorrectly.
  out = Eigen::Quaterniond(in.w, in.x, in.y, in.z);
}

/** tf::wrenchMsgToEigen */
inline void wrenchMsgToEigen(const geometry_msgs_c::Wrench& in, Eigen::Matrix<double, 6, 1>& out)
{
  out[0] = in.force.x;
  out[1] = in.force.y;
  out[2] = in.force.z;
  out[3] = in.torque.x;
  out[4] = in.torque.y;
  out[5] = in.torque.z;
}

}  // namespace ros_compat
