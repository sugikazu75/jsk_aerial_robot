// Checks every helper in tf_compat.h against the tf1 function it replaces.
//
// The datatypes rename across cleanly, but the conversion helpers are written
// out by hand rather than delegating to tf2's toMsg/fromMsg, whose overload set
// varies between distributions. Hand-written conversions are exactly the kind of
// thing that silently transposes a rotation or swaps a quaternion's storage
// order, so each one is compared against its original over a sweep of the input
// range. ROS1-only: tf1 is what it checks against.

#include <aerial_robot_ros_compat/tf_compat.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>

#include <cmath>
#include <cstdio>

namespace
{

int failures = 0;

void check(const char* what, double diff, double tol = 0.0)
{
  if (!(std::fabs(diff) <= tol) || std::isnan(diff))
  {
    std::printf("  MISMATCH %-34s diff=%.3e\n", what, diff);
    ++failures;
  }
}

double maxDiff(const tf2::Vector3& a, const tf::Vector3& b)
{
  return std::max({ std::fabs(a.x() - b.x()), std::fabs(a.y() - b.y()), std::fabs(a.z() - b.z()) });
}

double maxDiff(const tf2::Quaternion& a, const tf::Quaternion& b)
{
  return std::max(
      { std::fabs(a.x() - b.x()), std::fabs(a.y() - b.y()), std::fabs(a.z() - b.z()), std::fabs(a.w() - b.w()) });
}

double maxDiff(const tf2::Transform& a, const tf::Transform& b)
{
  double d = maxDiff(a.getOrigin(), b.getOrigin());
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      d = std::max(d, std::fabs(a.getBasis()[i][j] - b.getBasis()[i][j]));
  return d;
}

}  // namespace

int main()
{
  for (int t = 0; t < 500; ++t)
  {
    const double r = (t % 37) * 0.17 - 3.0;
    const double p = (t % 53) * 0.11 - 2.5;
    const double y = (t % 71) * 0.09 - 3.1;
    const double vx = r * 1.3, vy = p - 2.0, vz = y * 0.5;

    // Vector3 <-> Vector3 / Point message
    {
      tf2::Vector3 v2(vx, vy, vz);
      tf::Vector3 v1(vx, vy, vz);
      geometry_msgs::Vector3 m2, m1;
      ros_compat::vector3TfToMsg(v2, m2);
      tf::vector3TFToMsg(v1, m1);
      check("vector3TFToMsg", std::max({ std::fabs(m2.x - m1.x), std::fabs(m2.y - m1.y), std::fabs(m2.z - m1.z) }));

      tf2::Vector3 back2;
      tf::Vector3 back1;
      ros_compat::vector3MsgToTf(m2, back2);
      tf::vector3MsgToTF(m1, back1);
      check("vector3MsgToTF", maxDiff(back2, back1));

      geometry_msgs::Point p2, p1;
      ros_compat::pointTfToMsg(v2, p2);
      tf::pointTFToMsg(v1, p1);
      check("pointTFToMsg", std::max({ std::fabs(p2.x - p1.x), std::fabs(p2.y - p1.y), std::fabs(p2.z - p1.z) }));

      ros_compat::pointMsgToTf(p2, back2);
      tf::pointMsgToTF(p1, back1);
      check("pointMsgToTF", maxDiff(back2, back1));
    }

    // Quaternion
    {
      tf2::Quaternion q2;
      q2.setRPY(r, p, y);
      tf::Quaternion q1;
      q1.setRPY(r, p, y);
      check("Quaternion::setRPY", maxDiff(q2, q1));

      geometry_msgs::Quaternion m2, m1;
      ros_compat::quaternionTfToMsg(q2, m2);
      tf::quaternionTFToMsg(q1, m1);
      check("quaternionTFToMsg", std::max({ std::fabs(m2.x - m1.x), std::fabs(m2.y - m1.y), std::fabs(m2.z - m1.z),
                                            std::fabs(m2.w - m1.w) }));

      tf2::Quaternion back2;
      tf::Quaternion back1;
      ros_compat::quaternionMsgToTf(m2, back2);
      tf::quaternionMsgToTF(m1, back1);
      check("quaternionMsgToTF", maxDiff(back2, back1));

      check("createQuaternionFromYaw", maxDiff(ros_compat::createQuaternionFromYaw(y), tf::createQuaternionFromYaw(y)));

      geometry_msgs::Quaternion c2 = ros_compat::createQuaternionMsgFromRollPitchYaw(r, p, y);
      geometry_msgs::Quaternion c1 = tf::createQuaternionMsgFromRollPitchYaw(r, p, y);
      check("createQuaternionMsgFromRPY", std::max({ std::fabs(c2.x - c1.x), std::fabs(c2.y - c1.y),
                                                     std::fabs(c2.z - c1.z), std::fabs(c2.w - c1.w) }));
    }

    // Transform / Pose
    {
      geometry_msgs::Pose pose;
      pose.position.x = vx;
      pose.position.y = vy;
      pose.position.z = vz;
      pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, y);

      tf2::Transform tr2;
      tf::Transform tr1;
      ros_compat::poseMsgToTf(pose, tr2);
      tf::poseMsgToTF(pose, tr1);
      check("poseMsgToTF", maxDiff(tr2, tr1), 1e-15);

      geometry_msgs::Transform m2, m1;
      ros_compat::transformTfToMsg(tr2, m2);
      tf::transformTFToMsg(tr1, m1);
      check("transformTFToMsg",
            std::max({ std::fabs(m2.translation.x - m1.translation.x), std::fabs(m2.rotation.x - m1.rotation.x),
                       std::fabs(m2.rotation.w - m1.rotation.w) }),
            1e-15);
    }

    // KDL
    {
      KDL::Frame f(KDL::Rotation::RPY(r, p, y), KDL::Vector(vx, vy, vz));

      tf2::Transform tr2;
      tf::Transform tr1;
      ros_compat::transformKdlToTf(f, tr2);
      tf::transformKDLToTF(f, tr1);
      check("transformKDLToTF", maxDiff(tr2, tr1), 1e-15);

      tf2::Quaternion q2;
      tf::Quaternion q1;
      ros_compat::quaternionKdlToTf(f.M, q2);
      tf::quaternionKDLToTF(f.M, q1);
      check("quaternionKDLToTF", maxDiff(q2, q1));

      KDL::Rotation r2, r1;
      ros_compat::quaternionTfToKdl(q2, r2);
      tf::quaternionTFToKDL(q1, r1);
      double d = 0;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          d = std::max(d, std::fabs(r2(i, j) - r1(i, j)));
      check("quaternionTFToKDL", d, 1e-15);
    }

    // Eigen conversions
    {
      tf2::Vector3 v2(vx, vy, vz);
      tf::Vector3 v1(vx, vy, vz);
      Eigen::Vector3d e2, e1;
      ros_compat::vectorTfToEigen(v2, e2);
      tf::vectorTFToEigen(v1, e1);
      check("vectorTFToEigen", (e2 - e1).cwiseAbs().maxCoeff());

      tf2::Vector3 bt2;
      tf::Vector3 bt1;
      ros_compat::vectorEigenToTf(e2, bt2);
      tf::vectorEigenToTF(e1, bt1);
      check("vectorEigenToTF", maxDiff(bt2, bt1));

      tf2::Quaternion q2;
      q2.setRPY(r, p, y);
      tf::Quaternion q1;
      q1.setRPY(r, p, y);
      Eigen::Matrix3d m2, m1;
      ros_compat::matrixTfToEigen(tf2::Matrix3x3(q2), m2);
      tf::matrixTFToEigen(tf::Matrix3x3(q1), m1);
      check("matrixTFToEigen", (m2 - m1).cwiseAbs().maxCoeff());

      geometry_msgs::Vector3 mv2, mv1;
      ros_compat::vectorEigenToMsg(e2, mv2);
      tf::vectorEigenToMsg(e1, mv1);
      check("vectorEigenToMsg",
            std::max({ std::fabs(mv2.x - mv1.x), std::fabs(mv2.y - mv1.y), std::fabs(mv2.z - mv1.z) }));

      Eigen::Vector3d be2, be1;
      ros_compat::vectorMsgToEigen(mv2, be2);
      tf::vectorMsgToEigen(mv1, be1);
      check("vectorMsgToEigen", (be2 - be1).cwiseAbs().maxCoeff());

      geometry_msgs::Point mp2, mp1;
      ros_compat::pointEigenToMsg(e2, mp2);
      tf::pointEigenToMsg(e1, mp1);
      check("pointEigenToMsg",
            std::max({ std::fabs(mp2.x - mp1.x), std::fabs(mp2.y - mp1.y), std::fabs(mp2.z - mp1.z) }));

      ros_compat::pointMsgToEigen(mp2, be2);
      tf::pointMsgToEigen(mp1, be1);
      check("pointMsgToEigen", (be2 - be1).cwiseAbs().maxCoeff());

      Eigen::Quaterniond eq2(Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()));
      geometry_msgs::Quaternion mq2, mq1;
      ros_compat::quaternionEigenToMsg(eq2, mq2);
      tf::quaternionEigenToMsg(eq2, mq1);
      check("quaternionEigenToMsg", std::max({ std::fabs(mq2.x - mq1.x), std::fabs(mq2.y - mq1.y),
                                               std::fabs(mq2.z - mq1.z), std::fabs(mq2.w - mq1.w) }));

      Eigen::Quaterniond bq2, bq1;
      ros_compat::quaternionMsgToEigen(mq2, bq2);
      tf::quaternionMsgToEigen(mq1, bq1);
      check("quaternionMsgToEigen", std::max({ std::fabs(bq2.x() - bq1.x()), std::fabs(bq2.y() - bq1.y()),
                                               std::fabs(bq2.z() - bq1.z()), std::fabs(bq2.w() - bq1.w()) }));

      geometry_msgs::Wrench w;
      w.force.x = vx;
      w.force.y = vy;
      w.force.z = vz;
      w.torque.x = r;
      w.torque.y = p;
      w.torque.z = y;
      Eigen::Matrix<double, 6, 1> we2, we1;
      ros_compat::wrenchMsgToEigen(w, we2);
      tf::wrenchMsgToEigen(w, we1);
      check("wrenchMsgToEigen", (we2 - we1).cwiseAbs().maxCoeff());
    }
  }

  if (failures == 0)
    std::printf("tf_compat: all helpers match their tf1 originals over 500 orientations\n");
  else
    std::printf("tf_compat: %d MISMATCHES\n", failures);
  return failures == 0 ? 0 : 1;
}
