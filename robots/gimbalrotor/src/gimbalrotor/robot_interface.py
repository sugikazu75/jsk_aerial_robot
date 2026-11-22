#!/usr/bin/env python3
"""RobotInterface of aerial_robot_base for gimbalrotor.

  * The servos of gimbalrotor are in groups (config/<airframe>/Servo.yaml): servo_bridge takes the
    targets of a group on <group>_ctrl, not joints_ctrl. The joints of the arms are the group
    "arm_joints"; the gimbals ("gimbals") are driven by the flight controller and are not sent.
  * The body attitude goes as a quaternion to <ns>/final_target_baselink_rot (geometry_msgs/
    QuaternionStamped): the navigator of gimbalrotor turns the desired CoG frame to it (rate limited by
    navigation/baselink_rot_change_thresh per baselink_rot_pub_interval), and sends it to spinal as
    desire_coordinate. The CoG position goes to uav/nav, with the yaw and the roll and the pitch of
    the navigator at 0 (relative to that frame): sendCogTarget.
"""

import numpy as np
import rospy
from aerial_robot_base.robot_interface import RobotInterface
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import JointState


def heading(q):
    """The rotation about the world z of a quaternion [x, y, z, w] (its twist about z): level, at the
    same heading. Identity if undefined (upside down)."""
    twist = np.array([0.0, 0.0, q[2], q[3]], dtype=float)
    norm = np.linalg.norm(twist)
    return twist / norm if norm > 1e-6 else np.array([0.0, 0.0, 0.0, 1.0])


class GimbalrotorInterface(RobotInterface):
    def __init__(self, robot_ns="", debug_view=False, init_node=False, joint_group="arm_joints"):
        super(GimbalrotorInterface, self).__init__(robot_ns, debug_view, init_node)
        self.joint_ctrl_pub = rospy.Publisher(self.robot_ns + "/" + joint_group + "_ctrl", JointState, queue_size=1)
        self._wait_for_subscriber(self.joint_ctrl_pub)
        self.final_rot_quat_pub = rospy.Publisher(
            self.robot_ns + "/final_target_baselink_rot", QuaternionStamped, queue_size=1
        )
        self._wait_for_subscriber(self.final_rot_quat_pub)
        # the yaw handed over from the target yaw of the navigator to the desired CoG frame
        self._yaw_on_desire = False

    @staticmethod
    def _wait_for_subscriber(publisher, timeout=2.0):
        """A message published right after its publisher is made can be lost: wait for servo_bridge."""
        deadline = rospy.get_time() + timeout
        while not rospy.is_shutdown() and publisher.get_num_connections() == 0 and rospy.get_time() < deadline:
            rospy.sleep(0.05)
        if publisher.get_num_connections() == 0:
            rospy.logwarn("no subscriber on %s", publisher.resolved_name)

    def sendJointTarget(self, names, angles):
        """Send joint angles (no motion check, for streaming; setJointAngle waits for them)."""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(names)
        msg.position = [float(a) for a in angles]  # no effort: servo_bridge would send it as torque
        self.joint_ctrl_pub.publish(msg)

    def sendCogTarget(self, pos, vel=None, rot=None):
        """One target of the CoG position and velocity (POS_VEL_MODE), and of the body attitude: rot, the
        quaternion [x, y, z, w] of the baselink in the world (None: level at the current heading).

        The attitude is the desired CoG frame (final_target_baselink_rot -> desire_coordinate); the
        yaw, the roll and the pitch of the navigator are kept at 0. At the first target in a hover the
        attitude goes first: while the target yaw of the navigator is not 0, the navigator starts the
        desired frame from that yaw, which then goes to 0 without a jump."""
        if self.flight_state != self.HOVER_STATE:
            self._yaw_on_desire = False  # the navigator resets the desired frame (landing, ...)
            rospy.logwarn_throttle(
                1.0, "[sendCogTarget] the current flight state({}) does not allow navigation".format(self.flight_state)
            )
            return False
        if vel is None:
            vel = np.zeros(3)
        if rot is None:
            rot = heading(self.getBaseRot())
        rot = np.asarray(rot, dtype=float)
        rot = rot / np.linalg.norm(rot)

        quat = QuaternionStamped()
        quat.header.stamp = rospy.Time.now()
        quat.header.frame_id = "world"
        quat.quaternion.x, quat.quaternion.y, quat.quaternion.z, quat.quaternion.w = (float(v) for v in rot)
        self.final_rot_quat_pub.publish(quat)
        if not self._yaw_on_desire:
            rospy.sleep(0.1)  # taken by the navigator before its target yaw goes to 0
            self._yaw_on_desire = True

        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.COG
        msg.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
        msg.pos_z_nav_mode = FlightNav.POS_VEL_MODE
        msg.target_pos_x, msg.target_pos_y, msg.target_pos_z = (float(v) for v in pos)
        msg.target_vel_x, msg.target_vel_y, msg.target_vel_z = (float(v) for v in vel)
        # relative to the desired CoG frame: 0
        msg.yaw_nav_mode = FlightNav.POS_VEL_MODE
        msg.target_yaw = 0.0
        msg.target_omega_z = 0.0
        msg.roll_nav_mode = msg.pitch_nav_mode = FlightNav.POS_MODE  # gimbalrotor_navigation: the target angle
        msg.target_roll = msg.target_pitch = 0.0
        self.direct_nav_pub.publish(msg)
        return True

    def waitForState(self, state, timeout=-1):
        start = rospy.get_time()
        while not rospy.is_shutdown() and self.flight_state != state:
            if 0 <= timeout < rospy.get_time() - start:
                return False
            rospy.sleep(0.1)
        return not rospy.is_shutdown()


if __name__ == "__main__":
    from IPython import embed

    ri = GimbalrotorInterface(init_node=True)
    embed()
