#!/usr/bin/env python
# -*- coding: utf-8 -*-

# admittance control (x, y, yaw) based on the momentum-observer external wrench estimation
#   M_a * dv/dt + D_a * v = F_ext
# the bias of the estimation is removed in CoG frame, since it mainly comes from the body-fixed model error,
# then the admittance is integrated in world frame.
# the reference trajectory is sent to the flight navigator in POS_VEL_MODE, z is kept by the original controller

import math
import numpy as np
import rospy
import tf.transformations as tft
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from std_srvs.srv import SetBool, SetBoolResponse

HOVER_STATE = 5  # aerial_robot_navigation::HOVER_STATE

class AdmittanceControl:
    def __init__(self):
        robot_ns = rospy.get_param("~robot_ns", "gimbalrotor")

        self.rate = rospy.get_param("~rate", 100.0)
        # virtual inertia and damping for x/y [kg], [Ns/m] and yaw [kgm^2], [Nms/rad]
        self.mass = rospy.get_param("~mass", 2.0)
        self.damping = rospy.get_param("~damping", 4.0)
        self.inertia_yaw = rospy.get_param("~inertia_yaw", 0.2)
        self.damping_yaw = rospy.get_param("~damping_yaw", 0.4)
        # deadband to reject the estimation noise and residual bias [N], [Nm]
        self.force_deadband = rospy.get_param("~force_deadband", 0.5)
        self.torque_deadband = rospy.get_param("~torque_deadband", 0.05)
        # limits for safety
        self.max_vel = rospy.get_param("~max_vel", 0.5)  # [m/s]
        self.max_omega = rospy.get_param("~max_omega", 0.5)  # [rad/s]
        self.max_pos_err = rospy.get_param("~max_pos_err", 0.3)  # max distance between reference and current position [m]
        self.max_yaw_err = rospy.get_param("~max_yaw_err", 0.3)  # [rad]
        # bias estimation: averaged for calib_duration right after enabled, then tracked by a slow LPF (disabled if <= 0)
        self.calib_duration = rospy.get_param("~calib_duration", 2.0)  # [s]
        self.bias_time_constant = rospy.get_param("~bias_time_constant", 0.0)  # [s]

        self.flight_state = None
        self.odom = None
        self.wrench = None  # [fx, fy, fz, tx, ty, tz] in CoG frame
        self.enabled = False
        self.reset()

        self.nav_pub = rospy.Publisher("/" + robot_ns + "/uav/nav", FlightNav, queue_size=1)
        self.debug_pub = rospy.Publisher("~admittance_wrench", WrenchStamped, queue_size=1)
        rospy.Subscriber("/" + robot_ns + "/flight_state", UInt8, self.flightStateCallback)
        rospy.Subscriber("/" + robot_ns + "/uav/cog/odom", Odometry, self.odomCallback)
        rospy.Subscriber("/" + robot_ns + "/estimated_external_wrench_cog", WrenchStamped, self.wrenchCallback)
        rospy.Service("~enable", SetBool, self.enableCallback)

        if rospy.get_param("~auto_start", False):
            self.enabled = True

    def reset(self):
        self.ref_pos = None  # [x, y]
        self.ref_vel = np.zeros(2)
        self.ref_yaw = None
        self.ref_omega = 0.0
        self.bias = np.zeros(6)  # in CoG frame
        self.calib_sum = np.zeros(6)
        self.calib_count = 0
        self.calib_end_time = None

    def flightStateCallback(self, msg):
        self.flight_state = msg.data

    def odomCallback(self, msg):
        self.odom = msg

    def wrenchCallback(self, msg):
        # both force and torque are in CoG frame
        f = msg.wrench.force
        t = msg.wrench.torque
        self.wrench = np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def enableCallback(self, req):
        self.enabled = req.data
        if not self.enabled:
            self.stop()
            self.reset()
        rospy.loginfo("admittance control: %s", "enabled" if self.enabled else "disabled")
        return SetBoolResponse(success=True, message="")

    def currentPose(self):
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        rot = tft.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]  # from CoG frame to world frame
        return np.array([p.x, p.y]), yaw, rot

    def stop(self):
        # hold the last reference
        if self.ref_pos is None:
            return
        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.COG
        msg.pos_xy_nav_mode = FlightNav.POS_MODE
        msg.target_pos_x = self.ref_pos[0]
        msg.target_pos_y = self.ref_pos[1]
        msg.yaw_nav_mode = FlightNav.POS_MODE
        msg.target_yaw = self.ref_yaw
        self.nav_pub.publish(msg)

    @staticmethod
    def deadband(v, width):
        # apply to the norm to keep the direction of the force
        norm = np.linalg.norm(v)
        if norm <= width:
            return np.zeros_like(v)
        return v * (norm - width) / norm

    @staticmethod
    def admittance(vel, force, mass, damping, dt):
        # exact discretization of M dv/dt + D v = F with zero-order hold
        decay = math.exp(-damping / mass * dt)
        return vel * decay + (1.0 - decay) * force / damping

    def update(self, dt):
        if self.flight_state != HOVER_STATE or self.odom is None or self.wrench is None:
            if self.ref_pos is not None:
                rospy.logwarn("admittance control: not in hover state, reset")
                self.reset()
            return

        curr_pos, curr_yaw, rot = self.currentPose()

        # initialize the reference and calibrate the bias in hovering without contact
        if self.ref_pos is None:
            self.ref_pos = curr_pos.copy()
            self.ref_yaw = curr_yaw
            self.calib_end_time = rospy.Time.now() + rospy.Duration(self.calib_duration)
            rospy.loginfo("admittance control: start bias calibration for %.1f [s], do not touch the robot", self.calib_duration)
        if rospy.Time.now() < self.calib_end_time:
            self.calib_sum += self.wrench
            self.calib_count += 1
            return
        if self.calib_count > 0:
            self.bias = self.calib_sum / self.calib_count
            self.calib_count = 0
            rospy.loginfo("admittance control: bias in CoG frame [fx, fy, fz, tx, ty, tz] = %s, start admittance",
                          np.round(self.bias, 3))

        # remove the bias in CoG frame, then convert to world frame
        wrench = self.wrench - self.bias
        force_w = rot.dot(wrench[:3])
        torque_w = rot.dot(wrench[3:])
        force = self.deadband(force_w[:2], self.force_deadband)
        torque = self.deadband(torque_w[2:], self.torque_deadband)[0]

        # slowly track the bias only when no interaction is detected
        if self.bias_time_constant > 0 and np.linalg.norm(force) == 0 and torque == 0:
            alpha = dt / (self.bias_time_constant + dt)
            self.bias += alpha * (self.wrench - self.bias)

        # admittance dynamics
        self.ref_vel = self.admittance(self.ref_vel, force, self.mass, self.damping, dt)
        speed = np.linalg.norm(self.ref_vel)
        if speed > self.max_vel:
            self.ref_vel *= self.max_vel / speed
        self.ref_omega = self.admittance(self.ref_omega, torque, self.inertia_yaw, self.damping_yaw, dt)
        self.ref_omega = float(np.clip(self.ref_omega, -self.max_omega, self.max_omega))

        self.ref_pos += self.ref_vel * dt
        self.ref_yaw = self.ref_yaw + self.ref_omega * dt

        # keep the reference close to the robot, e.g., when the robot is blocked by the environment
        err = self.ref_pos - curr_pos
        dist = np.linalg.norm(err)
        if dist > self.max_pos_err:
            self.ref_pos = curr_pos + err * self.max_pos_err / dist
        yaw_err = math.atan2(math.sin(self.ref_yaw - curr_yaw), math.cos(self.ref_yaw - curr_yaw))
        self.ref_yaw = curr_yaw + float(np.clip(yaw_err, -self.max_yaw_err, self.max_yaw_err))
        self.ref_yaw = math.atan2(math.sin(self.ref_yaw), math.cos(self.ref_yaw))

        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.COG
        msg.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
        msg.target_pos_x = self.ref_pos[0]
        msg.target_pos_y = self.ref_pos[1]
        msg.target_vel_x = self.ref_vel[0]
        msg.target_vel_y = self.ref_vel[1]
        msg.yaw_nav_mode = FlightNav.POS_VEL_MODE
        msg.target_yaw = self.ref_yaw
        msg.target_omega_z = self.ref_omega
        msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
        self.nav_pub.publish(msg)

        debug = WrenchStamped()
        debug.header.stamp = msg.header.stamp
        debug.header.frame_id = "world"
        debug.wrench.force.x = force[0]
        debug.wrench.force.y = force[1]
        debug.wrench.torque.z = torque
        self.debug_pub.publish(debug)

    def run(self):
        rate = rospy.Rate(self.rate)
        prev_time = rospy.Time.now()
        while not rospy.is_shutdown():
            rate.sleep()
            now = rospy.Time.now()
            dt = (now - prev_time).to_sec()
            prev_time = now
            if not self.enabled or dt <= 0:
                continue
            self.update(dt)


if __name__ == "__main__":
    rospy.init_node("admittance_control")
    AdmittanceControl().run()
