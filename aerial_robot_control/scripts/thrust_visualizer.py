#!/usr/bin/env python3
"""The thrust of each rotor from the command to the flight controller (four_axes/command), for RViz.

  rosrun aerial_robot_control thrust_visualizer.py __ns:=<robot_ns>

  four_axes/command       spinal/FourAxisCommand: base_thrust, one value per rotor (the thrust [N],
                          gimbal_calc_in_fc false), or rotor_coef values per rotor (its components,
                          gimbal_calc_in_fc true: their norm, without the attitude terms of spinal)
  ~thrust                 WrenchStamped per rotor: the thrust along z of its thrust frame
                          (<tf_prefix>/thrust<i>), the rotor axis
  ~thrust_value/thrust<i> Float32 per rotor: its thrust [N] (e.g. for the jsk_rviz_plugins plotters)

  ~tf_prefix              of the thrust frames (default: the namespace)
  ~rotor_num              the number of rotors (default: the thrust<i> links of robot_description)
  ~rate                   [Hz] at most (the command comes at the control rate)
"""

import re

import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
from spinal.msg import FourAxisCommand
from std_msgs.msg import Float32


class ThrustVisualizer:
    def __init__(self):
        prefix = rospy.get_param("~tf_prefix", rospy.get_namespace().strip("/"))
        rotor_num = rospy.get_param("~rotor_num", 0)
        if rotor_num <= 0:
            urdf = rospy.get_param("robot_description", "")
            rotor_num = len(set(re.findall(r'<link\s+name="thrust(\d+)"', urdf)))
        if rotor_num <= 0:
            raise RuntimeError("no thrust<i> link in robot_description: give ~rotor_num")
        self.period = rospy.Duration(1.0 / rospy.get_param("~rate", 50.0))
        self.last = rospy.Time(0)
        self.names = ["thrust%d" % (i + 1) for i in range(rotor_num)]
        self.frames = [prefix + "/" + name if prefix else name for name in self.names]
        self.thrust_pub = rospy.Publisher("~thrust", WrenchStamped, queue_size=2 * rotor_num)
        self.value_pubs = [rospy.Publisher("~thrust_value/" + name, Float32, queue_size=1) for name in self.names]
        rospy.Subscriber("four_axes/command", FourAxisCommand, self.callback, queue_size=1)
        rospy.loginfo("thrust of %d rotors from %s", rotor_num, rospy.resolve_name("four_axes/command"))

    def callback(self, msg):
        values = np.asarray(msg.base_thrust, dtype=float)
        n = len(self.names)
        if len(values) == 0 or len(values) % n != 0:
            rospy.logwarn_throttle(5.0, "base_thrust of %d values for %d rotors: skipped", len(values), n)
            return
        thrust = np.linalg.norm(values.reshape(n, -1), axis=1) if len(values) > n else values
        stamp = rospy.Time.now()
        if stamp - self.last < self.period and stamp >= self.last:  # (the clock may go back in simulation)
            return
        self.last = stamp
        for frame, value, value_pub in zip(self.frames, thrust, self.value_pubs):
            wrench = WrenchStamped()
            wrench.header.stamp = stamp
            wrench.header.frame_id = frame
            wrench.wrench.force.z = float(value)
            self.thrust_pub.publish(wrench)
            value_pub.publish(Float32(float(value)))


if __name__ == "__main__":
    rospy.init_node("thrust_visualizer")
    ThrustVisualizer()
    rospy.spin()
