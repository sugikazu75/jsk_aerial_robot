#!/usr/bin/env python
# -*- coding: utf-8 -*-

# apply a sequence of external wrenches to the robot in gazebo, and publish the applied wrench for comparison with the estimated one

import rospy
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest, BodyRequest
from geometry_msgs.msg import Point, Vector3, Wrench, WrenchStamped

# [force (world frame) [N], torque (world frame) [Nm], duration [s]]
WRENCH_SEQUENCE = [
    [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 3.0],
    [[2.0, 0.0, 0.0], [0.0, 0.0, 0.0], 5.0],
    [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 3.0],
    [[0.0, -2.0, 0.0], [0.0, 0.0, 0.0], 5.0],
    [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 3.0],
    [[0.0, 0.0, -1.0], [0.0, 0.0, 0.0], 5.0],
    [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 3.0],
    [[0.0, 0.0, 0.0], [0.0, 0.0, 0.3], 5.0],
    [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 3.0],
]

class ExternalWrenchApplier:
    def __init__(self):
        robot_ns = rospy.get_param("~robot_ns", "gimbalrotor")
        self.body_name = rospy.get_param("~body_name", robot_ns + "::main_body")
        self.reference_frame = rospy.get_param("~reference_frame", "world")
        self.offset = rospy.get_param("~offset", [0.0, 0.0, 0.0])  # point of action in body frame
        self.loop = rospy.get_param("~loop", False)
        self.pub_rate = rospy.get_param("~pub_rate", 50.0)

        rospy.wait_for_service("/gazebo/apply_body_wrench")
        rospy.wait_for_service("/gazebo/clear_body_wrenches")
        self.apply_srv = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
        self.clear_srv = rospy.ServiceProxy("/gazebo/clear_body_wrenches", BodyRequest)

        self.applied_wrench_pub = rospy.Publisher("/" + robot_ns + "/applied_external_wrench", WrenchStamped, queue_size=1)
        self.applied_wrench = Wrench()

        rospy.on_shutdown(self.clear)

    def clear(self):
        try:
            self.clear_srv(self.body_name)
        except rospy.ServiceException as e:
            rospy.logwarn("failed to clear wrench: %s", e)
        self.applied_wrench = Wrench()

    def apply(self, force, torque):
        self.clear()  # gazebo accumulates wrenches, so clear the previous one first
        if all(v == 0.0 for v in force + torque):
            return

        req = ApplyBodyWrenchRequest()
        req.body_name = self.body_name
        req.reference_frame = self.reference_frame
        req.reference_point = Point(*self.offset)
        req.wrench = Wrench(Vector3(*force), Vector3(*torque))
        req.start_time = rospy.Time(0)
        req.duration = rospy.Duration(-1)  # until cleared
        res = self.apply_srv(req)
        if not res.success:
            rospy.logerr("failed to apply wrench: %s", res.status_message)
            return
        self.applied_wrench = req.wrench

    def publish(self):
        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.reference_frame
        msg.wrench = self.applied_wrench
        self.applied_wrench_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            for force, torque, duration in WRENCH_SEQUENCE:
                rospy.loginfo("apply force: %s, torque: %s for %.1f [s]", force, torque, duration)
                self.apply(force, torque)
                end_time = rospy.Time.now() + rospy.Duration(duration)
                while not rospy.is_shutdown() and rospy.Time.now() < end_time:
                    self.publish()
                    rate.sleep()
                if rospy.is_shutdown():
                    return

            self.clear()
            if not self.loop:
                break

        rospy.loginfo("finish applying external wrench sequence")


if __name__ == "__main__":
    rospy.init_node("apply_external_wrench")
    ExternalWrenchApplier().run()
