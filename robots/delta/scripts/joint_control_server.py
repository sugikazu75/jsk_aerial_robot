#!/usr/bin/env python

import rospy
from IPython import embed
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32
import time

class jointControlServer:
    def __init__(self):
        self.joint_num = 2
        self.joint_list = ["joint1", "joint2"]
        self.torque_control_flag = False
        self.target_effort = np.zeros(self.joint_num)
        self.current_effort = np.zeros(self.joint_num)
        self.target_position = None
        self.current_position = np.zeros(self.joint_num)
        self.command_position = np.zeros(self.joint_num)
        self.upper_bound = np.zeros(self.joint_num)
        self.lower_bound = np.zeros(self.joint_num)
        self.bound_range = 0.05 * np.ones(self.joint_num)
        self.p_gain = 0.001
        self.node_id = rospy.get_name()

        self.joints_ctrl_pub_ = rospy.Publisher('joints_ctrl', JointState, queue_size = 1)
        rospy.Subscriber("p_gain", Float32, self.pGainCallback)
        rospy.Subscriber("torque_control_flag", Bool, self.torqueControlFlagCallback)
        rospy.Subscriber("joints_ctrl", JointState, self.jointControlCallback)
        rospy.Subscriber("joint_states", JointState, self.jointStateCallback)
        rospy.Subscriber("target_effort", JointState, self.targetEffortCallback)

        rospy.Timer(rospy.Duration(1.0 / 40.0), self.timerCallback)

        time.sleep(2.0)

        rospy.loginfo("started " + rospy.get_namespace() + " joint control server")

    def jointStateCallback(self, msg):
        for i in range(len(msg.position)):
            if(msg.name[i] == "joint1"):
                self.current_position[0] = msg.position[i]
                self.current_effort[0] = msg.effort[i]
            if(msg.name[i] == "joint2"):
                self.current_position[1] = msg.position[i]
                self.current_effort[1] = msg.effort[i]

    def jointControlCallback(self, msg):
        if msg.header.frame_id == self.node_id:
            return
        self.target_position = np.zeros(self.joint_num)
        if(len(msg.name) != len(msg.position)):
            if len(msg.position) == self.joint_num:
                for i in range(self.joint_num):
                    self.target_position[i] = msg.position[i]
            else:
                rospy.logerr("size of robot joint is not identical to message")
                return
        else:
            for i in range(len(msg.position)):
                if(msg.name[i] == "joint1"):
                    self.target_position[0] = msg.position[i]
                if(msg.name[i] == "joint2"):
                    self.target_position[1] = msg.position[i]
        self.lower_bound = self.target_position - self.bound_range
        self.upper_bound = self.target_position + self.bound_range
        rospy.loginfo("target position: {}".format(self.target_position))

    def targetEffortCallback(self, msg):
        if(len(msg.name) != len(msg.effort)):
            if len(msg.effort) == self.joint_num:
                for i in range(self.joint_num):
                    self.target_effort[i] = msg.effort[i]
            else:
                rospy.logerr("size of robot joint is not identical to message")
            return
        for i in range(len(msg.effort)):
            if(msg.name[i] == "joint1"):
                self.target_effort[0] = msg.effort[i]
            if(msg.name[i] == "joint2"):
                self.target_effort[1] = msg.effort[i]
        rospy.loginfo("target effort: {}".format(self.target_effort))

    def torqueControlFlagCallback(self, msg):
        self.torque_control_flag = msg.data
        rospy.loginfo("torque control flag: {}".format(self.torque_control_flag))

    def pGainCallback(self, msg):
        self.p_gain = msg.data
        rospy.loginfo("torque control p gain: {}".format(self.p_gain))

    def timerCallback(self, event):
        if not self.torque_control_flag:
            return
        if self.target_position is None:
            return
        effort_error = self.target_effort - self.current_effort
        self.command_position = np.clip(self.current_position + self.p_gain * effort_error, self.lower_bound, self.upper_bound)
        msg = JointState()
        msg.header.frame_id = self.node_id
        msg.name = self.joint_list
        msg.position = self.command_position
        self.joints_ctrl_pub_.publish(msg)

if __name__ == "__main__":
    rospy.init_node("joint_control_server", anonymous=True)
    robot = jointControlServer()
    embed()
