#!/usr/bin/env python3

import time
import rospy
from aerial_robot_msgs.msg import ControlInput

rospy.init_node("init_pose")
control_input_pub = rospy.Publisher("/dragon/mujoco/ctrl_input", ControlInput, queue_size=1)
control_input = ControlInput()
control_input.name = ['joint1_yaw' , 'joint2_yaw', 'joint3_yaw']
control_input.input = [1.57, 1.57, 1.57]

time.sleep(1.0)
control_input_pub.publish(control_input)
