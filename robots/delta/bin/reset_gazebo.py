#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

rospy.wait_for_service("/gazebo/set_model_state") # wait for ros service
set_model_state_client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
set_model_state_msg = ModelState()
set_model_state_msg.model_name = "delta"
set_model_state_msg.pose.position.z = 0.4
set_model_state_msg.pose.orientation.w = 1.0
set_model_state_client(set_model_state_msg)

