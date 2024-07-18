#!/usr/bin/env python

# rosrun delta object_release.py

import rospy

from aerial_robot_model.srv import AddExtraModule
from sensor_msgs.msg import JointState
import time

class objectRelease:
    def __init__(self):
        self.joints_ctrl_pub_ = rospy.Publisher('joints_ctrl', JointState, queue_size = 1)
        self.add_extra_module_service = rospy.wait_for_service('add_extra_module')

        time.sleep(2.0)

        joint_state_msg = JointState()
        joint_state_msg.name = ["joint1", "joint2"]
        joint_state_msg.position = [1.7, 1.7]
        self.joints_ctrl_pub_.publish(joint_state_msg)

        try:
            add_extra_module = rospy.ServiceProxy('add_extra_module', AddExtraModule)
            add_extra_module(action=-1, module_name="object")

        except rospy.ServiceException as e:
            print ("Service call failed: ", e)

if __name__ == "__main__":
    rospy.init_node("object_grasp")
    node = objectRelease()

