#!/usr/bin/env python

import rospy
from aerial_robot_msgs.msg import FlightNav, PoseControlPid
from delta.msg import GimbalPlanning
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from dynamic_reconfigure.msg import DoubleParameter
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from IPython import embed
import math
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, Empty, Int8, Int16, UInt8
import tf
import time

class DeltaInterface:
    def __init__(self):
        # ros messages
        self.cog_odom_ = Odometry()
        self.flight_state_ = UInt8()
        self.pid_ = PoseControlPid()
        self.joint_state_ = JointState()
        self.joy_ = Joy()

        ## flight states
        self.ARM_OFF_STATE = 0
        self.START_STATE = 1
        self.ARM_ON_STATE = 2
        self.TAKEOFF_STATE = 3
        self.LAND_STATE = 4
        self.HOVER_STATE = 5
        self.STOP_STATE = 6

        # parameters for task
        self.robot_ns = rospy.get_namespace()
        self.force_skip_ = False
        self.dynamic_reconfigure_params = ["thrust_weight", "gimbal_linear_solution_dist_weight", "gimbal_current_angle_dist_weight", "gimbal_center_dist_weight", "joint_torque_weight", "attitude_control_roll_threshold", "attitude_control_pitch_threshold", "attitude_control_yaw_threshold"]

        # ros subscriber and publisher
        self.tf_listener_ = tf.TransformListener()
        self.joints_ctrl_pub_ = rospy.Publisher('joints_ctrl', JointState, queue_size = 1)
        self.target_pose_pub_ = rospy.Publisher('target_pose', PoseStamped, queue_size = 1)
        self.uav_nav_pub_ = rospy.Publisher("uav/nav", FlightNav, queue_size=1)
        self.current_target_baselink_rpy_pub = rospy.Publisher("current_target_baselink_rpy", Vector3Stamped, queue_size = 1)
        self.final_target_baselink_rpy_pub = rospy.Publisher("final_target_baselink_rpy", Vector3Stamped, queue_size = 1)
        self.nominal_contact_flag_pub = rospy.Publisher("contact_planning/nominal_contact_flag", Bool, queue_size = 1)
        self.commanded_contacting_link_index_pub = rospy.Publisher("contact_planning/commanded_contacting_link_index", Int16, queue_size = 1)
        self.correct_baselink_pose_pub = rospy.Publisher("correct_baselink_pose", Bool, queue_size = 1)
        self.gimbal_planning_pub = rospy.Publisher("gimbal_planning", GimbalPlanning, queue_size = 1)

        self.start_pub_ = rospy.Publisher('teleop_command/start', Empty, queue_size = 1)
        self.takeoff_pub_ = rospy.Publisher('teleop_command/takeoff', Empty, queue_size = 1)
        self.land_pub_ = rospy.Publisher('teleop_command/land', Empty, queue_size = 1)
        self.force_landing_pub_ = rospy.Publisher('teleop_command/force_landing', Empty, queue_size = 1)
        self.halt_pub_ = rospy.Publisher('teleop_command/halt', Empty, queue_size = 1)
        self.control_mode_pub_ = rospy.Publisher('teleop_command/ctrl_mode', Int8, queue_size = 1)
        rospy.Subscriber("debug/pose/pid", PoseControlPid, self.pidCallback)
        rospy.Subscriber('flight_state', UInt8, self.flightStateCallback)
        rospy.Subscriber('force_skip', Empty, self.forceSkipCallback)
        rospy.Subscriber("joint_states", JointState, self.jointStateCallback)
        rospy.Subscriber("joy", Joy, self.joyCallback)
        rospy.Subscriber("uav/cog/odom", Odometry, self.cogOdomCallback)

        rospy.wait_for_service("controller/nlopt/set_parameters")
        self.dynamic_reconfigure_client = rospy.ServiceProxy("controller/nlopt/set_parameters", Reconfigure)

        time.sleep(2.0)
        print("created " + self.robot_ns + " interface")


    def start(self, sleep = 1.0):
        self.start_pub_.publish()
        rospy.sleep(sleep)
    def takeoff(self):
        self.takeoff_pub_.publish()
    def land(self):
        self.land_pub_.publish()
    def forceLanding(self):
        self.force_landing_pub_.publish()
    def halt(self):
        self.halt_pub_.publish()

    def cogOdomCallback(self, msg):
        self.cog_odom_ = msg

    def getCogOdom(self):
        return self.cog_odom_
    def getCogPosition(self):
        return np.array([self.cog_odom_.pose.pose.position.x, self.cog_odom_.pose.pose.position.y, self.cog_odom_.pose.pose.position.z])
    def getCogQuaternion(self):
        return np.array([self.cog_odom_.pose.pose.orientation.x, self.cog_odom_.pose.pose.orientation.y, self.cog_odom_.pose.pose.orientation.z, self.cog_odom_.pose.pose.orientation.w])
    def getCogRotationMatrix(self):
        return np.array(tf.transformations.quaternion_matrix(self.getCogQuaternion()))[0:3, 0:3]

    def flightStateCallback(self, msg):
        self.flight_state_ = msg.data
    def getFlightState(self):
        return self.flight_state_

    def pidCallback(self, msg):
        self.pid_ = msg
    def getPID(self):
        return self.pid_

    def posControlConverged(self, target=[0, 0, 0], thresh=0.05):
        return math.sqrt((target[0] - self.cog_odom_.pose.pose.position.x)**2 + (target[1] - self.cog_odom_.pose.pose.position.y)**2 + (target[2] - self.cog_odom_.pose.pose.position.z)**2) < thresh

    def getPosControlError(self, target=[0, 0, 0]):
        return [target[0] - self.cog_odom_.pose.pose.position.x, target[1] - self.cog_odom_.pose.pose.position.y, target[2] - self.cog_odom_.pose.pose.position.z]

    def yawControlConverged(self, target=None, thresh=0.1):
        if target is None:
            return abs(self.pid_.yaw.err_p) < thresh
        else:
            yaw = tf.transformations.euler_from_quaternion([self.cog_odom_.pose.pose.orientation.x, self.cog_odom_.pose.pose.orientation.y, self.cog_odom_.pose.pose.orientation.z, self.cog_odom_.pose.pose.orientation.w])[2]
            return abs(yaw - target) < thresh

    def setTargetPos(self, pos=[0, 0, 0]):
        msg = FlightNav()
        msg.control_frame = msg.WORLD_FRAME
        msg.target = msg.COG
        msg.pos_xy_nav_mode = msg.POS_MODE
        msg.target_pos_x = pos[0]
        msg.target_pos_y = pos[1]
        msg.pos_z_nav_mode = msg.POS_MODE
        msg.target_pos_z = pos[2]
        self.uav_nav_pub_.publish(msg)

    def setTargetPosTraj(self, pos=[0, 0, 0]):
        msg = PoseStamped()
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        self.target_pose_pub_.publish(msg)

    def setTargetPosFromCurrentState(self):
        msg = FlightNav()
        msg.control_frame = msg.WORLD_FRAME
        msg.target = msg.COG
        msg.pos_xy_nav_mode = msg.POS_MODE
        msg.target_pos_x = self.cog_odom_.pose.pose.position.x
        msg.target_pos_y = self.cog_odom_.pose.pose.position.y
        msg.pos_z_nav_mode = msg.POS_MODE
        msg.target_pos_z = self.cog_odom_.pose.pose.position.z

    def setXYControlMode(self, mode):
        msg = Int8()
        if mode > 2:
            return
        else:
            msg.data = mode
            self.control_mode_pub_.publish(msg)

    def setTargetYaw(self, yaw):
        msg = FlightNav()
        msg.yaw_nav_mode = msg.POS_MODE
        msg.target_yaw = yaw
        self.uav_nav_pub_.publish(msg)

    def setTargetYawTraj(self, yaw):
        msg = PoseStamped()
        msg.pose.position.x = self.cog_odom_.pose.pose.position.x
        msg.pose.position.y = self.cog_odom_.pose.pose.position.y
        msg.pose.position.z = self.cog_odom_.pose.pose.position.z
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        self.target_pose_pub_.publish(msg)

    def setCurrentTargetBaselinkRPY(self, current_target_baselink_rpy_msg):
        self.current_target_baselink_rpy_pub.publish(current_target_baselink_rpy_msg)

    def setFinalTargetBaselinkRPY(self, final_target_baselink_rpy_msg):
        self.final_target_baselink_rpy_pub.publish(final_target_baselink_rpy_msg)

    def setNominalContactFlag(self, nominal_contact_flag_msg):
        self.nominal_contact_flag_pub.publish(nominal_contact_flag_msg)

    def setCommandedContactingLinkIndex(self, commanded_contacting_link_index_msg):
        self.commanded_contacting_link_index_pub.publish(commanded_contacting_link_index_msg)

    def setCorrectBaselinkPose(self, correct_baselink_pose_msg):
        self.correct_baselink_pose_pub.publish(correct_baselink_pose_msg)

    def setGimbalPlanningState(self, index, angle):
        msg = GimbalPlanning()
        msg.index = [index]
        msg.angle = [angle]
        self.gimbal_planning_pub.publish(msg)

    def setDynamicReconfigureParameter(self, index, value):
        double_param = DoubleParameter()
        double_param.name = self.dynamic_reconfigure_params[index]
        double_param.value = value
        req = ReconfigureRequest()
        req.config.doubles.append(double_param)
        try:
            res = self.dynamic_reconfigure_client(req)
        except rospy.ServiceException as e:
            rospy.logerr("dynamic_reconfigure service call failed")

    def getTransform(self, parent, child):
        try:
            (trans, rot) = self.tf_listener_.lookupTransform(self.robot_ns + parent, self.robot_ns + child, rospy.Time(0))
            trans = np.array([trans[0], trans[1], trans[2]])
            rot = np.array([rot[0], rot[1], rot[2], rot[3]])
            return (trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)

    def sendJointState(self, joint_state_msg):
        self.joints_ctrl_pub_.publish(joint_state_msg)

    def jointStateCallback(self, msg):
        self.joint_state_ = msg

    def joyCallback(self, msg):
        self.joy_ = msg
        if(self.joy_.axes[9] == -1 and self.joy_.axes[10] == -1 and (not self.getForceSkipFlag())):
            print("force skip!")
            self.setForceSkipFlag(True)

    def forceSkipCallback(self, msg):
        self.setForceSkipFlag(True)
    def getForceSkipFlag(self):
        return self.force_skip_
    def setForceSkipFlag(self, flag):
        self.force_skip_ = flag

if __name__ == "__main__":
    rospy.init_node("delta_interface")
    robot = DeltaInterface()
    embed()
