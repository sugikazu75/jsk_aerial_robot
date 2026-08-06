"""ROS2 counterpart of src/aerial_robot_base/robot_interface.py.

Same class, same method names and the same semantics; what changes is
underneath. Four things had no direct translation:

  - **A node to hang things off.** rospy had a process-global one; rclpy does
    not, so RobotInterface owns one (or takes yours) and spins it on a thread of
    its own. Without that spinning nothing arrives, and every convergence check
    would wait forever on state that never updates.
  - **`rospy.sleep`.** Replaced by the node clock's sleep, so under simulation
    everything here follows /clock the way rospy did. A test that slept in wall
    seconds against a simulator running at a different rate would be measuring
    the wrong thing.
  - **ros_numpy and tf.transformations**, neither of which exists for ROS2.
    See transform_utils.
  - **Finding the robot namespace.** rospy asked the master which nodes
    subscribe to what; ROS2 has no master, so this reads the graph instead.
"""

import threading

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import tf2_ros

from aerial_robot_msgs.msg import FlightNav, PoseControlPid
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, UInt8
from std_srvs.srv import SetBool

from aerial_robot_base.transform_utils import (euler_from_quaternion, numpify_quaternion,
                                               numpify_vector, quaternion_from_euler,
                                               quaternion_inverse, quaternion_multiply)


class RobotInterface(object):

    # flight states; mirrors aerial_robot_navigation::BaseNavigator
    ARM_OFF_STATE = 0
    START_STATE = 1
    ARM_ON_STATE = 2
    TAKEOFF_STATE = 3
    LAND_STATE = 4
    HOVER_STATE = 5
    STOP_STATE = 6

    def __init__(self, robot_ns='', debug_view=False, node=None, node_name='robot_interface'):

        self.debug_view = debug_view
        self.joint_state = JointState()
        self.cog_odom = None
        self.base_odom = None
        self.control_pid = None
        self.flight_state = None
        self.target_pos = np.array([0, 0, 0])

        self._owns_node = node is None
        if self._owns_node:
            if not rclpy.ok():
                rclpy.init()
            node = Node(node_name)
        self.node = node
        self.logger = node.get_logger()

        self.default_pos_thresh = self._param('default_pos_thresh', [0.1, 0.1, 0.1])  # m
        self.default_rot_thresh = self._param('default_rot_thresh', [0.05, 0.05, 0.05])  # rad
        self.default_vel_thresh = self._param('default_vel_thresh', [0.05, 0.05, 0.05])  # m/s

        self.robot_ns = robot_ns
        if not self.robot_ns:
            self.robot_ns = self._discover_robot_ns()

        # Teleop
        self.start_pub = node.create_publisher(Empty, self.robot_ns + '/teleop_command/start', 1)
        self.takeoff_pub = node.create_publisher(Empty, self.robot_ns + '/teleop_command/takeoff', 1)
        self.land_pub = node.create_publisher(Empty, self.robot_ns + '/teleop_command/land', 1)
        self.force_landing_pub = node.create_publisher(
            Empty, self.robot_ns + '/teleop_command/force_landing', 1)
        self.halt_pub = node.create_publisher(Empty, self.robot_ns + '/teleop_command/halt', 1)

        # Odometry & control
        self.cog_odom_sub = node.create_subscription(
            Odometry, self.robot_ns + '/uav/cog/odom', self.cogOdomCallback, 1)
        self.base_odom_sub = node.create_subscription(
            Odometry, self.robot_ns + '/uav/baselink/odom', self.baseOdomCallback, 1)
        self.control_pid_sub = node.create_subscription(
            PoseControlPid, self.robot_ns + '/debug/pose/pid', self.controlPidCallback, 1)

        # Navigation
        self.flight_state_sub = node.create_subscription(
            UInt8, self.robot_ns + '/flight_state', self.flightStateCallback, 1)
        self.traj_nav_pub = node.create_publisher(PoseStamped, self.robot_ns + '/target_pose', 1)
        self.direct_nav_pub = node.create_publisher(FlightNav, self.robot_ns + '/uav/nav', 1)
        self.final_rot_pub = node.create_publisher(
            Vector3Stamped, self.robot_ns + '/final_target_baselink_rpy', 1)

        # Joint
        self.joint_state_sub = node.create_subscription(
            JointState, self.robot_ns + '/joint_states', self.jointStateCallback, 1)
        self.joint_ctrl_pub = node.create_publisher(JointState, self.robot_ns + '/joints_ctrl', 1)
        self.set_joint_torque_client = node.create_client(
            SetBool, self.robot_ns + '/joints/torque_enable')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        # Nothing arrives until something spins. rospy did this for you.
        self._executor = None
        self._spin_thread = None
        if self._owns_node:
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(node)
            self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._spin_thread.start()

        start_time = self.now()
        while rclpy.ok():
            if self.now() - start_time > 5.0:
                self.logger.error('cannot connect to {}'.format(self.robot_ns))
                break

            if self.base_odom is not None:
                self.logger.info('connect to {}!'.format(self.robot_ns))
                break

            self.sleep(0.1)

    # ---- ROS2 plumbing -------------------------------------------------------

    def _param(self, name, default):
        """A private-namespace rosparam; ROS2 parameters are per-node and flat."""
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, default)
        return self.node.get_parameter(name).value

    def _discover_robot_ns(self):
        """The robot namespace, from whoever advertises teleop_command/start.

        rospy asked the master for the system state. ROS2 has no master, so ask
        the graph - which needs a moment to populate after a node starts.
        """
        deadline = self.now() + 5.0
        while rclpy.ok() and self.now() < deadline:
            names = [name for name, _ in self.node.get_topic_names_and_types()
                     if name.endswith('/teleop_command/start')]
            if len(names) == 1:
                return names[0].split('/teleop')[0]
            self.sleep(0.1)

        self.logger.warn('could not find a robot namespace from teleop_command/start')
        return ''

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def sleep(self, duration):
        """Sleep on the node's clock, so simulation time is honoured."""
        self.node.get_clock().sleep_for(Duration(seconds=duration))

    def shutdown(self):
        if self._executor is not None:
            self._executor.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
        if self._owns_node:
            self.node.destroy_node()

    # ---- teleop --------------------------------------------------------------

    def start(self, sleep=1.0):
        self.start_pub.publish(Empty())
        self.sleep(sleep)

    def takeoff(self):
        self.takeoff_pub.publish(Empty())

    def land(self):
        self.land_pub.publish(Empty())

    def forceLanding(self):
        self.force_landing_pub.publish(Empty())

    def halt(self):
        self.halt_pub.publish(Empty())

    # ---- callbacks -----------------------------------------------------------

    def baseOdomCallback(self, msg):
        self.base_odom = msg

    def cogOdomCallback(self, msg):
        self.cog_odom = msg

    def flightStateCallback(self, msg):
        self.flight_state = msg.data

    def controlPidCallback(self, msg):
        self.control_pid = msg

    def jointStateCallback(self, msg):
        joint_state = JointState()

        # only extract joints, excluding other components like gimbals
        for n, j in zip(msg.name, msg.position):
            if 'joint' not in n:
                continue
            joint_state.name.append(n)
            joint_state.position.append(j)
        self.joint_state = joint_state

    # ---- state ---------------------------------------------------------------

    def getControlPid(self):
        return self.control_pid

    def getBaseOdom(self):
        return self.base_odom

    def getCogOdom(self):
        return self.cog_odom

    def getBasePos(self):
        return numpify_vector(self.base_odom.pose.pose.position)

    def getBaseRot(self):
        return numpify_quaternion(self.base_odom.pose.pose.orientation)

    def getBaseRPY(self):
        return euler_from_quaternion(self.getBaseRot())

    def getBaseLinearVel(self):
        return numpify_vector(self.base_odom.twist.twist.linear)

    def getBaseAngularVel(self):
        return numpify_vector(self.base_odom.twist.twist.angular)

    def getCogPos(self):
        return numpify_vector(self.cog_odom.pose.pose.position)

    def getCogRot(self):
        return numpify_quaternion(self.cog_odom.pose.pose.orientation)

    def getCogRPY(self):
        return euler_from_quaternion(self.getCogRot())

    def getCogLinVel(self):
        return numpify_vector(self.cog_odom.twist.twist.linear)

    def getCogAngVel(self):
        return numpify_vector(self.cog_odom.twist.twist.angular)

    def getFlightState(self):
        return self.flight_state

    def getTargetPos(self):
        return self.target_pos

    def getJointState(self):
        return self.joint_state

    def getTF(self, frame_id, wait=0.5, parent_frame_id='world'):
        return self.tf_buffer.lookup_transform(
            parent_frame_id, frame_id, rclpy.time.Time(), Duration(seconds=wait))

    # ---- convergence ---------------------------------------------------------

    def convergenceCheck(self, timeout, func, *args, **kwargs):

        # if timeout is -1, it means no time constraint
        if timeout < 0:
            return True

        start_time = self.now()

        while rclpy.ok():

            if self.flight_state != self.HOVER_STATE and self.flight_state != self.ARM_OFF_STATE:
                self.logger.warn(
                    '[{}]: preempt because current flight state({}) allows no more motion'.format(
                        func.__name__, self.flight_state))
                return False

            if func(*args, **kwargs):
                self.logger.info('[{}]: convergence'.format(func.__name__))
                return True

            if self.now() - start_time > timeout:
                self.logger.warn('[{}]: timeout, cannot convergence'.format(func.__name__))
                return False

            self.sleep(0.1)

        return False

    # ---- navigation ----------------------------------------------------------

    def goPos(self, pos, pos_thresh=0.1, vel_thresh=0.05, timeout=30):
        return self.navigate(pos=pos, pos_thresh=pos_thresh, vel_thresh=vel_thresh, timeout=timeout)

    def rotateYaw(self, yaw, yaw_thresh=0.1, timeout=30):
        return self.rotate(quaternion_from_euler(0, 0, yaw), yaw_thresh, timeout)

    def rotate(self, rot, rot_thresh=0.1, timeout=30):
        return self.navigate(rot=rot, rot_thresh=rot_thresh, timeout=timeout)

    def goPosYaw(self, pos, yaw, pos_thresh=0.1, vel_thresh=0.05, yaw_thresh=0.1, timeout=30):
        rot = quaternion_from_euler(0, 0, yaw)
        return self.goPose(pos, rot, pos_thresh, vel_thresh, yaw_thresh, timeout)

    def goPose(self, pos, rot, pos_thresh=0.1, vel_thresh=0.05, rot_thresh=0.1, timeout=30):
        return self.navigate(pos=pos, rot=rot, pos_thresh=pos_thresh, vel_thresh=vel_thresh,
                             rot_thresh=rot_thresh, timeout=timeout)

    def goVel(self, vel):
        return self.navigate(lin_vel=vel)

    def rotateVel(self, vel):
        return self.navigate(ang_vel=vel)

    def trajectoryNavigate(self, pos, rot):

        if self.flight_state != self.HOVER_STATE:
            self.logger.error(
                '[Navigate] the current flight state({}) does not allow navigation'.format(
                    self.flight_state))
            return

        if pos is None:
            pos = self.getCogPos()
        if rot is None:
            rot = self.getCogRot()

        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.x = float(rot[0])
        msg.pose.orientation.y = float(rot[1])
        msg.pose.orientation.z = float(rot[2])
        msg.pose.orientation.w = float(rot[3])
        self.traj_nav_pub.publish(msg)

    def directNavigate(self, pos, rot, lin_vel, ang_vel):

        if self.flight_state != self.HOVER_STATE:
            self.logger.error(
                '[Navigate] the current flight state({}) does not allow navigation'.format(
                    self.flight_state))
            return

        msg = FlightNav()
        msg.header.stamp = self.node.get_clock().now().to_msg()

        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.COG

        pos_mode = FlightNav.NO_NAVIGATION
        if pos is None:
            if lin_vel is not None:
                pos_mode = FlightNav.VEL_MODE
        else:
            pos_mode = FlightNav.POS_MODE if lin_vel is None else FlightNav.POS_VEL_MODE

        rot_mode = FlightNav.NO_NAVIGATION
        if rot is None:
            if ang_vel is not None:
                rot_mode = FlightNav.VEL_MODE
        else:
            rot_mode = FlightNav.POS_MODE if ang_vel is None else FlightNav.POS_VEL_MODE

        if pos is None:
            pos = self.getCogPos()
        if rot is None:
            rot = self.getCogRot()
        _, _, yaw = euler_from_quaternion(rot)

        if lin_vel is None:
            lin_vel = np.array([0, 0, 0])
        if ang_vel is None:
            ang_vel = np.array([0, 0, 0])

        msg.pos_xy_nav_mode = pos_mode
        msg.target_pos_x = float(pos[0])
        msg.target_pos_y = float(pos[1])
        msg.target_vel_x = float(lin_vel[0])
        msg.target_vel_y = float(lin_vel[1])
        msg.pos_z_nav_mode = pos_mode
        msg.target_pos_z = float(pos[2])
        msg.target_vel_z = float(lin_vel[2])
        msg.yaw_nav_mode = rot_mode
        msg.target_yaw = float(yaw)
        msg.target_omega_z = float(ang_vel[2])

        self.direct_nav_pub.publish(msg)

    def navigate(self, pos=None, rot=None, lin_vel=None, ang_vel=None,
                 pos_thresh=0.1, vel_thresh=0, rot_thresh=0, timeout=-1):

        if self.flight_state != self.HOVER_STATE:
            self.logger.error(
                '[Navigate] the current flight state({}) does not allow navigation'.format(
                    self.flight_state))
            return False

        if rot is not None and len(rot) == 3:
            rot = quaternion_from_euler(*rot)

        if lin_vel is None and ang_vel is None:
            self.trajectoryNavigate(pos, rot)
        else:
            self.directNavigate(pos, rot, lin_vel, ang_vel)

        return self.poseConvergenceCheck(timeout, target_pos=pos, target_rot=rot,
                                         pos_thresh=pos_thresh, vel_thresh=vel_thresh,
                                         rot_thresh=rot_thresh)

    def poseConvergenceCheck(self, timeout, target_pos=None, target_rot=None,
                             pos_thresh=None, vel_thresh=None, rot_thresh=None):
        return self.convergenceCheck(timeout, self.poseThresholdCheck, target_pos, target_rot,
                                     pos_thresh, vel_thresh, rot_thresh)

    def poseThresholdCheck(self, target_pos=None, target_rot=None,
                           pos_thresh=None, vel_thresh=None, rot_thresh=None):

        if pos_thresh is None:
            pos_thresh = self.default_pos_thresh
        if isinstance(pos_thresh, float):
            pos_thresh = [pos_thresh] * 3

        if rot_thresh is None:
            rot_thresh = self.default_rot_thresh
        if isinstance(rot_thresh, float):
            rot_thresh = [rot_thresh] * 3

        if vel_thresh is None:
            vel_thresh = self.default_vel_thresh
        if isinstance(vel_thresh, float):
            vel_thresh = [vel_thresh] * 3

        if target_pos is None:
            target_pos = self.getCogPos()
            pos_thresh = np.array([1e6] * 3)
            vel_thresh = np.array([1e6] * 3)

        if target_rot is None:
            # assume the coordinate axes of baselink are identical to those of CoG
            target_rot = self.getBaseRot()
            rot_thresh = np.array([1e6] * 3)
        if len(target_rot) == 3:
            target_rot = quaternion_from_euler(*target_rot)

        current_pos = self.getCogPos()
        current_vel = self.getCogLinVel()
        current_rot = self.getBaseRot()

        delta_pos = np.asarray(target_pos) - current_pos
        delta_vel = current_vel
        delta_rot = euler_from_quaternion(
            quaternion_multiply(quaternion_inverse(current_rot), target_rot))

        self.logger.info(
            'pose diff: {}, rot: {}, vel: {};  target pose: pos: {}, rot: {}; '
            'current pose: pos: {}, rot: {}, vel: {}'.format(
                delta_pos, delta_rot, delta_vel, target_pos, target_rot,
                current_pos, current_rot, current_vel),
            throttle_duration_sec=1.0)

        return (np.all(np.abs(delta_pos) < pos_thresh) and
                np.all(np.abs(delta_rot) < rot_thresh) and
                np.all(np.abs(delta_vel) < vel_thresh))

    # special rotation function for DRAGON like robots
    def rotateCog(self, roll, pitch):
        msg = Vector3Stamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.vector.x = float(roll)
        msg.vector.y = float(pitch)
        self.final_rot_pub.publish(msg)

    # ---- joints --------------------------------------------------------------

    def setJointAngle(self, target_joint_names, target_joint_angles, thresh=0.05, timeout=-1):

        if self.flight_state != self.HOVER_STATE and self.flight_state != self.ARM_OFF_STATE:
            self.logger.error(
                '[Send Joint] the current flight state({}) does not allow joint motion'.format(
                    self.flight_state))
            return False

        if len(target_joint_names) != len(target_joint_angles):
            self.logger.error(
                '[Send Joint] the size of joint names {} and angles {} are not same'.format(
                    len(target_joint_names), len(target_joint_angles)))
            return False

        for name in target_joint_names:
            if name not in self.joint_state.name:
                self.logger.error('set joint angle: cannot find {}'.format(name))
                return False

        target_joint_state = JointState()
        target_joint_state.name = list(target_joint_names)
        target_joint_state.position = [float(a) for a in target_joint_angles]
        self.joint_ctrl_pub.publish(target_joint_state)

        return self.jointConvergenceCheck(timeout, target_joint_names, target_joint_angles, thresh)

    def jointConvergenceCheck(self, timeout, target_joint_names, target_joint_angles, thresh):
        return self.convergenceCheck(timeout, self.jointThresholdCheck,
                                     target_joint_names, target_joint_angles, thresh)

    def jointThresholdCheck(self, target_joint_names, target_joint_angles, thresh):

        if len(target_joint_names) != len(target_joint_angles):
            self.logger.error(
                '[Send Joint] the sizes of joint names {} and angles {} are not same'.format(
                    len(target_joint_names), len(target_joint_angles)))
            return False

        index_map = []
        for name in target_joint_names:
            try:
                index_map.append(self.joint_state.name.index(name))
            except ValueError:
                self.logger.error('set joint angle: cannot find {}'.format(name))
                return False

        delta_ang = [target - self.joint_state.position[index]
                     for index, target in zip(index_map, target_joint_angles)]

        self.logger.info('delta angle: {}'.format(delta_ang), throttle_duration_sec=1.0)

        return np.all(np.abs(delta_ang) < thresh)

    def setJointTorque(self, state):
        if not self.set_joint_torque_client.wait_for_service(timeout_sec=1.0):
            self.logger.error('joints/torque_enable service is not available')
            return False

        req = SetBool.Request()
        req.data = state
        future = self.set_joint_torque_client.call_async(req)

        # The executor is already spinning this node on its own thread, so wait
        # on the future rather than spinning it again - spinning a node twice
        # deadlocks.
        deadline = self.now() + 5.0
        while rclpy.ok() and not future.done() and self.now() < deadline:
            self.sleep(0.05)

        if not future.done():
            self.logger.error('joints/torque_enable call timed out')
            return False
        return True
