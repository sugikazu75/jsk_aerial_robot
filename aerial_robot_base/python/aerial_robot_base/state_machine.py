"""ROS2 counterpart of src/aerial_robot_base/state_machine.py.

Same states, same `execute(userdata)` protocol, same outcomes. The one
structural change is that SMACH is gone: `smach` and `smach_ros` have no humble
release, and what this stack asks of them is small - a linear sequence of states
that each return 'succeeded' or 'preempted', plus a userdata dict passed along.
That is `State` and `StateMachine` below, in forty lines. The introspection
server has no replacement and no user here; a task that wants one can log.

Everything else is the ROS1 file with rospy swapped for the node the
RobotInterface already owns, so the states keep reading like their originals.
"""

import numpy as np
import rclpy

from sensor_msgs.msg import Joy
from std_msgs.msg import Empty


class State(object):
    """A step with a set of outcomes, as smach.State was."""

    def __init__(self, outcomes=None):
        self.outcomes = outcomes or []

    def execute(self, userdata):
        raise NotImplementedError


class UserData(dict):
    """smach's userdata was attribute-addressed; keep that so states read the same."""

    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError:
            raise AttributeError(name)

    def __setattr__(self, name, value):
        self[name] = value


class StateMachine(object):
    """A linear sequence of states.

    smach let a state name its successor per outcome. Every machine in this
    stack is a straight line that stops on 'preempted', so the sequence is the
    transition table.
    """

    def __init__(self, states=None):
        self.states = list(states or [])
        self.userdata = UserData(flags={}, extra={})

    def add(self, label, state):
        self.states.append((label, state))

    def execute(self, logger=None):
        for label, state in self.states:
            if logger is not None:
                logger.info('state machine: entering {}'.format(label))
            outcome = state.execute(self.userdata)
            if outcome != 'succeeded':
                if logger is not None:
                    logger.warn('state machine: {} returned {}'.format(label, outcome))
                return outcome
        return 'succeeded'


class BaseState(State):
    def __init__(self, robot, outcomes=None):
        State.__init__(self, outcomes or ['succeeded', 'preempted'])
        self.robot = robot
        self.node = robot.node
        self.logger = robot.logger

    def param(self, name, default):
        """A '~<prefix>/<name>' rosparam. ROS2 parameters are flat and dotted."""
        name = name.replace('/', '.')
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, default)
        value = self.node.get_parameter(name).value
        return default if value is None else value

    def hold(self, t, flags):

        if not flags.get('interfere_mode', False):
            self.robot.sleep(t)
            return

        message = '\n\n Please press "L1" and "R1" at the same time to proceed the state \n'
        self.logger.info('\033[32m' + message + '\033[0m')

        while rclpy.ok():
            if flags.get('interfering'):
                flags['interfering'] = False  # reset for subsequent state
                break
            self.robot.sleep(0.1)


# trigger to start state machine
class Start(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot)

        self.flags = {'interfere_mode': False, 'interfering': False}

        self.task_start = False
        self.task_start_sub = self.node.create_subscription(
            Empty, '/task_start', self.taskStartCallback, 1)
        self.joy_sub = self.node.create_subscription(
            Joy, self.robot.robot_ns + '/joy', self.joyCallback, 1)

    def taskStartCallback(self, msg):
        self.task_start = True

    def joyCallback(self, msg):

        interfere_flag = False

        # PS4 Joy Controller
        if len(msg.axes) == 14 and len(msg.buttons) == 14:
            if msg.buttons[4] == 1 and msg.buttons[5] == 1:
                interfere_flag = True

        # RoG1 Controller
        if len(msg.axes) == 8 and len(msg.buttons) == 11:
            if msg.buttons[4] == 1 and msg.buttons[5] == 1:
                interfere_flag = True

        if interfere_flag and not self.flags['interfere_mode']:
            self.logger.info('Enter interfere mode')
            self.flags['interfere_mode'] = True

        self.flags['interfering'] = interfere_flag

    def execute(self, userdata):

        message = \
            '\n\n  Please run following command to start the state machine: \n' + \
            '  "$ ros2 topic pub -1 /task_start std_msgs/msg/Empty {}" \n \n' + \
            '  Or you can press "L1" and "R1" at the same time to enter interfere mode, \n' + \
            '  which need to use controller like PS4 to manually proceed the state. \n'
        self.logger.info('\033[32m' + message + '\033[0m')

        userdata.flags = self.flags

        while not self.task_start:
            self.robot.sleep(0.1)

            if userdata.flags['interfere_mode']:
                self.logger.info(type(self).__name__ + ': enter the inteferece mode')
                userdata.flags['interfering'] = False  # reset for subsequent state
                break

            if not rclpy.ok():
                self.logger.info('shutdown ros')
                return 'preempted'

        self.logger.info('start task')
        return 'succeeded'


# the template state for arm, takeoff, land
class SingleCommandState(BaseState):
    def __init__(self, robot, prefix, func, start_flight_state, target_flight_state,
                 timeout, hold_time):
        BaseState.__init__(self, robot)

        self.prefix = prefix
        self.func = func
        self.start_flight_state = start_flight_state
        self.target_flight_state = target_flight_state
        self.hold_time = self.param(self.prefix + '/hold_time', hold_time)
        self.timeout = self.param(self.prefix + '/timeout', timeout)

    def execute(self, userdata):

        flight_state = self.robot.getFlightState()

        if flight_state != self.start_flight_state:
            self.logger.warn(
                type(self).__name__ +
                ': the robot state ({}) is not at the start_flight_state ({}). preempted!'.format(
                    flight_state, self.start_flight_state))
            return 'preempted'

        self.logger.info(type(self).__name__ + ': start {}'.format(self.prefix))
        self.func()

        start_t = self.robot.now()
        while self.robot.now() < start_t + self.timeout:

            if self.robot.getFlightState() == self.target_flight_state:
                self.logger.info(
                    type(self).__name__ + ': robot succeed to {}!'.format(self.prefix))
                self.hold(self.hold_time, userdata.flags)
                return 'succeeded'

            if not rclpy.ok():
                self.logger.info('shutdown ros')
                return 'preempted'

            # The ROS1 loop spun here implicitly; without a yield this would be
            # a busy wait that starves the executor thread.
            self.robot.sleep(0.05)

        self.logger.warn(
            type(self).__name__ + ': timeout ({}sec). preempted!'.format(self.timeout))
        return 'preempted'


class Arm(SingleCommandState):
    def __init__(self, robot):
        SingleCommandState.__init__(self, robot, 'arm', robot.start,
                                    robot.ARM_OFF_STATE, robot.ARM_ON_STATE, 2.0, 2.0)

    def execute(self, userdata):
        if self.robot.getFlightState() == self.robot.HOVER_STATE:
            self.logger.info(type(self).__name__ + ': robot already hovers, skip')
            return 'succeeded'
        return SingleCommandState.execute(self, userdata)


class Takeoff(SingleCommandState):
    def __init__(self, robot):
        SingleCommandState.__init__(self, robot, 'takeoff', robot.takeoff,
                                    robot.ARM_ON_STATE, robot.HOVER_STATE, 30.0, 2.0)

    def execute(self, userdata):
        if self.robot.getFlightState() == self.robot.HOVER_STATE:
            self.logger.info(type(self).__name__ + ': robot already hovers, skip')
            return 'succeeded'
        return SingleCommandState.execute(self, userdata)


class Land(SingleCommandState):
    def __init__(self, robot):
        SingleCommandState.__init__(self, robot, 'land', robot.land,
                                    robot.HOVER_STATE, robot.ARM_OFF_STATE, 20.0, 0.0)


# waypoint
class WayPoint(BaseState):
    def __init__(self, robot, prefix='waypoint', waypoints=None, timeout=30.0, hold_time=1.0):
        BaseState.__init__(self, robot)

        self.waypoints = self.param(prefix + '/waypoints', waypoints or [])
        self.timeout = self.param(prefix + '/timeout', timeout)
        self.hold_time = self.param(prefix + '/hold_time', hold_time)
        self.pos_thresh = self.param(prefix + '/pos_thresh', 0.1)
        self.vel_thresh = self.param(prefix + '/vel_thresh', 0.05)
        self.yaw_thresh = self.param(prefix + '/yaw_thresh', 0.1)

    def execute(self, userdata):

        flight_state = self.robot.getFlightState()

        if flight_state != self.robot.HOVER_STATE:
            self.logger.warn(
                type(self).__name__ +
                ': the robot state ({}) is not at HOVER_STATE. preempted!'.format(flight_state))
            return 'preempted'

        if len(self.waypoints) == 0:
            self.logger.warn(type(self).__name__ + ': the waypoints is empty. preempted')
            return 'preempted'

        for i, waypoint in enumerate(self.waypoints):

            if len(waypoint) == 3:
                # only position
                ret = self.robot.goPos(pos=waypoint, pos_thresh=self.pos_thresh,
                                       vel_thresh=self.vel_thresh, timeout=self.timeout)
            elif len(waypoint) == 4:
                # position + yaw
                ret = self.robot.goPosYaw(pos=waypoint[:3], yaw=waypoint[-1],
                                          pos_thresh=self.pos_thresh, vel_thresh=self.vel_thresh,
                                          yaw_thresh=self.yaw_thresh, timeout=self.timeout)
            else:
                self.logger.warn(
                    type(self).__name__ +
                    ': the format of waypoint {} is not supported, the size should be either '
                    '3 and 4. preempted!'.format(waypoint))
                return 'preempted'

            if ret:
                self.logger.info(
                    type(self).__name__ +
                    ': robot succeeds to reach the {}th waypoint [{}]'.format(i + 1, waypoint))
                self.hold(self.hold_time, userdata.flags)
            else:
                self.logger.warn(
                    type(self).__name__ +
                    ': robot cannot reach {}. preempted'.format(waypoint))
                return 'preempted'

        return 'succeeded'


class CircleTrajectory(BaseState):
    def __init__(self, robot, period=20.0, radius=1.0, init_theta=0.0, yaw=True, loop=1,
                 timeout=30.0, hold_time=2.0):
        BaseState.__init__(self, robot)

        self.period = self.param('circle/period', period)
        self.radius = self.param('circle/radius', radius)
        self.init_theta = self.param('circle/init_theta', init_theta)
        self.yaw = self.param('circle/yaw', yaw)
        self.loop = self.param('circle/loop', loop)
        self.hold_time = self.param('circle/hold_time', hold_time)

        self.omega = 2 * np.pi / self.period
        self.velocity = self.omega * self.radius

        self.nav_rate = 1 / self.param('nav_rate', 20.0)  # hz

    def execute(self, userdata):

        current_pos = self.robot.getCogPos()
        center_pos_x = current_pos[0] - np.cos(self.init_theta) * self.radius
        center_pos_y = current_pos[1] - np.sin(self.init_theta) * self.radius
        center_pos_z = current_pos[2]

        init_yaw = self.robot.getCogRPY()[2]

        self.logger.info('the center position is [{}, {}]'.format(center_pos_x, center_pos_y))

        loop = 0
        cnt = 0

        while loop < self.loop:

            if self.robot.flight_state != self.robot.HOVER_STATE:
                self.logger.error('[Circle Motion] the robot is not hovering, preempt!')
                return 'preempted'

            theta = self.init_theta + cnt * self.nav_rate * self.omega

            pos = [center_pos_x + np.cos(theta) * self.radius,
                   center_pos_y + np.sin(theta) * self.radius,
                   center_pos_z]
            vel = [-np.sin(theta) * self.velocity, np.cos(theta) * self.velocity, 0]

            if self.yaw:
                yaw = init_yaw + cnt * self.nav_rate * self.omega
                self.robot.navigate(pos=pos, rot=[0, 0, yaw], lin_vel=vel,
                                    ang_vel=[0, 0, self.omega])
            else:
                self.robot.navigate(pos=pos, lin_vel=vel)

            cnt += 1

            if cnt == self.period // self.nav_rate:
                loop += 1
                cnt = 0

            self.robot.sleep(self.nav_rate)

        # stop
        self.robot.navigate(lin_vel=[0, 0, 0], ang_vel=[0, 0, 0])
        self.hold(self.hold_time, userdata.flags)

        return 'succeeded'


# Joint
class FormCheck(BaseState):
    def __init__(self, robot, prefix='form_check', joint_names=None, joint_angles=None,
                 thresh=0.02, timeout=10.0):
        BaseState.__init__(self, robot)

        self.target_joint_names = self.param(prefix + '/joint_names', joint_names or [])
        self.target_joint_angles = self.param(prefix + '/joint_angles', joint_angles or [])
        self.timeout = self.param(prefix + '/timeout', timeout)
        self.thresh = self.param(prefix + '/angle_thresh', thresh)

    def execute(self, userdata):

        ret = self.robot.jointConvergenceCheck(self.timeout, self.target_joint_names,
                                               self.target_joint_angles, self.thresh)

        if ret:
            self.logger.info(
                type(self).__name__ +
                ': robot succeed to convernge to the target joints {}:{}!'.format(
                    self.target_joint_names, self.target_joint_angles))
            return 'succeeded'

        self.logger.warn(
            type(self).__name__ + ': timeout ({}sec). preempted!'.format(self.timeout))
        return 'preempted'


class Transform(BaseState):
    def __init__(self, robot, prefix='transform', joint_names=None, joint_trajectory=None,
                 thresh=0.05, timeout=10.0, hold_time=2.0):
        BaseState.__init__(self, robot)

        self.target_joint_names = self.param(prefix + '/joint_names', joint_names or [])
        self.target_joint_trajectory = self.param(
            prefix + '/joint_trajectory', joint_trajectory or [])
        self.timeout = self.param(prefix + '/timeout', timeout)
        self.hold_time = self.param(prefix + '/hold_time', hold_time)
        self.thresh = self.param(prefix + '/angle_thresh', thresh)

    def execute(self, userdata):

        flight_state = self.robot.getFlightState()

        if flight_state != self.robot.HOVER_STATE:
            self.logger.warn(
                type(self).__name__ +
                ': the robot state ({}) is not at HOVER_STATE. preempted!'.format(flight_state))
            return 'preempted'

        if len(self.target_joint_trajectory) == 0:
            self.logger.warn(type(self).__name__ + ': the joint trajectory is empty. preempted')
            return 'preempted'

        for i, target_angles in enumerate(self.target_joint_trajectory):
            ret = self.robot.setJointAngle(self.target_joint_names, target_angles,
                                           self.thresh, self.timeout)

            if ret:
                self.logger.info(
                    type(self).__name__ +
                    ': robot succeed to convernge to the {} th target joints {}:{}!'.format(
                        i + 1, self.target_joint_names, target_angles))
                self.hold(self.hold_time, userdata.flags)
            else:
                self.logger.warn(
                    type(self).__name__ +
                    ': timeout ({}sec), fail to reach the {} th target joints. preempted!'.format(
                        self.timeout, i + 1))
                return 'preempted'

        return 'succeeded'


# Joint + Pose
class TransformWithPose(BaseState):
    def __init__(self, robot, prefix='motion', joint_names=None, joint_trajectory=None,
                 pos_trajectory=None, rot_trajectory=None, joint_thresh=0.05, pos_thresh=0.1,
                 rot_thresh=0.1, timeout=10.0, hold_time=2.0, rotate_cog=False):
        BaseState.__init__(self, robot)

        self.target_joint_names = self.param(prefix + '/joint_names', joint_names or [])
        self.target_joint_trajectory = self.param(
            prefix + '/joint_trajectory', joint_trajectory or [])
        self.target_pos_trajectory = self.param(prefix + '/pos_trajectory', pos_trajectory or [])
        self.target_rot_trajectory = self.param(prefix + '/rot_trajectory', rot_trajectory or [])
        self.timeout = self.param(prefix + '/timeout', timeout)
        self.hold_time = self.param(prefix + '/hold_time', hold_time)
        self.joint_thresh = self.param(prefix + '/joint_thresh', joint_thresh)
        self.pos_thresh = self.param(prefix + '/pos_thresh', pos_thresh)
        self.rot_thresh = self.param(prefix + '/rot_thresh', rot_thresh)
        # special rotation for CoG
        self.rotate_cog = self.param(prefix + '/rotate_cog', rotate_cog)

    def execute(self, userdata):

        flight_state = self.robot.getFlightState()

        if flight_state != self.robot.HOVER_STATE:
            self.logger.warn(
                type(self).__name__ +
                ': the robot state ({}) is not at HOVER_STATE. preempted!'.format(flight_state))
            return 'preempted'

        if len(self.target_joint_trajectory) == 0:
            self.logger.warn(type(self).__name__ + ': the joint trajectory is empty. preempted')
            return 'preempted'

        for i, target_angles in enumerate(self.target_joint_trajectory):

            # send target position
            target_pos = None
            if len(self.target_pos_trajectory) > 0:

                if len(self.target_pos_trajectory) != len(self.target_joint_trajectory):
                    self.logger.warn(
                        type(self).__name__ +
                        ': the size of joint trajectory and that of pos is not equal. preempted!')
                    return 'preempted'

                target_pos = self.target_pos_trajectory[i]
                self.robot.goPos(target_pos, timeout=0)

            # send target rotation
            target_rot = None
            if len(self.target_rot_trajectory) > 0:
                if len(self.target_rot_trajectory) != len(self.target_joint_trajectory):
                    self.logger.warn(
                        type(self).__name__ +
                        ': the size of joint trajectory and that of rot is not equal. preempted!')
                    return 'preempted'

                target_rot = self.target_rot_trajectory[i]

                if self.rotate_cog:

                    if len(target_rot) > 3 or len(target_rot) < 2:
                        self.logger.warn(
                            type(self).__name__ +
                            ': the size of target rot should be 2 or 3 for special cog ratate '
                            'mode. preempted!')
                        return 'preempted'

                    self.robot.rotateCog(target_rot[0], target_rot[1])

                    if len(target_rot) == 3:
                        self.robot.rotateYaw(target_rot[2], timeout=0)
                    else:
                        # fill the yaw element for subsequent pose convergence check
                        target_rot.append(self.robot.getCogRPY()[2])

                else:
                    self.robot.rotate(target_rot, timeout=0)

            start_time = self.robot.now()

            self.logger.info(
                type(self).__name__ +
                ': start to change the joint motion {} with pose of [{}, {}]'.format(
                    target_angles, target_pos, target_rot))
            ret = self.robot.setJointAngle(self.target_joint_names, target_angles,
                                           self.joint_thresh, self.timeout)

            if ret:
                self.logger.info(
                    type(self).__name__ +
                    ': robot succeed to convernge to the {} th target joints {}:{}!'.format(
                        i + 1, self.target_joint_names, target_angles))
            else:
                self.logger.warn(
                    type(self).__name__ +
                    ': timeout ({}sec), fail to reach the {} th target joints. preempted!'.format(
                        self.timeout, i + 1))
                return 'preempted'

            # reset
            timeout = start_time + self.timeout - self.robot.now()

            ret = self.robot.poseConvergenceCheck(timeout, target_pos, target_rot,
                                                  self.pos_thresh, rot_thresh=self.rot_thresh)

            if not ret:
                self.logger.warn(
                    type(self).__name__ +
                    ': timeout ({}sec), fail to reach the {} th target pose. preempted!'.format(
                        self.timeout, i + 1))
                return 'preempted'

            self.logger.info(
                type(self).__name__ +
                ': robot succeed to convernge to the {} th target pose {}: [{}, {}]!'.format(
                    i + 1, self.target_joint_names, target_pos, target_rot))

            self.hold(self.hold_time, userdata.flags)

        return 'succeeded'
