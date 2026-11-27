"""The control loop of the hanging whole-body IK (hanging_ik.py) shared by hanging_teleop.py and
hand_accuracy_test.py: the IK is started at the posture of the robot, and each cycle the target it
follows moves toward the requested one (speed and lag limited), one IK step is solved and its result
is sent to the robot (the CoG and the body attitude to the navigation, the arm joints to the servos).

The IK always keeps the conditions of the hanging (the hook point fixed where it was at the start, the
angle of the hook, the CoM below the hook), whether the hook contact of the whole-body controller is on
or not: switch_contact() switches it.
"""

import dataclasses
import math
import threading

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from gimbalrotor.hanging_ik import HangingIK, HangingIKConfig

IK_KEYS = {f.name for f in dataclasses.fields(HangingIKConfig)} - {"front_joints", "rear_joints"}
FOLLOWER_KEYS = IK_KEYS | {
    "joint_group",
    "rate",
    "track_direction",
    "tip_speed",
    "tip_rotation_speed",
    "tip_lag",
    "tip_angle_lag",
    "root_speed",
    "root_rotation_speed",
    "joint_speed",
    "ik_iterations",
}


def toward(current, goal, step):
    """current moved toward goal by at most step (norm)."""
    delta = np.asarray(goal, float) - current
    norm = np.linalg.norm(delta)
    return current + (delta if norm <= step else delta * step / norm)


class JointStates:
    """The latest angle of each joint on <ns>/joint_states (merged by name: several publishers)."""

    def __init__(self, robot_ns):
        self.lock = threading.Lock()
        self.angles = {}
        rospy.Subscriber(robot_ns + "/joint_states", JointState, self._callback, queue_size=10)

    def _callback(self, msg):
        with self.lock:
            self.angles.update(zip(msg.name, msg.position))

    def get(self):
        with self.lock:
            return dict(self.angles)

    def wait(self, names, timeout=5.0):
        deadline = rospy.get_time() + timeout
        while not rospy.is_shutdown() and not set(names) <= set(self.get()):
            if rospy.get_time() > deadline:
                raise RuntimeError("no joint states of %s" % sorted(set(names) - set(self.get())))
            rospy.sleep(0.05)
        return self.get()


def switch_contact(robot_ns, name, on):
    """Switch the contact of the whole-body controller on or off; False if it cannot."""
    service = "%s/whole_body/contacts/%s/activate" % (robot_ns, name)
    try:
        rospy.wait_for_service(service, timeout=1.0)
        rospy.loginfo("contact %s: %s", name, rospy.ServiceProxy(service, SetBool)(on).message)
        return True
    except (rospy.ROSException, rospy.ServiceException) as e:
        rospy.logwarn(
            "cannot switch the contact %s %s (%s): is the whole-body controller running "
            "(bringup whole_body:=true)?",
            name,
            "on" if on else "off",
            e,
        )
        return False


def frame_configuration(ik, data, index):
    """The configuration (pinocchio q of the IK model) of frame index of a trajectory file (its root pose
    and joints)."""
    frame = data["frames"][index]
    q = ik.configuration(np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]), dict(zip(data["joint_names"], frame["joints"])))
    q[:7] = frame["root"]
    return q


class HangingFollower:
    def __init__(self, robot, p, start=None):
        """Wait for the hover and the state of the robot, and start the IK: at start, a function of the
        IK returning a configuration (e.g. a frame of the plan: the hook point on the bar where it was
        planned), or at the posture of the robot now (None). The hook point, the plane and the starting
        posture of the IK are those of that configuration."""
        self.robot, self.p = robot, p
        rospy.loginfo("waiting for the hover state and the odometry")
        if not robot.waitForState(robot.HOVER_STATE):
            raise rospy.ROSInterruptException("shutdown")
        while not rospy.is_shutdown() and robot.getBaseOdom() is None:
            rospy.sleep(0.1)
        config = HangingIKConfig(**{k: (tuple(p[k]) if isinstance(p[k], list) else p[k]) for k in IK_KEYS})
        self.ik = HangingIK(rospy.get_param("robot_description"), config)
        self.joints = JointStates(robot.robot_ns)
        joints = self.joints.wait(self.ik.joint_names)
        q = start(self.ik) if start is not None else self.ik.configuration(robot.getBasePos(), robot.getBaseRot(), joints)
        self.x = self.ik.start(q)
        self.f = self.ik.features(self.x)
        self.result = None
        self.dt = 1.0 / float(p["rate"])
        self.step_bound = (
            np.array([p["root_speed"]] * 2 + [p["root_rotation_speed"]] + [p["joint_speed"]] * 6, float) * self.dt
        )
        self.tip, self.angle = self.f.tip.copy(), self.f.tip_angle  # what the IK follows
        self.cog_prev = None
        self.cog = self.rot = self.joint_targets = None  # the last targets sent

    def hold_hand(self):
        """The target the IK follows: where the hand of the IK is now."""
        self.tip, self.angle = self.f.tip.copy(), self.f.tip_angle

    def reached(self, goal, goal_angle, tolerance=1e-4, angle_tolerance=1e-3):
        """The target the IK follows is at the requested one (not that the hand is there)."""
        angle_ok = (not self.p["track_direction"]) or abs(math.remainder(goal_angle - self.angle, 2 * math.pi)) < angle_tolerance
        return np.linalg.norm(np.asarray(goal) - self.tip) < tolerance and angle_ok

    def step(self, goal, goal_angle):
        """One cycle toward the requested hand (u, z) and angle: the IK and the targets sent. False if the
        robot is not hovering any more."""
        p, f, dt = self.p, self.f, self.dt
        # within the lag of the hand, then toward the goal
        if np.linalg.norm(self.tip - f.tip) > p["tip_lag"]:
            self.tip = toward(f.tip, self.tip, p["tip_lag"])
        self.tip = toward(self.tip, goal, p["tip_speed"] * dt)
        lag = math.remainder(self.angle - f.tip_angle, 2 * math.pi)
        self.angle = f.tip_angle + max(-p["tip_angle_lag"], min(p["tip_angle_lag"], lag))
        turn = math.remainder(goal_angle - self.angle, 2 * math.pi)
        self.angle += max(-p["tip_rotation_speed"] * dt, min(p["tip_rotation_speed"] * dt, turn))

        result = self.ik.solve(
            self.x, self.tip, self.angle if p["track_direction"] else None, self.step_bound, int(p["ik_iterations"])
        )
        self.result, self.x, self.f = result, result.p, result.features
        cog, rot, joints = self.ik.targets(self.x)
        vel = np.zeros(3) if self.cog_prev is None else (cog - self.cog_prev) / dt
        self.cog_prev = cog
        if not self.robot.sendCogTarget(cog, vel, rot):
            rospy.logerr("not hovering any more (state %s): stopped", self.robot.getFlightState())
            return False
        self.robot.sendJointTarget(self.ik.joint_names, joints)
        self.cog, self.rot, self.joint_targets = cog, rot, joints

        if result.hook_soft:
            rospy.logwarn_throttle(1.0, "the hook cannot be kept exactly (%.1f mm off)" % (1e3 * result.hook_error))
        if result.blocked:
            rospy.logwarn_throttle(1.0, "stopped by a self-collision of %s" % result.blocked)
        if np.linalg.norm(self.tip - self.f.tip) > 0.9 * p["tip_lag"]:  # the hand does not follow
            rospy.logwarn_throttle(
                2.0,
                "the hand stops %.1f cm from the target (out of reach, or at a limit): body "
                "pitch %.2f rad, rear arm torques %s Nm"
                % (1e2 * np.linalg.norm(np.asarray(goal) - self.f.tip), self.x[2], self.f.torque.round(2)),
            )
        return True

    def measure(self):
        """The hand, the hook, the CoM and the pitch of the robot as estimated (the baselink odometry and
        the joint states), in the plane of the IK (HangingIK.measure)."""
        q = self.ik.configuration(self.robot.getBasePos(), self.robot.getBaseRot(), self.joints.get())
        return self.ik.measure(q)
