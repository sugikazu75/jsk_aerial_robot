"""Following a trajectory file with
the flight controller, through GimbalrotorInterface: the CoG and the attitude to the navigation, the
joints to the servos. Contacts are not taken into account: the controller tracks the targets with its
own feedback and allocation (the thrust and the gimbals of the file are not used).

  Targets           the CoG, the body attitude and the servo joint angles of each frame
  TrajectoryPlayer  sends them: a stretch of the file in time, a hold until the robot converges, a
                    smooth move from where the robot is to a frame
  load_params       the parameters of a YAML file, overridden by the private parameters of the node
  setup, take_off   what follow_trajectory.py and perching_task.py share
"""

import json
import math
import os
from xml.etree import ElementTree

import numpy as np
import pinocchio as pin
import rospkg
import rospy
import yaml

from gimbalrotor.robot_interface import GimbalrotorInterface


def resolve(path):
    """A path that may start with package://<package>/."""
    if path.startswith("package://"):
        package, _, rest = path[len("package://") :].partition("/")
        return os.path.join(rospkg.RosPack().get_path(package), rest)
    return path


def load_params(default_path, keys, config_param="~config"):
    """All the parameters in keys from the YAML file of config_param (default_path if not set), each one
    overridden by the private parameter of its name if set; an unknown or a missing key is an error."""
    path = rospy.get_param(config_param, default_path)
    with open(resolve(path)) as f:
        params = yaml.safe_load(f) or {}
    missing, unknown = sorted(set(keys) - set(params)), sorted(set(params) - set(keys))
    if missing or unknown:
        raise RuntimeError("%s: unknown keys %s, missing keys %s" % (path, unknown, missing))
    for key in keys:
        if rospy.has_param("~" + key):
            params[key] = rospy.get_param("~" + key)
    rospy.loginfo("parameters from %s: %s", path, params)
    return params


from aerial_motion_planning.targets import TrajectoryTargets, rest_to_rest


class Targets(TrajectoryTargets):
    """Gimbalrotor trajectory targets; actuator gimbals are not servo commands."""

    def __init__(self, data, urdf):
        baselink = ElementTree.fromstring(urdf).find("baselink")
        if baselink is None:
            raise RuntimeError("robot_description has no <baselink name=...>")
        super().__init__(
            data, urdf, body_frame=baselink.get("name"), servo_filter=lambda name: not name.startswith("gimbal")
        )
        # [s] of the file where named steps begin (plan_perching_motion.py: "release")
        self.events = {k: float(v) for k, v in data.get("events", {}).items()}


class TrajectoryPlayer:
    """Sends the targets of a Targets to the robot (a GimbalrotorInterface) at rate [Hz]. Every motion
    returns False, and stops sending, if the robot leaves the hover state."""

    def __init__(self, robot, targets, rate=50.0):
        if rate <= 0:
            raise ValueError("target rate must be positive")
        self.robot, self.targets, self.rate = robot, targets, rate

    def current_joints(self, fallback):
        state = self.robot.getJointState()
        current = dict(zip(state.name, state.position))
        return np.array([current.get(n, a) for n, a in zip(self.targets.servo_names, fallback)])

    def send(self, target):
        sent = self.robot.sendCogTarget(target["cog"], target["cog_vel"], target["rot"])
        if sent:
            self.robot.sendJointTarget(self.targets.servo_names, target["joints"])
        return sent

    def run(self, target_at, duration):
        """Send target_at(t) for t up to duration [s]."""
        start = rospy.Time.now()
        timer = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start).to_sec()
            if not self.send(target_at(min(t, duration))):
                rospy.logerr("not hovering any more (state %s): stopped", self.robot.getFlightState())
                return False
            if t >= duration:
                return True
            timer.sleep()
        return False

    def play(self, start, end, speed=1.0):
        """The file from time start to end [s] at speed (the velocity scaled by it)."""
        if speed <= 0:
            raise ValueError("playback speed must be positive")
        if end <= start:
            return True
        rospy.loginfo("playing %.2f..%.2f s of the trajectory at speed %.2f", start, end, speed)

        def target(t):
            here = self.targets.at(min(start + t * speed, end))
            return dict(here, cog_vel=speed * here["cog_vel"])

        return self.run(target, (end - start) / speed)

    def play_back(self, start, end, speed=1.0):
        """The file backwards from time start down to end [s] at speed (the velocity reversed and scaled);
        the frame at end held at rest at the end."""
        if speed <= 0:
            raise ValueError("playback speed must be positive")
        if start <= end:
            return True
        rospy.loginfo("playing %.2f..%.2f s of the trajectory backwards at speed %.2f", start, end, speed)
        duration = (start - end) / speed
        last = self.targets.still(end)

        def target(t):
            if t >= duration:
                return last
            here = self.targets.at(start - t * speed)
            return dict(here, cog_vel=-speed * here["cog_vel"])

        return self.run(target, duration)

    def hold(self, t, duration):
        """The frame at time t, at rest, for duration [s]."""
        target = self.targets.still(t)
        return self.run(lambda _: target, duration)

    def move_to(self, t, cog_speed, joint_speed):
        """From where the robot is to the frame at time t, at rest: first the CoG in a straight line at
        cog_speed [m/s] with the joints held, then the joints at joint_speed [rad/s] with the CoG held."""
        if cog_speed <= 0 or joint_speed <= 0:
            raise ValueError("move_to speeds must be positive")
        goal = self.targets.still(t)
        cog0 = self.robot.getCogPos().copy()
        joints0 = self.current_joints(goal["joints"])

        def ramp(duration, cog_a, cog_b, joints_a, joints_b):
            def target(time):
                s, rate = rest_to_rest(time / duration)
                return dict(
                    goal,
                    cog=cog_a + s * (cog_b - cog_a),
                    cog_vel=rate / duration * (cog_b - cog_a),
                    joints=joints_a + s * (joints_b - joints_a),
                )

            return target

        duration = max(np.linalg.norm(goal["cog"] - cog0) / cog_speed, 1.0)
        rospy.loginfo("CoG %s -> %s in %.1f s", cog0.round(3), goal["cog"].round(3), duration)
        if not self.run(ramp(duration, cog0, goal["cog"], joints0, joints0), duration):
            return False
        duration = max(np.max(np.abs(goal["joints"] - joints0), initial=0.0) / joint_speed, 1.0)
        rospy.loginfo("joints %s -> %s in %.1f s", joints0.round(2), goal["joints"].round(2), duration)
        return self.run(ramp(duration, goal["cog"], goal["cog"], joints0, goal["joints"]), duration)

    def converge(self, t, pos_tol, vel_tol, joint_tol, timeout):
        """Hold the frame at time t until the CoG is within pos_tol [m] of it and slower than vel_tol
        [m/s], and the joints within joint_tol [rad]: True; False if not in timeout [s] (still holding
        it) or if the robot leaves the hover state."""
        target = self.targets.still(t)
        start = rospy.get_time()
        timer = rospy.Rate(self.rate)
        rospy.loginfo("waiting for the CoG within %.3f m and the joints within %.3f rad", pos_tol, joint_tol)
        while not rospy.is_shutdown():
            if not self.send(target):
                rospy.logerr("not hovering any more (state %s): stopped", self.robot.getFlightState())
                return False
            pos_error = np.linalg.norm(self.robot.getCogPos() - target["cog"])
            speed = np.linalg.norm(self.robot.getCogLinVel())
            state = self.robot.getJointState()
            measured = dict(zip(state.name, state.position)) if state is not None else {}
            missing = [name for name in self.targets.servo_names if name not in measured]
            joint_error = (
                float("inf")
                if missing
                else np.max(
                    np.abs(np.array([measured[name] for name in self.targets.servo_names]) - target["joints"]),
                    initial=0.0,
                )
            )
            if missing:
                rospy.logwarn_throttle(2.0, "waiting for measured joints: %s" % missing)
            if pos_error < pos_tol and speed < vel_tol and joint_error < joint_tol:
                rospy.loginfo("converged: CoG %.1f mm, %.3f m/s, joints %.3f rad", 1e3 * pos_error, speed, joint_error)
                return True
            if rospy.get_time() - start > timeout:
                rospy.logwarn(
                    "not converged in %.1f s (CoG %.1f mm, %.3f m/s, joints %.3f rad)",
                    timeout,
                    1e3 * pos_error,
                    speed,
                    joint_error,
                )
                return False
            rospy.loginfo_throttle(
                1.0, "CoG %.1f mm, %.3f m/s, joints %.3f rad" % (1e3 * pos_error, speed, joint_error)
            )
            timer.sleep()
        return False


# --- follow_trajectory.py and perching_task.py
KEYS = {
    "trajectory_file",
    "end_frame",
    "speed",
    "rate",
    "takeoff",
    "ground_joints",
    "joint_group",
    "approach_speed",
    "joint_speed",
    "settle_time",
}
DEFAULT_CONFIG = os.path.join(
    rospkg.RosPack().get_path("gimbalrotor"), "config", "quad_with_dual_arm", "follow_trajectory.yaml"
)


def setup():
    """The parameters, the targets of the trajectory file and a player on the robot of the namespace."""
    p = load_params(DEFAULT_CONFIG, KEYS)
    with open(resolve(p["trajectory_file"])) as f:
        data = json.load(f)
    targets = Targets(data, rospy.get_param("robot_description"))
    ground_joints(p, targets)  # checked here
    robot = GimbalrotorInterface(robot_ns=rospy.get_namespace().rstrip("/"), joint_group=p["joint_group"])
    return p, targets, TrajectoryPlayer(robot, targets, float(p["rate"]))


def ground_joints(p, targets):
    """The joints on the ground: those of the first frame of the file (ground_joints "first"; the plan
    starts with the arms in their ground posture, folded_joints of perching_motion.yaml), or the
    values given."""
    if p["ground_joints"] == "first":
        return [float(a) for a in targets.at(0.0)["joints"]]
    if len(p["ground_joints"]) != len(targets.servo_names):
        raise RuntimeError(
            'ground_joints is "first" or %d values, for %s' % (len(targets.servo_names), targets.servo_names)
        )
    return [float(a) for a in p["ground_joints"]]


def take_off(p, player):
    """On the ground (the motors off): the joints to ground_joints first; with takeoff, then the motors
    armed and the takeoff. Then wait for the hover state."""
    robot = player.robot
    if robot.getFlightState() == robot.ARM_OFF_STATE:
        joints = ground_joints(p, player.targets)
        rospy.loginfo("joints to %s on the ground", np.round(joints, 3).tolist())
        robot.setJointAngle(player.targets.servo_names, joints, thresh=0.05, timeout=10.0)
        if p["takeoff"]:
            robot.start()
            robot.takeoff()
    rospy.loginfo("waiting for the hover state")
    return robot.waitForState(robot.HOVER_STATE) and robot.getCogOdom() is not None
