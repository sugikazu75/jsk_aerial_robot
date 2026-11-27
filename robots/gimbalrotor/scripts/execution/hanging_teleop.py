#!/usr/bin/env python3
"""Move the rear hand freely while hanging from the bar by the hook of the front arm: a whole-body IK
(src/gimbalrotor/hanging_ik.py) in the plane of the arms keeps the hook on the bar and the CoM below
it, and moves the root, the body pitch and both arms for the hand to reach its target.

  rosrun gimbalrotor hanging_teleop.py __ns:=gimbalrotor

Start it while the robot hangs on the bar (after the hooking of perching_task.py, for instance): the
hook point, the plane of the arms and the starting posture are taken from the robot then (the
baselink odometry and the joint states). With ~activate_hook the hook contact of the whole-body
controller is switched on (nothing if it already is; a warning without the controller).

The target of the hand, in ~frame_id (world):

  * the interactive marker ~target_marker in RViz (an InteractiveMarkers display on
    /<ns>/hanging_teleop/target_marker/update): drag the arrow in the plane of the arms, turn the
    ring for the direction of the hand
  * geometry_msgs/PoseStamped on ~target: the position, and the direction as the x axis of the
    orientation; both projected onto the plane of the arms, e.g.
      rostopic pub -1 /gimbalrotor/hanging_teleop/target geometry_msgs/PoseStamped \\
        '{header: {frame_id: world}, pose: {position: {x: -0.3, y: 0.0, z: 1.0}, orientation: {y: 1.0, w: 0.0}}}'

  * the joystick (sensor_msgs/Joy on <ns>/joy, raw indices of ~joy_*): while L1 (~joy_enable_button)
    is held, the left stick up/down moves the target along the heading, the right stick up/down in
    height, the right stick left/right turns the direction of the hand (up). When the sticks go back
    to neutral or L1 is released, the target is set to where the hand is then (it stays there). The
    navigation of gimbalrotor ignores the sticks for the flight while L1 is held.

The target the IK follows moves to the requested one at ~tip_speed and ~tip_rotation_speed, staying
within ~tip_lag (~tip_angle_lag) of where the hand is: a target out of reach (or behind a joint limit,
the torque limit of the rear arm, a self-collision) stops the hand at the closest point it gets to,
and moving the target back moves the hand at once. Each cycle (~rate) the IK result goes to the robot:
the CoG and the body attitude to the navigation, the arm joints to the servos.

Leaving the bar: hold L1 and R1 (~joy_unhook_button) together for ~joy_unhook_hold [s], or send
std_msgs/Empty on ~unhook:

  rostopic pub -1 /gimbalrotor/hanging_teleop/unhook std_msgs/Empty

The hand goes back to where it was at the start (the hooked posture) by the IK; the hook contact is
switched off and the posture held through its ramp_time and ~unhook_wait (the load back onto the
rotors); then the whole robot (the posture held) moves back along the opening of the hook by
~unhook_distance at ~unhook_speed, the reverse of the approach of plan_perching_motion.py: the hook
comes out of the bar. The node ends there, the navigation holding the last target: land by
teleoperation. The joystick and the targets are ignored once the unhook has begun.

~markers (visualization_msgs/MarkerArray): the hook point (red), the hand of the IK (green arrow),
the CoM of the IK (blue). The node stops when the robot leaves the hover state. The parameters are
in config/quad_with_dual_arm/hanging_teleop.yaml.
"""

import math
import os
import threading

import numpy as np
import pinocchio as pin
import rospkg
import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from sensor_msgs.msg import Joy
from std_msgs.msg import ColorRGBA, Empty
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)

from gimbalrotor.hanging_follower import FOLLOWER_KEYS, HangingFollower, switch_contact, toward
from gimbalrotor.perching_model import HOOK_OPENING
from gimbalrotor.robot_interface import GimbalrotorInterface
from gimbalrotor.trajectory_follower import load_params

KEYS = FOLLOWER_KEYS | {
    "frame_id",
    "activate_hook",
    "hook_contact",
    "joy_enable_button",
    "joy_axis_forward",
    "joy_axis_up",
    "joy_axis_turn",
    "joy_deadzone",
    "joy_unhook_button",
    "joy_unhook_hold",
    "unhook_wait",
    "unhook_distance",
    "unhook_speed",
}
DEFAULT_CONFIG = os.path.join(
    rospkg.RosPack().get_path("gimbalrotor"), "config", "quad_with_dual_arm", "hanging_teleop.yaml"
)


class HandTarget:
    """The requested target of the hand, (u, z) and angle in the plane of the IK, from the interactive
    marker and the topic ~target."""

    def __init__(self, ik, frame_id, tip, angle):
        self.ik, self.frame_id = ik, frame_id
        self.lock = threading.Lock()
        self.tip, self.angle = np.asarray(tip, float).copy(), float(angle)
        self.server = InteractiveMarkerServer(rospy.get_name() + "/target_marker")
        self.server.insert(self._marker(), self._feedback)
        self.server.applyChanges()
        rospy.Subscriber("~target", PoseStamped, self._target)

    def _orientation(self, angle):
        # x along the direction of the hand, y normal to the plane
        return Quaternion(*pin.Quaternion(pin.rpy.rpyToMatrix(0.0, -angle, self.ik.yaw)).coeffs())

    def _pose(self, tip, angle):
        pose = PoseStamped().pose
        pose.position = Point(*self.ik.from_plane(tip))
        pose.orientation = self._orientation(angle)
        return pose

    def _marker(self):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = "hand"
        marker.description = "rear hand"
        marker.scale = 0.2
        marker.pose = self._pose(self.tip, self.angle)
        # both controls about the normal of the plane (the x axis of their orientation), fixed in the world
        normal = Quaternion(*pin.Quaternion(pin.rpy.rpyToMatrix(0.0, 0.0, self.ik.yaw + math.pi / 2)).coeffs())
        arrow = Marker(type=Marker.ARROW, scale=Vector3(0.12, 0.015, 0.015), color=ColorRGBA(1.0, 0.6, 0.0, 0.9))
        arrow.pose.orientation.w = 1.0
        move = InteractiveMarkerControl(
            name="move",
            orientation=normal,
            orientation_mode=InteractiveMarkerControl.FIXED,
            interaction_mode=InteractiveMarkerControl.MOVE_PLANE,
            always_visible=True,
        )
        move.markers.append(arrow)
        rotate = InteractiveMarkerControl(
            name="rotate",
            orientation=normal,
            orientation_mode=InteractiveMarkerControl.FIXED,
            interaction_mode=InteractiveMarkerControl.ROTATE_AXIS,
        )
        marker.controls += [move, rotate]
        return marker

    def _set(self, position, orientation):
        tip = self.ik.to_plane(np.array([position.x, position.y, position.z]))
        rotation = pin.Quaternion(orientation.w, orientation.x, orientation.y, orientation.z)
        if rotation.norm() < 1e-6:
            angle = self.angle
        else:
            rotation.normalize()
            angle = self.ik.plane_angle(rotation.toRotationMatrix()[:, 0])
        with self.lock:
            self.tip, self.angle = tip, angle
        return tip, angle

    def _feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        if feedback.header.frame_id not in ("", self.frame_id):
            rospy.logwarn_throttle(1.0, "the marker in %s, not %s: ignored", feedback.header.frame_id, self.frame_id)
            return
        self._set(feedback.pose.position, feedback.pose.orientation)

    def _target(self, msg):
        if msg.header.frame_id not in ("", self.frame_id):
            rospy.logwarn("~target in %s, not %s: ignored", msg.header.frame_id, self.frame_id)
            return
        tip, angle = self._set(msg.pose.position, msg.pose.orientation)
        self.server.setPose("hand", self._pose(tip, angle))  # the marker shows it, in the plane
        self.server.applyChanges()

    def get(self):
        with self.lock:
            return self.tip.copy(), self.angle

    def set(self, tip, angle, show=True):
        """Set the requested target (from the joystick); show: move the marker to it too."""
        with self.lock:
            self.tip, self.angle = np.asarray(tip, float).copy(), float(angle)
        if show:
            self.server.setPose("hand", self._pose(tip, angle))
            self.server.applyChanges()


class JoyInput:
    """The sticks while the enable button (L1) is held: the requested velocity of the hand, (u, z) [-1, 1]
    and the turn [-1, 1], or None without input (released, or the sticks in the deadzone)."""

    def __init__(self, robot_ns, p):
        self.button = int(p["joy_enable_button"])
        self.axes = [int(p["joy_axis_forward"]), int(p["joy_axis_up"]), int(p["joy_axis_turn"])]
        self.deadzone = float(p["joy_deadzone"])
        self.unhook_button, self.unhook_hold = int(p["joy_unhook_button"]), float(p["joy_unhook_hold"])
        self.lock = threading.Lock()
        self.command = None
        self.unhook_since = None  # when the enable and the unhook buttons began to be held together
        rospy.Subscriber(robot_ns + "/joy", Joy, self._joy, queue_size=1)

    def _joy(self, msg):
        command = None
        pressed = lambda b: b < len(msg.buttons) and msg.buttons[b] == 1  # noqa: E731
        with self.lock:
            if pressed(self.button) and pressed(self.unhook_button):
                self.unhook_since = self.unhook_since or rospy.get_time()
            else:
                self.unhook_since = None
        if pressed(self.button) and max(self.axes) < len(msg.axes):
            values = np.array([msg.axes[i] for i in self.axes], float)
            values[np.abs(values) < self.deadzone] = 0.0
            if np.any(values != 0.0):
                command = values
        with self.lock:
            self.command = command

    def get(self):
        """The last command: joy_node publishes on changes only (no autorepeat), a stick held still
        keeps it."""
        with self.lock:
            return None if self.command is None else self.command.copy()

    def unhook(self):
        """The enable and the unhook buttons held together for unhook_hold [s]."""
        with self.lock:
            return self.unhook_since is not None and rospy.get_time() - self.unhook_since >= self.unhook_hold


class Markers:
    def __init__(self, ik, frame_id):
        self.ik, self.frame_id = ik, frame_id
        self.pub = rospy.Publisher("~markers", MarkerArray, queue_size=1)

    def _marker(self, ident, kind, color, scale):
        marker = Marker(type=kind, id=ident, action=Marker.ADD, scale=Vector3(*scale), color=ColorRGBA(*color))
        marker.header.frame_id, marker.header.stamp, marker.ns = self.frame_id, rospy.Time.now(), "hanging_teleop"
        marker.pose.orientation.w = 1.0
        return marker

    def publish(self, result):
        f, ik = result.features, self.ik
        hook = self._marker(0, Marker.SPHERE, (1.0, 0.1, 0.1, 0.8), (0.03, 0.03, 0.03))
        hook.pose.position = Point(*ik.from_plane(ik.hook_target))
        com = self._marker(1, Marker.SPHERE, (0.1, 0.3, 1.0, 0.8), (0.04, 0.04, 0.04))
        com.pose.position = Point(*ik.from_plane(f.com))
        hand = self._marker(2, Marker.ARROW, (0.1, 0.9, 0.1, 0.9), (0.012, 0.025, 0.03))
        start = ik.from_plane(f.tip)
        hand.points = [Point(*start), Point(*(start + 0.1 * ik.direction(f.tip_angle)))]
        self.pub.publish(MarkerArray(markers=[hook, com, hand]))


def unhook(follower, robot_ns, p, start_tip, start_angle, timer):
    """Leave the bar: the hand back to the start (the hooked posture), the hook contact off and held
    through its ramp, then the robot moved back along the opening of the hook. False if the robot stops
    hovering."""
    robot, ik = follower.robot, follower.ik
    rospy.loginfo("\033[32munhook: the hand back to the start %s\033[0m", ik.from_plane(start_tip).round(3))
    settled, deadline = None, rospy.get_time() + 30.0
    while not rospy.is_shutdown():
        if rospy.get_time() > deadline:
            rospy.logerr("unhook: the hand does not get back to the start in 30 s: not unhooked (contact on)")
            return False
        if not follower.step(start_tip, start_angle):
            return False
        if follower.reached(start_tip, start_angle) and np.linalg.norm(follower.f.tip - start_tip) < 2e-3:
            settled = settled or rospy.get_time()
            if rospy.get_time() - settled > 1.0:
                break
        timer.sleep()
    ramp = float(rospy.get_param("%s/controller/whole_body/contacts/%s/ramp_time" % (robot_ns, p["hook_contact"]), 2.0))
    switch_contact(robot_ns, p["hook_contact"], False)
    wait = ramp + float(p["unhook_wait"])
    rospy.loginfo("unhook: contact %s off, holding %.1f s while the load moves back onto the rotors", p["hook_contact"], wait)
    end = rospy.get_time() + wait
    while not rospy.is_shutdown() and rospy.get_time() < end:
        if not follower.step(start_tip, start_angle):
            return False
        timer.sleep()
    # the reverse of the approach: the whole robot back along the opening of the hook, the posture held
    pin.framesForwardKinematics(ik.model, ik.data, ik.q(follower.x))
    opening = ik.data.oMf[ik.hook_frame].rotation @ HOOK_OPENING
    cog0, rot, joints = ik.targets(follower.x)
    goal = cog0 - float(p["unhook_distance"]) * opening
    speed, dt = float(p["unhook_speed"]), follower.dt
    rospy.loginfo("unhook: the hook out of the bar, %.2f m along %s", float(p["unhook_distance"]), (-opening).round(3))
    cog = cog0.copy()
    while not rospy.is_shutdown():
        nxt = toward(cog, goal, speed * dt)
        vel = (nxt - cog) / dt
        cog = nxt
        if not robot.sendCogTarget(cog, vel, rot):
            rospy.logerr("not hovering any more (state %s): stopped", robot.getFlightState())
            return False
        robot.sendJointTarget(ik.joint_names, joints)
        if np.linalg.norm(goal - cog) < 1e-9:
            break
        timer.sleep()
    robot.sendCogTarget(goal, np.zeros(3), rot)
    rospy.loginfo("\033[32munhooked: land by teleoperation; the navigation holds the last target\033[0m")
    return True


def main():
    rospy.init_node("hanging_teleop")
    p = load_params(DEFAULT_CONFIG, KEYS)
    robot_ns = rospy.get_namespace().rstrip("/")
    robot = GimbalrotorInterface(robot_ns=robot_ns, joint_group=p["joint_group"])
    try:
        follower = HangingFollower(robot, p)
    except rospy.ROSInterruptException:
        return
    ik, f = follower.ik, follower.f
    if p["activate_hook"]:
        switch_contact(robot_ns, p["hook_contact"], True)
    rospy.loginfo(
        "hanging: hook at %s (plane u, z), body pitch %.3f rad, hand at %s, collisions %s",
        f.hook.round(3),
        follower.x[2],
        f.tip.round(3),
        sorted(ik.collisions(follower.x)) or "none",
    )

    start_tip, start_angle = f.tip.copy(), f.tip_angle  # the hooked posture: where the unhook begins
    target = HandTarget(ik, p["frame_id"], f.tip, f.tip_angle)
    joy = JoyInput(robot_ns, p)
    unhook_request = threading.Event()
    rospy.Subscriber("~unhook", Empty, lambda _: unhook_request.set())
    joy_active = False
    marker_period, marker_stamp = 0.1, 0.0  # [s] the marker follows the joystick at most so often
    markers = Markers(ik, p["frame_id"])
    dt = follower.dt
    timer = rospy.Rate(float(p["rate"]))
    rospy.loginfo(
        "\033[32mmove the rear hand with the interactive marker %s, on %s, or with the sticks while "
        "holding the joystick button %d (L1)\033[0m",
        target.server.topic_ns,
        rospy.resolve_name("~target"),
        joy.button,
    )
    rospy.loginfo(
        "\033[32mleave the bar: hold L1 and the button %d (R1) for %.1f s, or send std_msgs/Empty on %s\033[0m",
        joy.unhook_button,
        joy.unhook_hold,
        rospy.resolve_name("~unhook"),
    )
    while not rospy.is_shutdown():
        if joy.unhook() or unhook_request.is_set():
            unhook(follower, robot_ns, p, start_tip, start_angle, timer)
            return
        command = joy.get()
        if command is not None:
            # the requested target moves from the one the IK follows (no run-away out of reach)
            now = rospy.get_time()
            show = now - marker_stamp > marker_period
            if show:
                marker_stamp = now
            target.set(
                follower.tip + command[:2] * p["tip_speed"] * dt,
                follower.angle + command[2] * p["tip_rotation_speed"] * dt,
                show,
            )
            joy_active = True
        elif joy_active:
            # the input stopped: the target is where the hand is now
            target.set(follower.f.tip, follower.f.tip_angle)
            follower.hold_hand()
            joy_active = False
        goal, goal_angle = target.get()
        if not follower.step(goal, goal_angle):
            break
        markers.publish(follower.result)
        timer.sleep()


if __name__ == "__main__":
    main()
