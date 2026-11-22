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

The target the IK follows moves to the requested one at ~tip_speed and ~tip_rotation_speed, staying
within ~tip_lag (~tip_angle_lag) of where the hand is: a target out of reach (or behind a joint limit,
the torque limit of the rear arm, a self-collision) stops the hand at the closest point it gets to,
and moving the target back moves the hand at once. Each cycle (~rate) the IK result goes to the robot:
the CoG and the body attitude to the navigation, the arm joints to the servos.

~markers (visualization_msgs/MarkerArray): the hook point (red), the hand of the IK (green arrow),
the CoM of the IK (blue). The node stops when the robot leaves the hover state. The parameters are
in config/quad_with_dual_arm/hanging_teleop.yaml.
"""

import dataclasses
import math
import os
import threading

import numpy as np
import pinocchio as pin
import rospkg
import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)

from gimbalrotor.hanging_ik import HangingIK, HangingIKConfig
from gimbalrotor.robot_interface import GimbalrotorInterface
from gimbalrotor.trajectory_follower import load_params

IK_KEYS = {f.name for f in dataclasses.fields(HangingIKConfig)} - {"front_joints", "rear_joints"}
KEYS = IK_KEYS | {
    "joint_group",
    "frame_id",
    "rate",
    "activate_hook",
    "hook_contact",
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
DEFAULT_CONFIG = os.path.join(
    rospkg.RosPack().get_path("gimbalrotor"), "config", "quad_with_dual_arm", "hanging_teleop.yaml"
)


def toward(current, goal, step):
    """current moved toward goal by at most step (norm)."""
    delta = np.asarray(goal, float) - current
    norm = np.linalg.norm(delta)
    return current + (delta if norm <= step else delta * step / norm)


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


def switch_hook(robot_ns, name):
    service = "%s/whole_body/contacts/%s/activate" % (robot_ns, name)
    try:
        rospy.wait_for_service(service, timeout=1.0)
        rospy.loginfo("contact %s", rospy.ServiceProxy(service, SetBool)(True).message)
    except (rospy.ROSException, rospy.ServiceException) as e:
        rospy.logwarn(
            "cannot switch the contact %s on (%s): is the whole-body controller running " "(bringup whole_body:=true)?",
            name,
            e,
        )


def main():
    rospy.init_node("hanging_teleop")
    p = load_params(DEFAULT_CONFIG, KEYS)
    robot_ns = rospy.get_namespace().rstrip("/")
    robot = GimbalrotorInterface(robot_ns=robot_ns, joint_group=p["joint_group"])
    rospy.loginfo("waiting for the hover state and the odometry")
    if not robot.waitForState(robot.HOVER_STATE):
        return
    while not rospy.is_shutdown() and robot.getBaseOdom() is None:
        rospy.sleep(0.1)
    joint_state = rospy.wait_for_message(robot_ns + "/joint_states", JointState, timeout=5.0)

    config = HangingIKConfig(**{k: (tuple(p[k]) if isinstance(p[k], list) else p[k]) for k in IK_KEYS})
    ik = HangingIK(rospy.get_param("robot_description"), config)
    q = ik.configuration(robot.getBasePos(), robot.getBaseRot(), dict(zip(joint_state.name, joint_state.position)))
    x = ik.start(q)
    if p["activate_hook"]:
        switch_hook(robot_ns, p["hook_contact"])
    f = ik.features(x)
    rospy.loginfo(
        "hanging: hook at %s (plane u, z), body pitch %.3f rad, hand at %s, collisions %s",
        f.hook.round(3),
        x[2],
        f.tip.round(3),
        sorted(ik.collisions(x)) or "none",
    )

    target = HandTarget(ik, p["frame_id"], f.tip, f.tip_angle)
    markers = Markers(ik, p["frame_id"])
    rate = float(p["rate"])
    dt = 1.0 / rate
    step = np.array([p["root_speed"]] * 2 + [p["root_rotation_speed"]] + [p["joint_speed"]] * 6, float) * dt
    tip, angle = f.tip.copy(), f.tip_angle  # what the IK follows
    cog_prev = None
    timer = rospy.Rate(rate)
    rospy.loginfo(
        "\033[32mmove the rear hand with the interactive marker %s or on %s\033[0m",
        target.server.topic_ns,
        rospy.resolve_name("~target"),
    )
    while not rospy.is_shutdown():
        goal, goal_angle = target.get()
        # within the lag of the hand, then toward the goal
        hand = f.tip
        if np.linalg.norm(tip - hand) > p["tip_lag"]:
            tip = toward(hand, tip, p["tip_lag"])
        tip = toward(tip, goal, p["tip_speed"] * dt)
        lag = math.remainder(angle - f.tip_angle, 2 * math.pi)
        angle = f.tip_angle + max(-p["tip_angle_lag"], min(p["tip_angle_lag"], lag))
        turn = math.remainder(goal_angle - angle, 2 * math.pi)
        angle += max(-p["tip_rotation_speed"] * dt, min(p["tip_rotation_speed"] * dt, turn))

        result = ik.solve(x, tip, angle if p["track_direction"] else None, step, int(p["ik_iterations"]))
        x, f = result.p, result.features
        cog, rot, joints = ik.targets(x)
        vel = np.zeros(3) if cog_prev is None else (cog - cog_prev) / dt
        cog_prev = cog
        if not robot.sendCogTarget(cog, vel, rot):
            rospy.logerr("not hovering any more (state %s): stopped", robot.getFlightState())
            break
        robot.sendJointTarget(ik.joint_names, joints)
        markers.publish(result)

        if result.hook_soft:
            rospy.logwarn_throttle(1.0, "the hook cannot be kept exactly (%.1f mm off)" % (1e3 * result.hook_error))
        if result.blocked:
            rospy.logwarn_throttle(1.0, "stopped by a self-collision of %s" % result.blocked)
        if np.linalg.norm(tip - f.tip) > 0.9 * p["tip_lag"]:  # the hand does not follow
            rospy.logwarn_throttle(
                2.0,
                "the hand stops %.1f cm from the target (out of reach, or at a limit): body "
                "pitch %.2f rad, rear arm torques %s Nm"
                % (1e2 * np.linalg.norm(goal - f.tip), x[2], f.torque.round(2)),
            )
        timer.sleep()


if __name__ == "__main__":
    main()
