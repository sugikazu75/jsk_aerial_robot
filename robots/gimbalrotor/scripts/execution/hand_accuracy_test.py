#!/usr/bin/env python3
"""Evaluation of the accuracy of the rear hand with and without the bar: from the same hooked posture
of the plan, the hand is sent to the same targets in the world by the hanging whole-body IK
(hanging_follower.py), once hanging on the bar and once with no bar, and the hand of the robot (its
estimated baselink pose and joint states), the targets, the thrust of the rotors and the joint torques
of the whole-body QP are logged.

  # 1. to the hooked posture of the plan: the same trajectory in both conditions
  #    with the bar:    the hook contact switched on at the hook (operator OK)
  rosrun gimbalrotor perching_task.py __ns:=gimbalrotor _after_hook:=stop
  #    without the bar: the same motion, no contact switched (operator OK at the hook)
  rosrun gimbalrotor perching_task.py __ns:=gimbalrotor _after_hook:=stop _switch_contacts:=false
  # 2. the targets
  rosrun gimbalrotor hand_accuracy_test.py __ns:=gimbalrotor _contact:=true    # with the bar
  rosrun gimbalrotor hand_accuracy_test.py __ns:=gimbalrotor _contact:=false   # without the bar

The IK starts from the frame of the plan at the hook (~start_frame of ~trajectory_file, the pose
perching_task.py stops at with after_hook "stop"), not from the measured robot: its hook point is where
the plan puts it on the bar (fixed in the world, with or without the bar), and the plane, the starting
posture and so the targets sent are the same in both conditions. It keeps the conditions of the
perching (the hook point, the angle of the hook, the CoM below the hook) in both; what differs is the
bar and the whole-body controller: ~contact switches its hook contact (~hook_contact) on (the bar
carries load in the allocation) or off (the rotors alone). Without the whole-body controller (bringup
whole_body:=false) the contact cannot be switched: allowed for ~contact:=false (a warning), an error
for true.

Before the targets the frame is held until the robot converges to it (~converge_*). The targets
(~targets) are in the world: [x, z] of the hand [m] (y and the plane those of the frame), or [x, z,
dangle] with dangle [rad] the direction of the hand from that of the frame (the IK tracks it with
track_direction of hanging_teleop.yaml). For each repetition and each target: MOVE (the target the IK
follows goes there at tip_speed; ~move_timeout if out of reach), SETTLE (~settle_time), MEASURE
(~measure_time); at the end back to the hand of the frame with ~return_to_start.

The end (~finish): "hold" the last target held (the contact as it is), or "unhook" (with
~return_to_start) as UNHOOK of perching_task.py: the frame of the plan held until the robot converges
to it (else held there with the contact on), the hook contact switched off and held through its
ramp_time and ~unhook_wait (the load back onto the rotors), then the plan played backwards from the
frame to ~approach_window before its first contact at ~approach_speed_scale (the hook out through its
opening), held there: away from the bar and land by teleoperation.

The IK bounds only the static torques of the rear arm joints under their own gravity; the torques of
the front arm carrying the bar are bounded only by the QP of the whole-body controller, which then
moves load from the contact to the rotors (or fails and falls back to the usual allocation). The
torques of its statics (<ns>/whole_body/joint_torque) and whether it succeeded are logged so that such
samples can be told apart.

Output in ~output_dir: <stamp>_<hover|perching>[_<label>].csv, one row per cycle (~rate of
hanging_teleop.yaml), and a .yaml of the conditions (the parameters, the frame, the plane, the hook).
Compare runs with scripts/analysis/compare_hand_accuracy.py. The IK parameters are those of
hanging_teleop.yaml (~ik_config to replace it), the experiment's in hand_accuracy_test.yaml (~config).
"""

import csv
import datetime
import json
import os
import threading

import numpy as np
import rospkg
import rospy
import yaml
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from spinal.msg import FourAxisCommand

from gimbalrotor.hanging_follower import FOLLOWER_KEYS, HangingFollower, frame_configuration, switch_contact
from gimbalrotor.robot_interface import GimbalrotorInterface
from gimbalrotor.trajectory_follower import Targets, TrajectoryPlayer, load_params, resolve

CONFIG_DIR = os.path.join(rospkg.RosPack().get_path("gimbalrotor"), "config", "quad_with_dual_arm")
KEYS = {
    "contact",
    "hook_contact",
    "contact_wait",
    "label",
    "trajectory_file",
    "start_frame",
    "converge_pos",
    "converge_vel",
    "converge_joint",
    "converge_timeout",
    "targets",
    "repetitions",
    "move_timeout",
    "settle_time",
    "measure_time",
    "return_to_start",
    "finish",
    "unhook_wait",
    "approach_window",
    "approach_speed_scale",
    "output_dir",
}


class Latest:
    """The latest message of a topic (None before the first)."""

    def __init__(self, topic, kind):
        self.lock = threading.Lock()
        self.msg = None
        rospy.Subscriber(topic, kind, self._callback, queue_size=1)

    def _callback(self, msg):
        with self.lock:
            self.msg = msg

    def get(self):
        with self.lock:
            return self.msg


def columns(n_rotor, joints):
    names = ["t", "rep", "index", "phase", "goal_u", "goal_z", "goal_angle", "goal_x_world", "goal_y_world", "goal_z_world"]
    names += ["ik_u", "ik_z", "ik_angle", "ik_hook_u", "ik_hook_z", "ik_pitch"]
    names += ["meas_u", "meas_z", "meas_angle", "meas_lateral", "meas_x", "meas_y", "meas_z_world"]
    names += ["meas_hook_u", "meas_hook_z", "meas_hook_lateral", "meas_com_u", "meas_com_z", "meas_pitch"]
    names += ["cog_cmd_x", "cog_cmd_y", "cog_cmd_z", "cog_x", "cog_y", "cog_z"]
    names += ["thrust_%d" % i for i in range(n_rotor)] + ["thrust_sum"]
    names += ["contact_fx", "contact_fy", "contact_fz"]
    # the joint torques of the statics of the whole-body QP and their limits (NaN without it)
    names += ["tau_%s" % j for j in joints] + ["tau_limit_%s" % j for j in joints] + ["qp_ok"]
    return names


def start_index(data, start_frame):
    """The frame of start_frame: "hook" the first contact of the file (where perching_task.py stops with
    after_hook "stop"), or a frame number."""
    if start_frame == "hook":
        index = next((k for k, f in enumerate(data["frames"]) if f["contacts"]), None)
        if index is None:
            raise RuntimeError("the trajectory has no contact")
        return index
    index = int(start_frame)
    if not 0 <= index < len(data["frames"]):
        raise RuntimeError("start_frame %d out of the %d frames" % (index, len(data["frames"])))
    return index


def main():
    rospy.init_node("hand_accuracy_test")
    p = load_params(os.path.join(CONFIG_DIR, "hand_accuracy_test.yaml"), KEYS)
    p.update(load_params(os.path.join(CONFIG_DIR, "hanging_teleop.yaml"), FOLLOWER_KEYS, "~ik_config", True))
    robot_ns = rospy.get_namespace().rstrip("/")
    robot = GimbalrotorInterface(robot_ns=robot_ns, joint_group=p["joint_group"])
    thrust = Latest(robot_ns + "/four_axes/command", FourAxisCommand)
    force = Latest("%s/whole_body/contacts/%s/force" % (robot_ns, p["hook_contact"]), WrenchStamped)
    torque = Latest(robot_ns + "/whole_body/joint_torque", JointState)

    # the starting frame of the plan
    with open(resolve(p["trajectory_file"])) as f:
        data = json.load(f)
    index0 = start_index(data, p["start_frame"])
    plan_targets = Targets(data, rospy.get_param("robot_description"))
    player = TrajectoryPlayer(robot, plan_targets, float(p["rate"]))
    t0 = plan_targets.time_of(index0)
    rospy.loginfo("the start: frame %d (%.2f s) of %s", index0, t0, p["trajectory_file"])

    contact = bool(p["contact"])
    condition = "perching" if contact else "hover"
    rospy.loginfo("waiting for the hover state")
    if not robot.waitForState(robot.HOVER_STATE):
        return
    if not switch_contact(robot_ns, p["hook_contact"], contact) and contact:
        rospy.logerr("the hook contact cannot be switched on: stopped")
        return
    rospy.loginfo("%s: waiting %.1f s for the load of the contact to move", condition, p["contact_wait"])
    if not player.hold(t0, float(p["contact_wait"])):
        return
    if not player.converge(t0, p["converge_pos"], p["converge_vel"], p["converge_joint"], p["converge_timeout"]):
        rospy.logerr("the robot is not at the frame %d: stopped (bring it there, e.g. with perching_task.py)", index0)
        return

    try:
        follower = HangingFollower(robot, p, start=lambda ik: frame_configuration(ik, data, index0))
    except rospy.ROSInterruptException:
        return
    ik = follower.ik
    start_tip, start_angle = follower.f.tip.copy(), follower.f.tip_angle
    rospy.loginfo(
        "the plan at the start: hook %s, hand %s (world), hand direction %.3f rad in the plane",
        ik.from_plane(ik.hook_target).round(3),
        ik.from_plane(start_tip).round(3),
        start_angle,
    )

    goals = []
    for target in p["targets"]:
        target = [float(v) for v in target]
        if len(target) not in (2, 3):
            rospy.logerr("~targets: [x, z] or [x, z, dangle] each, not %s", target)
            return
        point = ik.from_plane(np.zeros(2))  # y of the plane
        point[0], point[2] = target[0], target[1]
        goals.append((ik.to_plane(point), start_angle + (target[2] if len(target) == 3 else 0.0)))
    sequence = [(rep, i, g) for rep in range(int(p["repetitions"])) for i, g in enumerate(goals)]
    if p["return_to_start"]:
        sequence.append((-1, -1, (start_tip, start_angle)))

    while not rospy.is_shutdown() and thrust.get() is None:
        rospy.logwarn_throttle(2.0, "waiting for %s/four_axes/command", robot_ns)
        rospy.sleep(0.1)
    n_rotor = len(thrust.get().base_thrust)

    output_dir = os.path.expanduser(p["output_dir"])
    os.makedirs(output_dir, exist_ok=True)
    name = datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + condition
    if p["label"]:
        name += "_" + str(p["label"])
    path = os.path.join(output_dir, name)
    meta = {
        "condition": condition,
        "contact": contact,
        "label": p["label"],
        "trajectory_file": p["trajectory_file"],
        "start_frame": index0,
        "start_tip": start_tip.tolist(),
        "start_tip_world": ik.from_plane(start_tip).tolist(),
        "start_angle": float(start_angle),
        "hook": ik.hook_target.tolist(),
        "hook_world": ik.from_plane(ik.hook_target).tolist(),
        "plane_origin": ik.origin.tolist(),
        "plane_yaw": float(ik.yaw),
        "n_rotor": n_rotor,
        "params": p,
    }
    with open(path + ".yaml", "w") as f:
        yaml.safe_dump(meta, f, sort_keys=False)

    timer = rospy.Rate(float(p["rate"]))
    with open(path + ".csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(columns(n_rotor, ik.joint_names))
        rospy.loginfo("\033[32m%s: %d targets x %d, logging to %s.csv\033[0m", condition, len(goals), int(p["repetitions"]), path)
        for rep, index, (goal, goal_angle) in sequence:
            rospy.loginfo("rep %d target %d: %s (world)", rep, index, ik.from_plane(goal).round(3))
            phase, phase_start = "move", rospy.get_time()
            while not rospy.is_shutdown():
                now = rospy.get_time()
                if phase == "move" and (follower.reached(goal, goal_angle) or now - phase_start > p["move_timeout"]):
                    if not follower.reached(goal, goal_angle):
                        rospy.logwarn("target %d not reached in %.0f s: measured where the hand stops", index, p["move_timeout"])
                    phase, phase_start = "settle", now
                elif phase == "settle" and now - phase_start > p["settle_time"]:
                    phase, phase_start = "measure", now
                elif phase == "measure" and now - phase_start > p["measure_time"]:
                    break
                if not follower.step(goal, goal_angle):
                    return
                writer.writerow(row(now, rep, index, phase, goal, goal_angle, follower, robot, thrust, force, torque, n_rotor))
                timer.sleep()
            f.flush()
    if p["finish"] == "unhook":
        if not p["return_to_start"]:
            rospy.logwarn("finish unhook needs return_to_start: holding the last target, the contact as it is")
        else:
            unhook(p, robot, player, plan_targets, t0, contact)
            rospy.loginfo("\033[32mdone: %s.csv\033[0m", path)
            return
    elif p["finish"] != "hold":
        rospy.logwarn('~finish is "hold" or "unhook", not %r: holding', p["finish"])
    rospy.loginfo("\033[32mdone: %s.csv (the robot holds the last target)\033[0m", path)


def unhook(p, robot, player, plan_targets, t0, contact):
    """UNHOOK of perching_task.py from the frame at t0: converge to it, the hook contact off, the
    plan backwards to ~approach_window before its first contact."""
    robot_ns = robot.robot_ns
    if not player.converge(t0, p["converge_pos"], p["converge_vel"], p["converge_joint"], p["converge_timeout"]):
        rospy.logerr("not back at the frame of the plan: not unhooked, held there with the contact on")
        return False
    if contact:
        ramp = float(rospy.get_param("%s/controller/whole_body/contacts/%s/ramp_time" % (robot_ns, p["hook_contact"]), 2.0))
        switch_contact(robot_ns, p["hook_contact"], False)
        rospy.loginfo("contact %s off: holding %.1f s while the load moves back onto the rotors", p["hook_contact"],
                      ramp + float(p["unhook_wait"]))
        if not player.hold(t0, ramp + float(p["unhook_wait"])):
            return False
    end = max(plan_targets.time_of(plan_targets.first_contact) - float(p["approach_window"]), 0.0)
    if not player.play_back(t0, end, float(p["approach_speed_scale"])):
        return False
    player.send(plan_targets.still(end))
    rospy.loginfo("unhooked: away from the bar and land by teleoperation; the navigation holds the start of the approach")
    return True


def row(now, rep, index, phase, goal, goal_angle, follower, robot, thrust, force, torque, n_rotor):
    ik_f, x = follower.f, follower.x
    m = follower.measure()
    cog = robot.getCogPos()
    cog = [np.nan] * 3 if cog is None else list(cog)
    command = thrust.get()
    thrusts = list(command.base_thrust)[:n_rotor] if command is not None else [np.nan] * n_rotor
    wrench = force.get()
    contact = [np.nan] * 3 if wrench is None else [wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z]
    values = [now, rep, index, phase, goal[0], goal[1], goal_angle, *follower.ik.from_plane(goal)]
    values += [ik_f.tip[0], ik_f.tip[1], ik_f.tip_angle, ik_f.hook[0], ik_f.hook[1], x[2]]
    values += [m["tip"][0], m["tip"][1], m["tip_angle"], m["tip_lateral"], *m["tip_world"]]
    values += [m["hook"][0], m["hook"][1], m["hook_lateral"], m["com"][0], m["com"][1], m["pitch"]]
    values += list(follower.cog) + cog
    values += thrusts + [float(np.sum(thrusts))]
    values += contact
    # the QP publishes its torques only when it succeeds: a stale message means it failed (the usual
    # allocation, without the contact) or that there is no whole-body controller
    msg = torque.get()
    fresh = msg is not None and now - msg.header.stamp.to_sec() < 0.1
    tau = dict(zip(msg.name, zip(msg.effort, msg.position))) if fresh else {}
    joints = follower.ik.joint_names
    values += [tau[j][0] if j in tau else np.nan for j in joints]
    values += [tau[j][1] if j in tau else np.nan for j in joints]
    values += [int(fresh)]
    return values


if __name__ == "__main__":
    main()
