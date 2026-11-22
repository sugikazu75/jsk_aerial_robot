#!/usr/bin/env python3
"""Plan a hanging-perching motion of gimbalrotor quad_with_dual_arm and solve its statics along
the way, to show how much thrust hanging saves. Offline: no ROS master is needed. The parameters are
all in config/quad_with_dual_arm/perching_motion.yaml (--config another; --key value overrides one).

The scene (urdf/environment/perching_environment.urdf.xacro) has a horizontal bar along y and a
wall facing +x behind it. The robot flies to the bar, hangs the hook module of the front arm (a C
whose opening faces down at the bar) on it, then reaches the wall with the rear arm and pushes on it.
The phases are

  hover     hovering at --start with both arms folded down           (no contact)
  flight    planned with OMPL RRTConnect to just off the hook pose     (no contact)
  pre       holding there (the corner of the path: kept by the         (no contact)
            optimisation, and where a controller can converge)
  approach  by --approach along the opening of the hook onto the bar   (no contact)
  load      the front arm on the bar, the load moved from the rotors   (hook contact)
            onto the bar over --load-time: the normal force of the hook
            is capped by a share of its full value that grows to one
  hang      the front arm on the bar                                  (hook contact)
  reach     the rear arm swung out to --standoff from the wall         (hook contact)
  touch     the rear hand moved onto the wall                          (hook contact)
  push      the push on the wall ramped up to --push-force             (hook contact + force)
  hold      holding the push                                           (hook contact + force)

Flight: the planning state is the root pose (the attitude as its offset from --reference-attitude,
a rotation vector, each component within --max-body-tilt, yaw included) and the six arm joints. The
gimbals are turned, in every checked state, so that each thrust axis is as close to straight up as
it gets. A state is valid when nothing of the robot is within --margin of the scene, no two
non-adjacent links collide, and the rotors can hold the robot (the horizontal CoM inside the rotor
hull, the lift capacity above --thrust-margin times the weight).

Contact postures: an IK (scipy least squares) on the root pose (the attitude within
--max-body-tilt of --reference-attitude) and the arm joints puts the hook point (the inside of the
hook) on top of the bar with the last link of the front arm tilted by --hook-tilt, the CoM right below the hook so that the bar carries
the weight, and the rear hand on its target. The attitude is redundant to the arms; the IK keeps it
and the joints close to the previous posture and pulls the attitude back to the reference by
--attitude-weight, so it tilts the body only where the tasks need it (tilting is a risk).

Statics (gimbalrotor.rotor_statics): each gimbal turns its rotor about the rotor arm, so a rotor can push
anywhere in the plane normal to that axis, and the thrust vectors enter the force and moment
balance linearly (the rotor drag torque included). One QP per frame finds the thrust vectors of
least squared norm, the hook force inside its friction cone and the push given, for the robot to
stand still. Every rotor keeps pushing up, so that the attitude stays under control while the bar
carries the robot: in the gimbal plane, its component along the most upward direction is at least
--min-lift times the weight over the number of rotors (and the thrust limit of the URDF), and the
thrust is tilted from that direction by at most --max-thrust-tilt. The joint torques, linear in
these forces, stay within --max-effort-ratio of the effort limits of the URDF, so the rotors take
over what the arm cannot hold; the gimbal angles follow from the thrust directions, and the joint torques from the
RNEA with these forces. From the hang on, every frame is solved once more without the hook
contact (the same posture and push in free flight); both are summed up per phase at the end and
drawn in a plot next to the trajectory file.

The trajectory file holds, every dt, the root pose, the joint angles, the thrust, the static joint
torques and the force of each active contact; the phases are not written, the contact states tell them apart. It is played
in RViz by trajectory_playback.py (launch/perching_motion.launch) and is the initial guess of the
trajectory optimisation of aerial_to_solver.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np
import pinocchio as pin
import rospkg
import yaml

from aerial_motion_planning.trajectory import waypoint_times as timed, resample_linear as resample, line
from aerial_motion_planning.search import rrt_connect

from gimbalrotor.perching_model import (
    ARM_JOINTS,
    ROOM,
    FRONT_TIP,
    REAR_TIP,
    FRONT_LINK3,
    REAR_LINK3,
    FRONT_HOOK,
    HOOK_OFFSET,
    HOOK_OPENING,
    PERCH_PHASES,
    HOOK,
    PUSH,
    BodyAttitude,
    HookPitch,
    Robot,
    Statics,
    StaticsLimits,
)


def xacro(path, **args):
    command = ["xacro", path] + ["%s:=%s" % item for item in args.items()]
    return subprocess.run(command, check=True, capture_output=True, text=True).stdout


def plan_flight(robot, x_start, x_goal, args):
    """Task-specific state chart and flight validity on a generic RRTConnect search."""
    attitude = robot.attitude
    dims = (
        [0, 1, 2]
        + [3 + i for i, enabled in enumerate(attitude.mask) if enabled and attitude.max_tilt > 0.0]
        + list(range(6, 12))
    )
    low = np.concatenate([ROOM[:, 0], [-attitude.max_tilt] * 3, robot.joint_low])
    high = np.concatenate([ROOM[:, 1], [attitude.max_tilt] * 3, robot.joint_high])

    def to_x(state):
        x = np.zeros(12)
        x[dims] = state
        x[3:6] = attitude.rotation_vector(x[3:6])
        return x

    def to_state(x):
        x = np.asarray(x, float).copy()
        x[3:6] = attitude.offset(x[3:6])
        return x[dims]

    def valid(state: np.ndarray) -> bool:
        """Check flight margins and collisions for a planner state."""
        q = robot.q(to_x(state))
        depth, lift = robot.flight_margin(q)
        return depth >= args.hull_margin and lift >= args.thrust_margin and not robot.in_collision(q)

    # the ends may lie beyond the margins kept inside the joint limits (e.g. the folded arms at a limit)
    ends = np.array([to_state(x_start), to_state(x_goal)])
    low, high = np.minimum(low[dims], ends.min(axis=0)), np.maximum(high[dims], ends.max(axis=0))
    states, exact, checks, elapsed = rrt_connect(
        to_state(x_start), to_state(x_goal), low, high, valid, args.resolution, args.range, args.time
    )
    print(
        "flight: %s after %.1f s and %d state checks"
        % ("Exact solution" if exact else "No exact solution", elapsed, checks)
    )
    if not exact:
        raise SystemExit("no flight path found; try a longer --time")
    return [to_x(state) for state in states]


def check(
    robot: Robot, name: str, x: np.ndarray, args: argparse.Namespace, margin: float, bar_margin: float | None = None
) -> None:
    """Report and reject a named flight pose that violates the planning limits."""
    q = robot.q(x)
    robot.set_margin(margin, bar_margin)
    hits = robot.collisions(q)
    depth, lift = robot.flight_margin(q)
    print("%s: flight margin %.3f m, lift %.2f weights, collisions %s" % (name, depth, lift, hits))
    if hits or depth < args.hull_margin or lift < args.thrust_margin:
        raise SystemExit("the state %s is not valid; change the scene or the options" % name)


def task_to_world(task_frame, root):
    """A root pose [x, y, z, qx, qy, qz, qw] of the task frame in the world, the task frame being
    [x, y, yaw] in it."""
    x, y, yaw = (float(v) for v in task_frame)
    placement = pin.SE3(pin.rpy.rpyToMatrix(0.0, 0.0, yaw), np.array([x, y, 0.0])) * pin.XYZQUATToSE3(
        np.asarray(root, float)
    )
    return [float(v) for v in pin.SE3ToXYZQUAT(placement)]


def contact_placement(robot, x, link, offset, normal):
    """The SE3 in link of a frame at offset whose z is the world normal at the posture x, with its
    x as close to the link's x as it gets."""
    pin.forwardKinematics(robot.model, robot.data, robot.raw_q(x))
    pin.updateFramePlacements(robot.model, robot.data)
    link_rotation = robot.data.oMf[robot.model.getFrameId(link)].rotation
    z = normal / np.linalg.norm(normal)
    axis = link_rotation[:, 0] - z * (z @ link_rotation[:, 0])
    axis /= np.linalg.norm(axis)
    return pin.SE3(link_rotation.T @ np.column_stack([axis, np.cross(z, axis), z]), offset)


def plot(path, records, phases, weight, rate):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.arange(len(records)) / rate
    perch = np.array([r["thrust"] for r in records])
    free = np.array([r["free"] for r in records])
    fig, ax = plt.subplots(figsize=(9, 3.6))
    start, count = 0.0, 0
    for k, (name, n) in enumerate(phases):
        count += n
        end = t[min(count, len(t)) - 1]
        if k % 2:
            ax.axvspan(start, end, color="0.93", lw=0)
        ax.text((start + end) / 2, 1.01, name, transform=ax.get_xaxis_transform(), ha="center", va="bottom", fontsize=8)
        start = end
    ax.axhline(weight, color="0.5", ls=":", lw=1, label="weight")
    ax.plot(t, free, color="#d1495b", lw=1.5, ls="--", label="same posture without perching")
    ax.plot(t, perch, color="#00798c", lw=2, label="planned (hanging from the bar)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("total thrust [N]")
    ax.set_xlim(t[0], t[-1])
    ax.set_ylim(0, max(np.nanmax(free), perch.max()) * 1.15)
    ax.legend(loc="lower left", fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print("wrote %s" % path)


def resolve(path):
    """A path that may start with package://<package>/."""
    if path.startswith("package://"):
        package, _, rest = path[len("package://") :].partition("/")
        return os.path.join(rospkg.RosPack().get_path(package), rest)
    return path


def parse_args():
    """The parameters of the plan: all of them in the YAML file of --config, each one overridable on the
    command line (--name value, the key with - for _)."""
    package = rospkg.RosPack().get_path("gimbalrotor")
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--config",
        default=os.path.join(package, "config", "quad_with_dual_arm", "perching_motion.yaml"),
        help="the parameters (YAML)",
    )
    parser.add_argument("--no-plot", action="store_true")
    # files (package:// or paths)
    parser.add_argument("--robot", help="the robot (xacro)")
    parser.add_argument("--scene", help="the scene (xacro)")
    parser.add_argument("--output", help="the trajectory file written")
    # the scene
    parser.add_argument("--bar-x", type=float, help="[m] the bar (pass the same to the launch)")
    parser.add_argument("--bar-z", type=float, help="[m]")
    parser.add_argument("--bar-radius", type=float, help="[m]")
    parser.add_argument("--wall-x", type=float, help="[m] the face of the wall")
    # the task
    parser.add_argument(
        "--task-frame",
        type=float,
        nargs=3,
        metavar=("X", "Y", "YAW"),
        help="[m, m, rad] the task frame (of the scene, the start and the plan) in the world; "
        "the trajectory file is written in the world",
    )
    parser.add_argument("--start", type=float, nargs=3, help="[m] hover position (task frame)")
    parser.add_argument(
        "--folded-joints",
        type=float,
        nargs=3,
        help="[rad] joint1..3 of both arms at the start (on the ground and in the hover)",
    )
    parser.add_argument(
        "--hang-rear-joints",
        type=float,
        nargs=3,
        help="[rad] joint1..3 of the rear arm while hanging, from where it reaches the wall",
    )
    parser.add_argument("--hook-tilt", type=float, help="[rad] the last link of the front arm, tip down")
    parser.add_argument(
        "--hook-pitch-range",
        type=float,
        help="[rad] the hook turns on the bar (a hinge about the bar axis, its roll and yaw held) "
        "by up to this from --hook-tilt; 0 fixes it at --hook-tilt",
    )
    parser.add_argument(
        "--hook-pitch-weight",
        type=float,
        help="per rad, of the IK residual pulling the pitch of the hook to --hook-tilt",
    )
    parser.add_argument("--friction", type=float, help="of the hook on the bar")
    parser.add_argument("--push-force", type=float, help="[N] pushed on the wall")
    parser.add_argument("--standoff", type=float, help="[m] of the rear hand from the wall after reach")
    parser.add_argument(
        "--approach",
        type=float,
        help="[m] along the opening of the hook onto the bar (the hook clear of the bar by --margin)",
    )
    # the statics
    parser.add_argument(
        "--min-lift",
        type=float,
        help="share of the weight the rotors carry at least, even when the bar could carry "
        "it all (each rotor the same share, along the most upward direction of its "
        "gimbal plane), to keep the attitude under control",
    )
    parser.add_argument(
        "--max-effort-ratio",
        type=float,
        help="share of the effort limit of the URDF each joint (arms and gimbals) may take; "
        "written for the optimisation as its effort limits",
    )
    parser.add_argument(
        "--max-thrust-tilt",
        type=float,
        help="[rad] of the thrust from the most upward direction of its gimbal plane; written "
        "for the optimisation as limits of the gimbal joints",
    )
    # the body attitude
    parser.add_argument(
        "--reference-attitude",
        type=float,
        nargs=3,
        metavar=("ROLL", "PITCH", "YAW"),
        help="[rad] the reference attitude of the body, around which the IK and the flight "
        "planning move it (which attitude suits the tasks depends on the robot)",
    )
    parser.add_argument(
        "--max-body-tilt",
        type=float,
        help="[rad] the largest offset of the body from the reference attitude (each component "
        "of its rotation vector, yaw included) in the IK and the flight planning; 0 keeps "
        "it at the reference",
    )
    parser.add_argument(
        "--attitude-mask",
        type=int,
        nargs=3,
        metavar=("ROLL", "PITCH", "YAW"),
        help="0 fixes an attitude offset at its reference; 1 allows it within --max-body-tilt",
    )
    parser.add_argument(
        "--attitude-weight",
        type=float,
        help="weight of the IK residual pulling the body back to the reference attitude (per "
        "rad; the joints and the attitude are kept close to the previous posture at 0.3): "
        "tilting is a risk, so the IK tilts the body only where the tasks need it",
    )
    # the flight planning
    parser.add_argument("--margin", type=float, help="[m] clearance from the scene in flight")
    parser.add_argument(
        "--bar-margin",
        type=float,
        help="[m] clearance of the front arm that passes over the bar (link2, link3, the hook) from "
        "the bar in flight, instead of --margin",
    )
    parser.add_argument("--hull-margin", type=float, help="[m] CoM inside the rotor hull")
    parser.add_argument("--thrust-margin", type=float, help="lift capacity / weight")
    parser.add_argument("--time", type=float, help="[s] planning time limit")
    parser.add_argument("--range", type=float, help="RRTConnect step")
    parser.add_argument("--resolution", type=float, help="edge check resolution (fraction)")
    parser.add_argument("--seed", type=int)
    # the timing (frames = duration x rate)
    parser.add_argument("--rate", type=float, help="[Hz] of the output frames")
    parser.add_argument("--hover-time", type=float, help="[s]")
    parser.add_argument("--flight-speed", type=float, help="[m/s] the root in flight at most")
    parser.add_argument("--flight-rotation-speed", type=float, help="[rad/s] the root attitude in flight")
    parser.add_argument("--flight-joint-speed", type=float, help="[rad/s] the joints in flight at most")
    parser.add_argument("--pre-time", type=float, help="[s] held off the bar before the approach")
    parser.add_argument("--approach-time", type=float, help="[s]")
    parser.add_argument(
        "--load-time", type=float, help="[s] over which the load moves from the rotors onto the bar after the touchdown"
    )
    parser.add_argument("--hang-time", type=float, help="[s]")
    parser.add_argument("--reach-time", type=float, help="[s]")
    parser.add_argument("--reach-ik-steps", type=int, help="IK points along the reach (the path, not the time)")
    parser.add_argument("--touch-time", type=float, help="[s]")
    parser.add_argument("--touch-ik-steps", type=int, help="IK points along the touch")
    parser.add_argument("--push-time", type=float, help="[s] the push ramped up")
    parser.add_argument("--hold-time", type=float, help="[s]")
    parser.add_argument("--release-time", type=float, help="[s] the load back onto the rotors (the push let go)")
    parser.add_argument("--unhook-time", type=float, help="[s] the hook out of the bar")
    parser.add_argument("--unhook-ik-steps", type=int, help="IK points along the unhook")
    parser.add_argument(
        "--unhook-distance",
        type=float,
        help="[m] the hook back along its opening (clear of the bar by --bar-margin at the end)",
    )
    parser.add_argument(
        "--unhook-drop",
        type=float,
        help="[m] the body goes down while the hook comes out of the bar (the arms take the difference)",
    )
    # for the optimisation
    parser.add_argument(
        "--armature",
        type=float,
        help="[kg m^2] reflected inertia of the servos of the arm joints and gimbals, written "
        "for the trajectory optimisation (an assumed value, not measured)",
    )

    keys = {a.dest for a in parser._actions} - {"help", "config", "no_plot"}
    config_path = parser.parse_known_args()[0].config
    with open(config_path) as f:
        config = yaml.safe_load(f) or {}
    unknown, missing = sorted(set(config) - keys), sorted(keys - set(config))
    if unknown or missing:
        parser.error("%s: unknown keys %s, missing keys %s" % (config_path, unknown, missing))
    parser.set_defaults(**config)
    args = parser.parse_args()
    for key in ("robot", "scene", "output"):
        setattr(args, key, resolve(getattr(args, key)))
    return args, package


def main():
    args, package = parse_args()

    from ompl import util as ou

    ou.RNG.setSeed(args.seed)
    clock = time.time()
    urdf = xacro(args.robot)
    scene_urdf = xacro(args.scene, bar_x=args.bar_x, bar_z=args.bar_z, bar_radius=args.bar_radius, wall_x=args.wall_x)
    robot = Robot(
        urdf,
        scene_urdf,
        args.margin,
        BodyAttitude(
            tuple(args.reference_attitude),
            args.max_body_tilt,
            args.attitude_weight,
            tuple(bool(v) for v in args.attitude_mask),
        ),
    )
    robot.hook_pitch = HookPitch(radius=args.bar_radius, range=args.hook_pitch_range, weight=args.hook_pitch_weight)
    weight = robot.mass * 9.81
    print(
        "robot: %.3f kg (%.2f N), %d rotors of %.1f..%.1f N"
        % (robot.mass, weight, len(robot.gimbals), robot.thrust_min, robot.thrust_max)
    )

    # --- the key postures
    normal = np.array([-np.sin(args.hook_tilt), 0.0, np.cos(args.hook_tilt)])  # the bar pushing the hook
    bar = np.array([args.bar_x, 0.0, args.bar_z])
    hook = (bar + args.bar_radius * normal, normal)
    x_start = np.zeros(12)
    x_start[:3] = args.start
    x_start[3:6] = robot.attitude.rotation_vector(np.zeros(3))  # hovering at the reference attitude
    folded = np.array(args.folded_joints, float)
    x_start[6:] = np.concatenate([folded, folded])
    guess = x_start.copy()
    guess[:3] = [args.bar_x, 0.0, args.bar_z - 0.3]
    guess[6:9] = [-1.4, -1.0, -1.0]  # up, then curled back over the bar
    x_hang, error = robot.ik(guess, hook=hook, rear_joints=np.array(args.hang_rear_joints, float), com_x=hook[0][0])
    print(
        "hang: root (%.3f, %.3f), front arm %s, IK error %s" % (x_hang[0], x_hang[2], np.round(x_hang[6:9], 3), error)
    )
    # off the bar along the opening of the hook: the approach moves the robot (as one) so that the bar
    # comes in through the opening onto the hook point
    frames_hang, _ = robot.kinematics(x_hang)
    x_pre = x_hang.copy()
    x_pre[:3] -= args.approach * (frames_hang[FRONT_TIP].rotation @ HOOK_OPENING)
    print(
        "approach: from %s, along %s"
        % (np.round(x_pre[:3], 3), np.round(frames_hang[FRONT_TIP].rotation @ HOOK_OPENING, 3))
    )

    front_on_bar = robot.pairs({(FRONT_HOOK, "env/env_bar")})
    rear_on_wall = robot.pairs({(REAR_LINK3, "env/env_wall")})
    check(robot, "start", x_start, args, args.margin, args.bar_margin)
    check(robot, "off the bar", x_pre, args, args.margin, args.bar_margin)

    # rear arm: swung out to the standoff, then onto the wall; the IK holds the hook all along
    frames_rear, _ = robot.kinematics(x_hang)
    rear_from = frames_rear[REAR_TIP].translation.copy()
    rear_to = np.array([args.wall_x, 0.0, x_hang[2]])
    rear_off = rear_to + np.array([args.standoff, 0.0, 0.0])
    rate = args.rate
    # The path is solved by IK at a fixed number of points (--reach-ik-steps, --touch-ik-steps) and then
    # timed: each IK starts from the previous solution, so the steps decide which way the arms go, and
    # the duration (the number of frames) must not.
    reach, touch, worst = [], [], {"hook": 0.0, "rear_tip": 0.0}
    x = x_hang
    for out, a, b, steps, duration, aligned in (
        (reach, rear_from, rear_off, args.reach_ik_steps, args.reach_time, False),
        (touch, rear_off, rear_to, args.touch_ik_steps, args.touch_time, True),
    ):
        path = [x]
        for k in range(1, steps + 1):
            s = k / steps
            # the hand turns to face the wall gradually over the reach, and stays so on the touch
            x, error = robot.ik(
                x,
                hook=hook,
                rear_tip=a + s * (b - a),
                com_x=hook[0][0],
                previous=x,
                rear_direction=np.array([-1.0, 0.0, 0.0]),
                direction_weight=3.0 if aligned else 3.0 * s,
            )
            path.append(x)
            worst = {key: max(worst[key], error.get(key, 0.0)) for key in worst}
        frames = max(int(round(duration * rate)), 1)  # the IK points spread evenly over the duration
        out.extend(resample(path, np.linspace(0.0, frames / rate, len(path)), rate)[1:])
    print(
        "reach/touch: worst IK error, hook %.1f mm, rear tip %.1f mm" % (1e3 * worst["hook"], 1e3 * worst["rear_tip"])
    )
    x_touch = touch[-1]

    # --- leaving the bar (after the load is back on the rotors): the rear arm back along the reach,
    # then the hook out of the bar the way it came in (its point back along the opening by
    # --unhook-distance, its pose held) while the body goes down by --unhook-drop and the arms take the
    # difference
    retract = list(reversed(reach + touch))[1:] + [x_hang]
    hook_point = robot.hook_point(frames_hang)
    hook_normal_hang = -frames_hang[FRONT_LINK3].rotation[:, 2]
    opening = frames_hang[FRONT_TIP].rotation @ HOOK_OPENING
    hinge, robot.hook_pitch = robot.hook_pitch, HookPitch(radius=args.bar_radius)  # the hook pose held
    path, x, worst_unhook = [x_hang], x_hang, 0.0
    for k in range(1, args.unhook_ik_steps + 1):
        s = k / args.unhook_ik_steps
        x, error = robot.ik(
            x,
            hook=(hook_point - s * args.unhook_distance * opening, hook_normal_hang),
            rear_joints=np.array(args.hang_rear_joints, float),
            root_z=x_hang[2] - s * args.unhook_drop,
            previous=x,
        )
        path.append(x)
        worst_unhook = max(worst_unhook, error["hook"])
    robot.hook_pitch = hinge
    unhook = resample(path, np.linspace(0.0, args.unhook_time, len(path)), rate)[1:]
    x_off = unhook[-1]
    print(
        "unhook: root down %.3f m, hook %.3f m along the opening, worst IK error %.1f mm"
        % (x_hang[2] - x_off[2], args.unhook_distance, 1e3 * worst_unhook)
    )
    check(robot, "unhooked", x_off, args, args.margin, args.bar_margin)

    # --- flight
    robot.set_margin(args.margin, args.bar_margin)
    flight = plan_flight(robot, x_start, x_pre, args)
    speed = {"position": args.flight_speed, "rotation": args.flight_rotation_speed, "joint": args.flight_joint_speed}
    frames_of = lambda duration: max(int(round(duration * rate)), 1)  # noqa: E731
    phases = [
        ("hover", [x_start] * frames_of(args.hover_time)),
        ("flight", resample(flight, timed(flight, speed), rate)[1:]),
        ("pre", [x_pre] * frames_of(args.pre_time)),
        ("approach", line(x_pre, x_hang, args.approach_time, rate)),
        ("load", [x_hang] * frames_of(args.load_time)),
        ("hang", [x_hang] * frames_of(args.hang_time)),
        ("reach", reach),
        ("touch", touch),
        ("push", [x_touch] * frames_of(args.push_time)),
        ("hold", [x_touch] * frames_of(args.hold_time)),
        ("release", [x_touch] * frames_of(args.release_time)),
        ("retract", retract),
        ("unhook", unhook),
        ("end", [x_off] * frames_of(args.hover_time)),
    ]  # off the bar: on by teleoperation
    # the start of the release (the operator lets it go on from there: perching_task.py)
    release_time = (
        sum(len(xs) for name, xs in phases[: phases.index(next(p for p in phases if p[0] == "release"))]) / rate
    )

    allowed = {
        "approach": front_on_bar,
        "load": front_on_bar,
        "hang": front_on_bar,
        "reach": front_on_bar,
        "touch": front_on_bar | rear_on_wall,
        "push": front_on_bar | rear_on_wall,
        "hold": front_on_bar | rear_on_wall,
        "release": front_on_bar | rear_on_wall,
        "retract": front_on_bar | rear_on_wall,
        "unhook": front_on_bar,
    }
    for name, xs in phases:
        robot.set_allowed(allowed.get(name, set()))
        if name in ("hover", "flight", "pre", "end"):
            robot.set_margin(args.margin, args.bar_margin)
        else:
            robot.set_margin(0.0)
        bad = [robot.collisions(robot.q(x)) for x in xs]
        hits = sorted({pair for b in bad for pair in b})
        print("%-9s %3d frames, %d in collision %s" % (name, len(xs), sum(1 for b in bad if b), hits if hits else ""))
    robot.set_allowed(set())

    # --- statics along the path
    # each contact frame has its z along the surface normal (the direction the scene pushes the robot)
    # the hook normal is that of the hook itself (-z of its last link), which turns with it on the bar
    hook_normal = -frames_hang[FRONT_LINK3].rotation[:, 2]
    placements = {
        HOOK: (FRONT_TIP, contact_placement(robot, x_hang, FRONT_TIP, HOOK_OFFSET, hook_normal)),
        PUSH: (REAR_TIP, contact_placement(robot, x_touch, REAR_TIP, np.zeros(3), np.array([1.0, 0, 0]))),
    }
    effort_limits = {n: args.max_effort_ratio * limit for n, limit in robot.effort.items()}
    statics = Statics(
        robot,
        placements,
        StaticsLimits(
            min_up=args.min_lift * weight / len(robot.gimbals),
            max_thrust_tilt=args.max_thrust_tilt,
            effort=effort_limits,
        ),
    )
    print(
        "joint torques: at most %.0f%% of the effort limits, %s Nm"
        % (100.0 * args.max_effort_ratio, {n: round(v, 2) for n, v in effort_limits.items()})
    )
    print(
        "thrust: at least %.2f N up per rotor (%.0f%% of the weight in all), at most %.0f deg off up"
        % (statics.min_up, 100.0 * statics.min_up * len(robot.gimbals) / weight, np.degrees(args.max_thrust_tilt))
    )
    # the gimbal angles that point the thrust most upward, which change with the attitude of the body:
    # the optimisation bounds each gimbal to the tilt around the range they take along the plan (a
    # joint limit cannot follow the attitude)
    upright = np.array([robot.with_upright_gimbals(robot.raw_q(x)) for _, xs in phases for x in xs])
    gimbal_limits = {
        g[0]: [
            float(upright[:, robot.idx_q[g[0]]].min() - args.max_thrust_tilt),
            float(upright[:, robot.idx_q[g[0]]].max() + args.max_thrust_tilt),
        ]
        for g in robot.gimbals
    }
    push_force = np.array([args.push_force, 0.0, 0.0])  # the wall pushing the hand back, world axes
    thrust_frames = [g[2] for g in robot.gimbals]
    joint_names = ARM_JOINTS + [g[0] for g in robot.gimbals]

    frames, records, failed = [], [], 0  # frames go to the file; records (per frame) only to the summary
    for name, xs in phases:
        for k, x in enumerate(xs):
            # the release: the share of the load and of the push going back to zero (the load in reverse)
            unload = 1.0 - (k + 1) / len(xs) if name == "release" else None
            scale = {"push": (k + 1) / len(xs), "hold": 1.0, "release": unload}.get(name, 0.0)
            perched = name in PERCH_PHASES or (unload is not None and unload > 0.0)
            result = statics.solve(x, hook=args.friction if perched else None, push=scale * push_force)
            if name in ("load", "release") and perched:  # the share of the full hook force the bar may take
                share = (k + 1) / len(xs) if name == "load" else unload
                full_normal = statics._world(HOOK).rotation[:, 2] @ result["hook"]
                if share < 1.0:
                    result = statics.solve(
                        x, hook=args.friction, push=scale * push_force, hook_max_normal=share * full_normal
                    )
            free = statics.solve(x, push=scale * push_force) if perched else result
            failed += not result["solved"]
            contacts = {}
            if perched:
                contacts[HOOK] = [float(v) for v in statics._world(HOOK).rotation.T @ result["hook"]]
            if scale > 0.0:
                contacts[PUSH] = [float(v) for v in statics._world(PUSH).rotation.T @ (scale * push_force)]
            frames.append(
                dict(
                    root=task_to_world(args.task_frame, result["q"][:7]),
                    joints=[float(result["q"][robot.idx_q[n]]) for n in joint_names],
                    thrust=[float(v) for v in result["thrust"]],
                    contacts=contacts,
                    effort=[result["effort"].get(n, 0.0) for n in joint_names],
                )
            )
            records.append(
                dict(
                    phase=name,
                    thrust=float(result["thrust"].sum()),
                    free=float(free["thrust"].sum()) if free["solved"] else np.nan,
                    carry=contacts.get(HOOK, [0, 0, 0])[2],
                    torque=max(abs(result["effort"].get(n, 0.0)) for n in ARM_JOINTS),
                )
            )
    print("statics: %d of %d frames did not solve" % (failed, len(frames)))

    # --- summary
    print(
        "\n%-9s %11s %14s %7s %14s %14s"
        % ("phase", "thrust [N]", "no perch [N]", "saved", "bar carry [N]", "max |tau| [Nm]")
    )
    for name, _ in phases:
        own = [r for r in records if r["phase"] == name]
        perch, free = np.mean([r["thrust"] for r in own]), np.nanmean([r["free"] for r in own])
        print(
            "%-9s %11.2f %14.2f %6.0f%% %14.2f %14.2f"
            % (
                name,
                perch,
                free,
                100.0 * (1.0 - perch / free),
                np.mean([r["carry"] for r in own]),
                max(r["torque"] for r in own),
            )
        )
    print("weight %.2f N; total %.1f s" % (weight, time.time() - clock))

    # The file: the posture, thrust and the wrench of each active contact per frame, every dt. A
    # contact is active in a frame when the frame has its force (in the contact frame); "support"
    # contacts carry what the statics gives them inside their friction cone, "prescribed" ones the
    # force the task asks for.
    robot_path = os.path.abspath(args.robot)
    if robot_path.startswith(package + os.sep):
        robot_path = "package://gimbalrotor/" + os.path.relpath(robot_path, package)
    contact_defs = {}
    for name, (link, placement) in placements.items():
        contact_defs[name] = dict(
            link=link,
            xyz=[float(v) for v in placement.translation],
            quaternion=[float(v) for v in pin.Quaternion(placement.rotation).coeffs()],
            friction=args.friction,
            type="point",
            mode="support" if name == HOOK else "prescribed",
        )
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w") as f:
        json.dump(
            dict(
                description="hover -> hang on the bar -> push the wall, planned by plan_perching_motion.py",
                # the root of the frames is in the world; the plan was made in this frame of it
                task_frame=[float(v) for v in args.task_frame],
                # [s] of the file: where the release begins (the load back onto the rotors, then
                # the rear arm back and the hook out of the bar, the body going down)
                events={"release": float(release_time)},
                robot=robot_path,
                dt=1.0 / rate,
                joint_names=joint_names,
                thrust_frames=thrust_frames,
                # the gimbals turn to point the thrust; the optimisation is free to choose them
                joint_weights={g[0]: 0.0 for g in robot.gimbals},
                # what keeps the attitude under control: every rotor pushing up (the
                # optimisation bounds the thrust, not its upward part) and the gimbals near up
                thrust_limits={"min": statics.min_up},
                joint_limits=gimbal_limits,
                effort_limits=effort_limits,
                armature={n: args.armature for n in joint_names},
                contacts=contact_defs,
                frames=frames,
            ),
            f,
        )
    print("wrote %s: %d frames, %.2f s" % (args.output, len(frames), (len(frames) - 1) / rate))
    if not args.no_plot:
        plot(os.path.splitext(args.output)[0] + ".png", records, [(n, len(xs)) for n, xs in phases], weight, rate)
    sys.stdout.flush()
    os._exit(0)  # skip the nanobind leak report of the ompl bindings at exit


if __name__ == "__main__":
    main()
