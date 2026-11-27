"""Perching model, IK tasks, collision geometry and static load allocation.

The motion phases and command line interface live in plan_perching_motion.py.
The reusable numerical IK, collision scene and statics helpers are in aerial_motion_planning.
"""

from __future__ import annotations

import time
import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass

import numpy as np
import pinocchio as pin

from aerial_motion_planning.ik import IKProblem
from aerial_motion_planning.model import FreeFlyerModel
from aerial_motion_planning.collision import CollisionScene
from gimbalrotor.rotor_statics import GimballedRotorStatics, StaticsLimits

ARM_JOINTS = ["front_arm_joint%d" % i for i in (1, 2, 3)] + ["back_arm_joint%d" % i for i in (1, 2, 3)]
JOINT_MARGIN = 0.05  # [rad], the planning keeps this far inside the joint limits of the URDF
ROOM = np.array([[-0.3, 2.0], [-0.5, 0.5], [0.6, 2.2]])  # [m], where the root may go in flight
FRONT_TIP, REAR_TIP = "front_arm_end_effector", "back_arm_end_effector"
FRONT_LINK3, REAR_LINK3 = "front_arm_link3", "back_arm_link3"
FRONT_HOOK = "front_arm_hook"  # the hook module on FRONT_TIP (robots/quad_with_dual_arm/gimbalrotor.urdf.xacro)
# the parts of the front arm that pass over the bar in flight (the body and the first link stay below)
OVER_BAR = ("front_arm_link2", FRONT_LINK3, FRONT_HOOK)
# The hook in the FRONT_TIP frame: two C-shaped plates (y = +-0.016..0.020) whose inner edge is about a
# circle of centre (0.0443, 0, 0.0005) and radius 0.042 in the xz plane, open from 66 to 156 deg (from
# +x towards +z). The bar rests on the inside at the -z end of that circle (the inner surface measured
# at x = 0.0443): the hook point, where the bar pushes the hook along -z of FRONT_TIP (as the IK sets
# the last link: +z facing -normal).
HOOK_OFFSET = np.array([0.0443, 0.0, -0.0420])  # [m], the hook point in FRONT_TIP
HOOK_OPENING = np.array([np.cos(np.radians(111.0)), 0.0, np.sin(np.radians(111.0))])  # the middle of the opening
PERCH_PHASES = ("load", "hang", "reach", "touch", "push", "hold")
HOOK, PUSH = "hook_bar", "rear_hand_wall"  # the contact and force frames


@dataclass
class BodyAttitude:
    """The attitude of the body in the IK and the flight planning: R = R_ref exp(offset), around a
    reference that suits the robot and its tasks; tilting away from it is a risk."""

    reference_rpy: tuple = (0.0, 0.0, 0.0)  # [rad] roll, pitch, yaw of R_ref
    max_tilt: float = 0.0  # [rad] the largest offset, on each component; 0: at R_ref
    weight: float = 0.0  # IK residual pulling the offset back to zero (per rad)
    mask: tuple = (True, True, True)  # free offsets: roll, pitch, yaw

    def __post_init__(self):
        self.reference = pin.rpy.rpyToMatrix(*[float(v) for v in self.reference_rpy])

    def rotation_vector(self, offset):
        """The rotation vector (world, as in the planning vector x) of the attitude at offset."""
        return pin.log3(self.reference @ pin.exp3(np.asarray(offset, float)))

    def offset(self, rotation_vector):
        """The offset from the reference of the attitude of a rotation vector."""
        return pin.log3(self.reference.T @ pin.exp3(np.asarray(rotation_vector, float)))


@dataclass
class IKWeights:
    """Weights of the IK residuals (Robot.ik); the direction of the rear hand has its own, ramped
    along the reach."""

    hook_position: float = 100.0  # per m
    hook_normal: float = 10.0  # the last link of the front arm against the bar normal
    rear_tip: float = 100.0  # per m
    rear_joints: float = 10.0  # per rad, when the rear arm holds its joints
    com: float = 10.0  # per m, the CoM under the hook (x) and on the plane of symmetry (y)
    root_z: float = 100.0  # per m, the root height when given
    previous: float = 0.3  # per rad, the attitude and the joints close to the previous posture


@dataclass
class HookPitch:
    """How the hook may turn on the bar in the IK. A hook on a round bar is a hinge about the bar: it
    turns about the bar axis (its pitch, all the arm joints being pitch joints), the point of the
    hook that sits on the bar going round the bar with it, and its roll and yaw are held by the two
    plates of the hook on either side of the bar. With range 0 the hook is fixed instead: its point on
    the bar and the last link along -normal (the pitch held at --hook-tilt)."""

    radius: float = 0.0  # [m] of the bar: the centre of the bar is this far inside the hook point
    range: float = 0.0  # [rad] the pitch (the tilt of the normal of the hook) may differ by from
    #                       --hook-tilt; 0: fixed
    weight: float = 1.0  # per rad, pulling the pitch to --hook-tilt (soft)
    limit_weight: float = 100.0  # per rad, past range


def hull_depth(points, p):
    """Signed distance of p inside the 2D convex hull of points (negative outside)."""
    if len(points) < 3:
        return -1.0
    pts = sorted(map(tuple, points))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for pt in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], pt) <= 0:
            lower.pop()
        lower.append(pt)
    for pt in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], pt) <= 0:
            upper.pop()
        upper.append(pt)
    hull = np.array(lower[:-1] + upper[:-1])
    depth = np.inf
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        edge = b - a
        normal = np.array([-edge[1], edge[0]]) / np.linalg.norm(edge)  # inward for a ccw hull
        depth = min(depth, normal @ (p - a))
    return depth


class Robot(FreeFlyerModel):
    """Kinematics, collision, IK and the flight test of the planning model.

    The planning vector x is [root position, root rotation vector, ARM_JOINTS]."""

    def __init__(self, urdf, env_urdf, margin, attitude=None, ik_weights=None):
        """attitude: BodyAttitude (default: held level); ik_weights: IKWeights."""
        self.attitude = attitude or BodyAttitude()
        self.ik_weights = ik_weights or IKWeights()
        self.hook_pitch = HookPitch()  # the hook fixed on the bar unless its range is set
        super().__init__(urdf)
        self.tree = ElementTree.fromstring(urdf)
        joints = {j.get("name"): j for j in self.tree.findall("joint")}
        self.gimbals = []  # (joint name, axis in the joint frame, thrust frame name, rotor direction)
        for name, joint in sorted(joints.items()):
            if name.startswith("gimbal") and joint.get("type") == "revolute":
                index = name[len("gimbal") :]
                axis = np.array([float(v) for v in joint.find("axis").get("xyz").split()])
                direction = float(joints["rotor" + index].find("axis").get("xyz").split()[2])
                self.gimbals.append((name, axis / np.linalg.norm(axis), "thrust" + index, np.sign(direction)))
        rotor_limit = joints["rotor1"].find("limit")
        self.thrust_min, self.thrust_max = float(rotor_limit.get("lower")), float(rotor_limit.get("upper"))
        self.drag_rate = float(self.tree.find("m_f_rate").get("value"))  # [Nm/N]
        self.idx_q = {
            n: self.model.joints[self.model.getJointId(n)].idx_q for n in ARM_JOINTS + [g[0] for g in self.gimbals]
        }
        self.arm_q = [self.idx_q[n] for n in ARM_JOINTS]
        limits = [joints[n].find("limit") for n in ARM_JOINTS]
        self.joint_low = np.array([float(l.get("lower")) for l in limits]) + JOINT_MARGIN
        self.joint_high = np.array([float(l.get("upper")) for l in limits]) - JOINT_MARGIN
        self.effort = {n: float(joints[n].find("limit").get("effort")) for n in self.idx_q}  # [Nm], URDF
        self.frame = {n: self.model.getFrameId(n) for n in (FRONT_TIP, REAR_TIP, FRONT_LINK3)}
        begun = time.perf_counter()
        self._build_collision(env_urdf)
        self.set_margin(margin)
        self.collision_setup_time = time.perf_counter() - begun  # [s] for the benchmark

    def _build_collision(self, env_urdf):
        self.scene = CollisionScene(
            self.model,
            self.data,
            ElementTree.tostring(self.tree, encoding="unicode"),
            env_urdf,
            [g[2] for g in self.gimbals],
        )
        # Keep the old attributes for the task's collision diagnostics.
        self.geom, self.gdata = self.scene.geom, self.scene.gdata
        self.pair_links = self.scene.pair_links
        self.env_pair_start = self.scene.env_pair_start

    def pairs(self, wanted):
        return self.scene.pairs(wanted)

    def set_margin(self, margin, bar_margin=None):
        overrides = {(link, "env/env_bar"): bar_margin for link in OVER_BAR} if bar_margin is not None else None
        self.scene.set_margin(margin, overrides)

    def set_allowed(self, allowed):
        self.scene.set_allowed(allowed)

    def collisions(self, q):
        return self.scene.collisions(q)

    def in_collision(self, q):
        return self.scene.in_collision(q)

    # --- configuration
    def raw_q(self, x):
        return self.configuration(x[:3], x[3:6], dict(zip(ARM_JOINTS, x[6:])))

    def q(self, x):
        return self.with_upright_gimbals(self.raw_q(x))

    def with_upright_gimbals(self, q):
        """Each gimbal turned so that its thrust axis is as close to straight up as it gets: for a
        thrust axis r(0) rotating about an axis a, z.r(t) is largest at
        t = atan2(z.(a x r0), z.r0 - (a.r0)(a.z))."""
        q = q.copy()
        for name, _, _, _ in self.gimbals:
            q[self.idx_q[name]] = 0.0
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        z = np.array([0.0, 0.0, 1.0])
        for name, axis, thrust, _ in self.gimbals:
            a = self.data.oMi[self.model.getJointId(name)].rotation @ axis
            r0 = self.data.oMf[self.model.getFrameId(thrust)].rotation[:, 2]
            q[self.idx_q[name]] = np.arctan2(z @ np.cross(a, r0), z @ r0 - (a @ r0) * (a @ z))
        return q

    def flight_margin(self, q):
        """(distance of the CoM inside the hull of the rotors that can point up, lift / weight)."""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        com = pin.centerOfMass(self.model, self.data, q)
        points, lift = [], 0.0
        for _, _, thrust, _ in self.gimbals:
            placement = self.data.oMf[self.model.getFrameId(thrust)]
            up = placement.rotation[2, 2]
            if up > 0.5:
                points.append(placement.translation[:2])
                lift += self.thrust_max * up
        return hull_depth(np.array(points), com[:2]), lift / (self.mass * 9.81)

    # --- IK
    def kinematics(self, x):
        q = self.raw_q(x)
        com = pin.centerOfMass(self.model, self.data, q)  # also runs the forward kinematics
        pin.updateFramePlacements(self.model, self.data)
        # copies: the placements in data change with the next kinematics
        return {name: self.data.oMf[i].copy() for name, i in self.frame.items()}, com.copy()

    def hook_point(self, frames):
        return frames[FRONT_TIP].act(HOOK_OFFSET)

    def hook_centre(self, frames, radius):
        """The centre of a bar of radius sitting on the hook point (it pushes the hook along -z of
        FRONT_TIP, so the centre is along +z)."""
        return frames[FRONT_TIP].act(HOOK_OFFSET + radius * np.array([0.0, 0.0, 1.0]))

    def ik(
        self,
        x0,
        hook=None,
        rear_tip=None,
        rear_direction=None,
        rear_joints=None,
        com_x=None,
        previous=None,
        direction_weight=3.0,
        check_collision=False,
        root_z=None,
    ):
        """The root pose (position, and attitude within self.attitude.max_tilt of the reference on
        each component of the offset) and the arm joints for the tasks given:

          hook            (point, normal): the hook point of the front arm on the point, the +z axis
                          of its last link along -normal
          rear_tip        point for the tip of the rear arm (x, z)
          rear_direction  direction of the last link of the rear arm (soft, by direction_weight)
          rear_joints     joint angles held by the rear arm instead of a tip target (soft)
          com_x           x of the CoM (soft; its y is held at 0, on the plane of symmetry)
          root_z          height of the root (soft)
          previous        x the root attitude and the joints are kept close to (the attitude is
                          redundant to the arms: this picks one)
          check_collision  reject a result that introduces a collision (the caller must first allow
                           any intended contact pairs with set_allowed)

        The targets are points and directions in the world (y included). The attitude is solved as
        its offset from the reference, which the IK pulls back to zero by self.attitude.weight:
        tilting away from the reference is a risk, taken only as far as the tasks need it."""
        previous = x0 if previous is None else previous
        attitude, w = self.attitude, self.ik_weights
        previous_offset = attitude.offset(previous[3:6])

        # the variables: x with the offset in place of the rotation vector; the attitude is one only
        # when it may tilt (least_squares needs low < high), else it is the reference
        free = np.ones(12, bool)
        free[3:6] = np.asarray(attitude.mask, bool) & (attitude.max_tilt > 0.0)
        fixed = np.asarray(x0, float).copy()
        fixed[3:6] = np.clip(attitude.offset(x0[3:6]), -attitude.max_tilt, attitude.max_tilt)
        fixed[3:6][~free[3:6]] = 0.0

        def unpack(y):
            x = y.copy()
            x[3:6] = attitude.rotation_vector(y[3:6])
            return x, y[3:6]

        def residual(y):
            x, offset = unpack(y)
            frames, com = self.kinematics(x)
            r = []
            if rear_tip is not None:
                r += list(w.rear_tip * (frames[REAR_TIP].translation - rear_tip))
            if rear_direction is not None:
                r += list(direction_weight * (frames[REAR_TIP].rotation[:, 0] - rear_direction))
            if rear_joints is not None:
                r += list(w.rear_joints * (x[9:] - rear_joints))
            if com_x is not None:
                r += [w.com * (com[0] - com_x), w.com * com[1]]
            if root_z is not None:
                r += [w.root_z * (x[2] - root_z)]
            if hook is not None and hinge:
                # the pitch of the hook on the bar: the tilt of its normal (-z of the last link) from up
                tilt = hook_tilt(frames) - reference_tilt
                r += [hp.weight * tilt, hp.limit_weight * max(abs(tilt) - hp.range, 0.0)]
            r += list(w.previous * (offset - previous_offset))
            r += list(w.previous * (x[6:] - previous[6:]))
            r += list(attitude.weight * offset)  # away from the reference only as far as it helps
            return np.array(r)

        hp = self.hook_pitch
        hinge = hook is not None and hp.range > 0.0
        if hook is not None:
            point, normal = hook
            reference_tilt = np.arctan2(-normal[0], normal[2])
            centre = point - hp.radius * normal  # of the bar
            bar_axis = np.array([0.0, 1.0, 0.0])

        def hook_tilt(frames):
            n = -frames[FRONT_LINK3].rotation[:, 2]
            return np.arctan2(-n[0], n[2])

        def contact_residual(y):
            x, _ = unpack(y)
            frames, _ = self.kinematics(x)
            if hinge:  # a hinge about the bar: the centre of the bar in the hook on it, the pitch free
                return np.concatenate(
                    [
                        w.hook_position * (self.hook_centre(frames, hp.radius) - centre),
                        w.hook_normal * (frames[FRONT_LINK3].rotation[:, 1] - bar_axis),
                    ]
                )
            return np.concatenate(
                [
                    w.hook_position * (self.hook_point(frames) - point),
                    w.hook_normal * (frames[FRONT_LINK3].rotation[:, 2] + normal),
                ]
            )

        low = np.concatenate([ROOM[:, 0] - 1.0, [-attitude.max_tilt] * 3, self.joint_low])
        high = np.concatenate([ROOM[:, 1] + 1.0, [attitude.max_tilt] * 3, self.joint_high])
        low[2] = 0.0

        def collisions(y):
            x, _ = unpack(y)
            return set(map(tuple, self.collisions(self.q(x))))

        solution = IKProblem(
            residual,
            low,
            high,
            mask=free,
            equality=contact_residual if hook is not None else None,
            collision=collisions if check_collision else None,
            contact_weight=1.0,
        ).offline(fixed, xtol=1e-10, ftol=1e-10)
        x, _ = unpack(solution)
        frames, _ = self.kinematics(x)
        error = {}
        if hook is not None:
            if hinge:
                error["hook"] = float(np.linalg.norm(self.hook_centre(frames, hp.radius) - centre))
                error["hook_tilt"] = float(hook_tilt(frames))
            else:
                error["hook"] = float(np.linalg.norm(self.hook_point(frames) - hook[0]))
        if rear_tip is not None:
            error["rear_tip"] = float(np.linalg.norm(frames[REAR_TIP].translation - rear_tip))
        return x, error


class Statics(GimballedRotorStatics):
    """Perching frame names and output keys for the shared static solver."""

    def solve(self, x, hook=None, push=None, hook_max_normal=None, iterations=30, warm_start=None):
        result = super().solve(
            x,
            contact_name=HOOK if hook is not None else None,
            friction=hook,
            force_name=PUSH if push is not None else None,
            force=push,
            contact_max_normal=hook_max_normal,
            iterations=iterations,
            warm_start=warm_start,
        )
        result["hook"] = result.pop("contact")
        return result
