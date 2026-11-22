"""Whole-body IK of gimbalrotor quad_with_dual_arm hanging from a bar by the hook of its front arm: the
hand of the rear arm is moved freely while the hook stays on the bar, the body and the front arm
following as they need to.

Planar: all the arm joints are pitch joints, so the arms move in the vertical plane through the root
along the heading of the body (the plane of the arms). The variables p (9) are

  p[0]    the root along the heading, from where it was at the start [m]
  p[1]    the root height (world z) [m]
  p[2]    the body pitch [rad]
  p[3:6]  the front arm joints, p[6:9] the rear arm joints [rad]

The yaw, the lateral position of the root and the gimbals stay as they were at the start, the roll at 0.
Points in the plane are (u, z): u along the heading from the start root, z the world height; angles
in the plane are atan2(z, u) of a direction.

Each iteration is one QP (ProxQP) on the step dp:

  hook       the hook point stays where it was at the start (on the bar)            equality
  hook angle the last link of the front arm keeps its angle (the hook seat on the bar)  weight_hook_angle
  com        the CoM right below the hook point, so that the bar carries the weight     weight_com
             with little moment for the rotors
  tip        the rear hand on its target (u, z)                                        weight_tip
  direction  the last link of the rear arm at the target angle (if given)              weight_direction
  attitude   the body pitch back to its reference                                       weight_attitude
  damping    small steps (the redundancy goes where the posture changes least)          damping

subject to the joint limits (joint_margin inside those of the URDF), the pitch within max_pitch of
its reference, the step bounds given (the speed of each variable over one control period) and the
static torque of the rear arm joints within max_effort_ratio of the effort limits of the URDF
(linearised; with the hand free, only the gravity of the arm loads them). The robot against itself
(the convex hulls of the meshes, the propellers as the cylinders they sweep, collision_margin apart;
the scene is not checked), once per call: a step that brings two links into collision that were not
is cut to a half, a quarter, or dropped.

The hook is kept by an equality: a target out of reach does not pull the hook off the bar, the hand
stops at the closest point it gets to. If the equality cannot be met (at the joint limits), the hook
falls back to a residual of weight_hook.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from xml.etree import ElementTree

import numpy as np
import pinocchio as pin
from aerial_motion_planning.ik import IKProblem, prepare_collision_geometry


FRONT_JOINTS = ("front_arm_joint1", "front_arm_joint2", "front_arm_joint3")
REAR_JOINTS = ("back_arm_joint1", "back_arm_joint2", "back_arm_joint3")


@dataclass
class HangingIKConfig:
    hook_link: str = "front_arm_end_effector"  # the hook point is fixed in this link
    hook_offset: tuple = (0.0443, 0.0, -0.0420)  # [m] (HOOK_OFFSET of plan_perching_motion.py)
    tip_link: str = "back_arm_end_effector"  # the hand; its x axis is its direction
    front_joints: tuple = FRONT_JOINTS
    rear_joints: tuple = REAR_JOINTS
    joint_margin: float = 0.05  # [rad] inside the joint limits of the URDF
    pitch_reference: float = 0.0  # [rad]
    max_pitch: float = 0.3  # [rad] from the reference
    max_effort_ratio: float = 0.75  # of the effort limits of the URDF (rear arm)
    weight_hook: float = 100.0  # per m (only if the equality fails)
    weight_hook_angle: float = 10.0  # per rad
    weight_com: float = 10.0  # per m
    weight_tip: float = 10.0  # per m
    weight_direction: float = 1.0  # per rad
    weight_attitude: float = 1.0  # per rad
    damping: float = 0.3  # per m or rad of the step
    max_tip_error: float = 0.05  # [m] the tip error is clipped to this in the residual
    check_collision: bool = True
    collision_margin: float = 0.01  # [m]


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class Features:
    """The task quantities at a posture: points (u, z) and angles in the plane, the rear arm torques."""

    hook: np.ndarray
    hook_angle: float
    com: np.ndarray
    tip: np.ndarray
    tip_angle: float
    torque: np.ndarray  # static torques of the rear arm joints [Nm]

    def vector(self):
        return np.concatenate([self.hook, [self.hook_angle, self.com[0]], self.tip, [self.tip_angle], self.torque])

    ANGLES = (2, 6)  # elements of vector() that are angles


@dataclass
class Result:
    p: np.ndarray
    features: Features
    hook_error: float = 0.0  # [m]
    tip_error: float = 0.0  # [m]
    angle_error: float = 0.0  # [rad] of the hand direction (0 if not tracked)
    hook_soft: bool = False  # the hook equality could not be met
    blocked: list = field(default_factory=list)  # link pairs that stopped a step


class HangingIK:
    def __init__(self, urdf, config=None):
        self.config = config or HangingIKConfig()
        c = self.config
        self.model = pin.buildModelFromXML(urdf, pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        self.mass = pin.computeTotalMass(self.model)
        tree = ElementTree.fromstring(urdf)
        baselink = tree.find("baselink")
        if baselink is None:
            raise RuntimeError("the URDF has no <baselink name=...>")
        self.baselink = self.model.getFrameId(baselink.get("name"))
        self.hook_frame = self.model.getFrameId(c.hook_link)
        self.tip_frame = self.model.getFrameId(c.tip_link)
        self.hook_offset = np.asarray(c.hook_offset, float)
        self.joint_names = list(c.front_joints) + list(c.rear_joints)
        joints = [self.model.joints[self.model.getJointId(n)] for n in self.joint_names]
        self.idx_q = [j.idx_q for j in joints]
        rear_v = [j.idx_v for j in joints[3:]]
        self.rear_v = rear_v
        low = self.model.lowerPositionLimit[self.idx_q] + c.joint_margin
        high = self.model.upperPositionLimit[self.idx_q] - c.joint_margin
        self.low = np.concatenate([[-np.inf, -np.inf, c.pitch_reference - c.max_pitch], low])
        self.high = np.concatenate([[np.inf, np.inf, c.pitch_reference + c.max_pitch], high])
        self.effort = c.max_effort_ratio * self.model.effortLimit[rear_v]
        # the baselink in the root (fixed: it is on the root body)
        q = pin.neutral(self.model)
        pin.framesForwardKinematics(self.model, self.data, q)
        self.root_to_base = self.data.oMf[self.baselink].copy()
        self.geom = None
        if c.check_collision:
            self._build_collision(urdf, tree)
        self.template = None

    # --- collision (the robot with itself)
    def _build_collision(self, urdf, tree):
        geom = pin.buildGeomFromUrdfString(self.model, urdf, pin.GeometryType.COLLISION)
        thrust_frames = {f.name for f in self.model.frames if f.name.startswith("thrust")}
        prepare_collision_geometry(self.model, geom, thrust_frames, convex_meshes=True)
        adjacent = set()
        for joint in tree.findall("joint"):
            pair = (joint.find("parent").get("link"), joint.find("child").get("link"))
            adjacent |= {pair, pair[::-1]}
        links = [self.model.frames[o.parentFrame].name for o in geom.geometryObjects]
        bodies = [o.parentJoint for o in geom.geometryObjects]
        # only the pairs that the IK moves relative to each other: one of them on an arm
        arm_joints = {self.model.getJointId(n) for n in self.joint_names}

        def on_arm(joint):
            while joint > 0:
                if joint in arm_joints:
                    return True
                joint = self.model.parents[joint]
            return False

        self.pair_links = []
        for i in range(geom.ngeoms):
            for j in range(i + 1, geom.ngeoms):
                if (
                    bodies[i] != bodies[j]
                    and (links[i], links[j]) not in adjacent
                    and (on_arm(bodies[i]) or on_arm(bodies[j]))
                ):
                    geom.addCollisionPair(pin.CollisionPair(i, j))
                    self.pair_links.append((links[i], links[j]))
        self.geom = geom
        self.gdata = pin.GeometryData(geom)
        for request in self.gdata.collisionRequests:
            request.security_margin = self.config.collision_margin

    def collisions(self, p):
        """The link pairs in collision at p (empty without the collision check)."""
        if self.geom is None:
            return set()
        pin.computeCollisions(self.model, self.data, self.geom, self.gdata, self.q(p), False)
        return {self.pair_links[k] for k in range(len(self.pair_links)) if self.gdata.collisionResults[k].isCollision()}

    # --- the plane
    def configuration(self, base_position, base_quaternion, joints):
        """The configuration (pinocchio q) of the baselink pose (world; quaternion [x, y, z, w]) and the
        joint angles {name: angle} (those not given at 0)."""
        q = pin.neutral(self.model)
        for name, angle in joints.items():
            if not self.model.existJointName(name):
                continue
            joint = self.model.joints[self.model.getJointId(name)]
            if joint.nq == 1:
                q[joint.idx_q] = angle
            elif joint.nq == 2:  # continuous: cos, sin
                q[joint.idx_q : joint.idx_q + 2] = [math.cos(angle), math.sin(angle)]
        base = pin.XYZQUATToSE3(np.concatenate([base_position, base_quaternion]))
        q[:7] = pin.SE3ToXYZQUAT(base * self.root_to_base.inverse())
        return q

    def start(self, q):
        """Fix the plane, the hook point and the hook angle at the configuration q (hanging); its
        variables p."""
        root = pin.XYZQUATToSE3(q[:7])
        _, pitch, yaw = pin.rpy.matrixToRpy(root.rotation)
        self.origin = root.translation.copy()
        self.yaw = float(yaw)
        self.heading = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        self.template = q.copy()
        p = np.concatenate([[0.0, self.origin[2], pitch], q[self.idx_q]])
        f = self.features(p)
        self.hook_target, self.hook_angle_target = f.hook.copy(), f.hook_angle
        return p

    def q(self, p):
        q = self.template.copy()
        q[:3] = self.origin + p[0] * self.heading
        q[2] = p[1]
        q[3:7] = pin.Quaternion(pin.rpy.rpyToMatrix(0.0, p[2], self.yaw)).coeffs()
        q[self.idx_q] = p[3:]
        return q

    def to_plane(self, point):
        return np.array([(point - self.origin) @ self.heading, point[2]])

    def from_plane(self, uz):
        point = self.origin + uz[0] * self.heading
        point[2] = uz[1]
        return point

    def plane_angle(self, direction):
        return math.atan2(direction[2], direction @ self.heading)

    def direction(self, angle):
        """The world direction of an angle in the plane."""
        return math.cos(angle) * self.heading + np.array([0.0, 0.0, math.sin(angle)])

    def features(self, p):
        q = self.q(p)
        com = pin.centerOfMass(self.model, self.data, q)  # the kinematics too
        pin.updateFramePlacements(self.model, self.data)
        hook = self.data.oMf[self.hook_frame]
        tip = self.data.oMf[self.tip_frame]
        torque = pin.computeGeneralizedGravity(self.model, self.data, q)[self.rear_v]
        return Features(
            hook=self.to_plane(hook.act(self.hook_offset)),
            hook_angle=self.plane_angle(hook.rotation[:, 0]),
            com=self.to_plane(com),
            tip=self.to_plane(tip.translation),
            tip_angle=self.plane_angle(tip.rotation[:, 0]),
            torque=torque.copy(),
        )

    def jacobian(self, p, f0, eps=1e-6):
        v0 = f0.vector()
        J = np.zeros((len(v0), len(p)))
        for i in range(len(p)):
            dp = p.copy()
            dp[i] += eps
            dv = self.features(dp).vector() - v0
            for k in Features.ANGLES:
                dv[k] = wrap(dv[k])
            J[:, i] = dv / eps
        return J

    def targets(self, p):
        """The CoG [m] and the baselink attitude (quaternion [x, y, z, w]) in the world, and the arm
        joint angles (self.joint_names) of p."""
        q = self.q(p)
        cog = pin.centerOfMass(self.model, self.data, q).copy()
        pin.updateFramePlacements(self.model, self.data)
        rot = pin.Quaternion(self.data.oMf[self.baselink].rotation).coeffs().copy()
        return cog, rot, p[3:].copy()

    # --- the IK
    def solve(self, p, tip, tip_angle=None, step=None, iterations=5):
        """Move p toward the hand at tip (u, z) (and at tip_angle [rad] if given), each variable by at
        most step (array, 9) from p in all (None: no bound), in a number of QP iterations."""
        c = self.config
        p0 = np.asarray(p, float)
        tip = np.asarray(tip, float)

        def linearize(current):
            f = self.features(current)
            J = self.jacobian(current, f)
            rows, rhs = [], []

            def task(weight, jac, error):
                rows.append(weight * np.atleast_2d(jac))
                rhs.append(weight * np.atleast_1d(error))

            task(c.weight_hook_angle, J[2], wrap(self.hook_angle_target - f.hook_angle))
            task(c.weight_com, J[3], self.hook_target[0] - f.com[0])
            tip_error = tip - f.tip
            norm = np.linalg.norm(tip_error)
            if norm > c.max_tip_error:
                tip_error *= c.max_tip_error / norm
            task(c.weight_tip, J[4:6], tip_error)
            if tip_angle is not None:
                task(c.weight_direction, J[6], wrap(tip_angle - f.tip_angle))
            attitude = np.zeros(len(p))
            attitude[2] = 1.0
            task(c.weight_attitude, attitude, c.pitch_reference - current[2])
            task(c.damping, np.eye(len(current)), np.zeros(len(current)))
            A, b = np.vstack(rows), np.concatenate(rhs)

            # Additional inequalities: rear arm torques. IKProblem supplies the bounds and mask.
            Jt = J[7:10]
            C = np.vstack([-Jt, Jt])
            d = np.concatenate([np.minimum(f.torque - self.effort, 0.0), np.minimum(-self.effort - f.torque, 0.0)])
            hook_jac, hook_error = J[0:2], self.hook_target - f.hook
            G = A.T @ A + 1e-9 * np.eye(len(p))
            a = A.T @ b
            return G, a, C, d, hook_jac, -hook_error

        problem = IKProblem(
            None,
            self.low,
            self.high,
            contact_weight=c.weight_hook,
            linearize=linearize,
            collision=self.collisions if self.geom is not None else None,
        )
        p, hook_soft, blocked = problem.incremental(p0, step=step, iterations=iterations)
        f = self.features(p)
        tip_error = float(np.linalg.norm(tip - f.tip))
        angle_error = abs(wrap(tip_angle - f.tip_angle)) if tip_angle is not None else 0.0
        return Result(
            p=p,
            features=f,
            hook_error=float(np.linalg.norm(self.hook_target - f.hook)),
            tip_error=tip_error,
            angle_error=angle_error,
            hook_soft=hook_soft,
            blocked=blocked,
        )
