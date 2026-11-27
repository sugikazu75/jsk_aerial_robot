"""Gimbalrotor-specific thrust and contact allocation for static postures."""

from dataclasses import dataclass, field

import numpy as np
import pinocchio as pin
from aerial_motion_planning.statics import StaticBalanceBase


@dataclass
class StaticsLimits:
    """Limits for the gimballed-rotor statics solver."""

    min_up: float = 0.0  # [N] the least thrust of each rotor along the most upward
    #                                   direction of its gimbal plane
    max_thrust_tilt: float = np.pi / 2  # [rad] the largest angle of the thrust from that direction
    effort: dict = field(default_factory=dict)  # {joint: [Nm]} the largest torque each joint may take


class GimballedRotorStatics(StaticBalanceBase):
    """Static balance of a robot with freely tilting gimballed rotors."""

    def __init__(self, robot, placements, limits=None):
        """limits: StaticsLimits (min_up is raised to the thrust limit of the URDF)."""
        limits = limits or StaticsLimits()
        self.robot = robot
        self.effort_limits = dict(limits.effort)
        self.min_up = max(limits.min_up, robot.thrust_min)
        self.max_tilt = limits.max_thrust_tilt
        super().__init__(robot.model, robot.data, placements, np.array([0.0, 0.0, -9.81 * robot.mass]))

    def solve(
        self,
        x,
        contact_name=None,
        friction=None,
        force_name=None,
        force=None,
        contact_max_normal=None,
        iterations=30,
        warm_start=None,
    ):
        """Solve for an optional frictional contact and optional prescribed world force. The gimbal
        angles are iterated to a fixed point from the upright ones, or from those of warm_start (a
        configuration, e.g. the "q" of the result of the previous frame), which takes fewer
        iterations to the same point when the posture changes little."""
        has_contact = friction is not None
        fixed_force = force
        if has_contact and contact_name not in self.placements:
            raise KeyError(contact_name)
        if force is not None and force_name not in self.placements:
            raise KeyError(force_name)
        robot = self.robot
        if warm_start is None:
            q = robot.q(x)
        else:
            q = robot.raw_q(x)
            for name, _, _, _ in robot.gimbals:
                q[robot.idx_q[name]] = warm_start[robot.idx_q[name]]
        n_rotor = len(robot.gimbals)
        n = 2 * n_rotor + (3 if has_contact else 0)
        for _ in range(iterations):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            com = pin.centerOfMass(self.model, self.data, q, False)
            # wrench balance A u + w = 0, u = [s_1, t_1, ..., s_4, t_4, contact force]: the thrust of
            # rotor i is s_i side_i + t_i up_i, up_i the most upward direction in its gimbal plane
            # (normal to the gimbal axis) and side_i the other one in that plane
            A = np.zeros((6, n))
            w = self._point_force(com, self.gravity).vector.copy()
            gimbal_axes, thrust_axes, planes = [], [], []
            for i, (name, axis, thrust, direction) in enumerate(robot.gimbals):
                placement = self.data.oMf[self.model.getFrameId(thrust)]
                p = placement.translation
                gimbal_axis = self.data.oMi[self.model.getJointId(name)].rotation @ axis
                up = np.array([0.0, 0.0, 1.0]) - gimbal_axis[2] * gimbal_axis
                up /= np.linalg.norm(up)
                side = np.cross(up, gimbal_axis)
                gimbal_axes.append(gimbal_axis)
                thrust_axes.append(placement.rotation[:, 2])
                planes.append((side, up))
                for k, e in enumerate((side, up)):
                    A[:3, 2 * i + k] = e
                    A[3:, 2 * i + k] = np.cross(p, e) + direction * robot.drag_rate * e
            if force is not None:
                w += self._point_force(self._world(force_name).translation, force).vector
            C, b = [], []
            for i in range(n_rotor):  # each rotor pushes up, at least min_up, at most max_tilt off
                row = np.zeros(n)
                row[2 * i + 1] = 1.0
                C.append(row)
                b.append(self.min_up)
                if self.max_tilt < np.pi / 2 - 1e-9:  # (at pi / 2, t >= min_up > 0 is the bound)
                    for sign in (1.0, -1.0):
                        row = np.zeros(n)
                        row[2 * i], row[2 * i + 1] = sign, np.tan(self.max_tilt)
                        C.append(row)
                        b.append(0.0)
            if has_contact:
                contact = self._world(contact_name)
                A[:3, -3:] = np.eye(3)
                A[3:, -3:] = pin.skew(contact.translation)
                frame = contact.rotation  # z: the normal
                for row in (
                    frame[:, 2],
                    friction * frame[:, 2] - frame[:, 0],
                    friction * frame[:, 2] + frame[:, 0],
                    friction * frame[:, 2] - frame[:, 1],
                    friction * frame[:, 2] + frame[:, 1],
                ):
                    full = np.zeros(n)
                    full[-3:] = row
                    C.append(full)
                    b.append(0.0)
                if contact_max_normal is not None:
                    full = np.zeros(n)
                    full[-3:] = -frame[:, 2]
                    C.append(full)
                    b.append(-contact_max_normal)
            if self.effort_limits:
                # the joint torques are tau0 + B u (linear in the forces): |tau_j| <= limit_j
                loads = []  # the load of each element of u, for a unit value
                for i, (_, _, thrust, direction) in enumerate(robot.gimbals):
                    frame_id = self.model.getFrameId(thrust)
                    point = self.data.oMf[frame_id].translation
                    loads += [(frame_id, point, e, direction * robot.drag_rate * e) for e in planes[i]]
                if has_contact:
                    frame_id = self.model.getFrameId(self.placements[contact_name][0])
                    loads += [(frame_id, self._world(contact_name).translation, e, np.zeros(3)) for e in np.eye(3)]
                fixed = []
                if force is not None:
                    fixed.append(
                        (
                            self.model.getFrameId(self.placements[force_name][0]),
                            self._world(force_name).translation,
                            force,
                            np.zeros(3),
                        )
                    )
                tau0 = self._torques(q, fixed)
                B = self._load_torques(q, loads)
                for name, limit in self.effort_limits.items():
                    j = self.model.joints[self.model.getJointId(name)].idx_v
                    C += [-B[j], B[j]]
                    b += [tau0[j] - limit, -limit - tau0[j]]
            G = np.eye(n)
            if has_contact:
                G[-3:, -3:] *= 1e-4  # let the contact carry load where feasible
            u = self._solve_balance(G, np.zeros(n), A, w, np.array(C), np.array(b))
            if u is None:
                return dict(solved=False, q=q, thrust=np.zeros(n_rotor), contact=np.zeros(3), effort={})
            # each gimbal turns its thrust axis onto the direction found (moving the rotor a little),
            # so this is repeated until the angles settle
            delta = np.zeros(n_rotor)
            for i, (name, _, _, _) in enumerate(robot.gimbals):
                side, up = planes[i]
                target = u[2 * i] * side + u[2 * i + 1] * up
                delta[i] = np.arctan2(gimbal_axes[i] @ np.cross(thrust_axes[i], target), thrust_axes[i] @ target)
                q[robot.idx_q[name]] += delta[i]
            if np.max(np.abs(delta)) < 1e-10:
                break
        thrust = np.hypot(u[0 : 2 * n_rotor : 2], u[1 : 2 * n_rotor : 2])
        contact_force = u[-3:] if has_contact else np.zeros(3)

        # joint torques from the RNEA with the thrust, the contact and fixed force as external forces
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        loads = []
        for i, (_, _, name, direction) in enumerate(robot.gimbals):
            frame_id = self.model.getFrameId(name)
            rotor_force = thrust[i] * self.data.oMf[frame_id].rotation[:, 2]
            loads.append(
                (frame_id, self.data.oMf[frame_id].translation, rotor_force, direction * robot.drag_rate * rotor_force)
            )
        if has_contact:
            loads.append(
                (
                    self.model.getFrameId(self.placements[contact_name][0]),
                    self._world(contact_name).translation,
                    contact_force,
                    np.zeros(3),
                )
            )
        if fixed_force is not None:
            loads.append(
                (
                    self.model.getFrameId(self.placements[force_name][0]),
                    self._world(force_name).translation,
                    fixed_force,
                    np.zeros(3),
                )
            )
        tau = self._torques(q, loads)
        residual = float(np.linalg.norm(tau[:6]))  # the floating base should be balanced
        effort = {}
        for name in self.robot.idx_q:
            effort[name] = float(tau[self.model.joints[self.model.getJointId(name)].idx_v])
        over = max([abs(effort[n]) - limit for n, limit in self.effort_limits.items()] + [0.0])
        return dict(
            solved=residual < 1e-6 and over < 1e-6,
            residual=residual,
            q=q,
            thrust=thrust,
            contact=contact_force,
            effort=effort,
        )
