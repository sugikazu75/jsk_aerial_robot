"""The handful of quaternion helpers this stack used from tf.transformations.

ROS2 has no `tf` package, and `tf_transformations` is a separate pip package
that is not part of a desktop install. Only four functions were ever used, so
they are written out here rather than taking a dependency.

Quaternions are (x, y, z, w) numpy arrays throughout, which is tf1's order and
the order the messages store - *not* Eigen's, which puts w first. Getting that
backwards produces a robot that flies subtly wrong rather than an error, which
is why these are checked against tf1 by test_transform_utils.py.
"""

import math

import numpy as np


def quaternion_from_euler(roll, pitch, yaw):
    """tf1's quaternion_from_euler with its default 'sxyz' axes."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)

    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def euler_from_quaternion(q):
    """tf1's euler_from_quaternion with its default 'sxyz' axes."""
    x, y, z, w = q[0], q[1], q[2], q[3]

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Clamped: a quaternion that is a hair off unit length otherwise makes asin
    # return NaN, and a NaN attitude propagates silently.
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])


def quaternion_multiply(q1, q2):
    """q1 * q2, both (x, y, z, w)."""
    x1, y1, z1, w1 = q1[0], q1[1], q1[2], q1[3]
    x2, y2, z2, w2 = q2[0], q2[1], q2[2], q2[3]

    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quaternion_inverse(q):
    """The inverse of a quaternion, as tf1 computed it - conjugate over norm."""
    q = np.asarray(q, dtype=float)
    return np.array([-q[0], -q[1], -q[2], q[3]]) / np.dot(q, q)


def numpify_vector(msg):
    """A geometry_msgs Point/Vector3 as a numpy array; ros_numpy is ROS1-only."""
    return np.array([msg.x, msg.y, msg.z])


def numpify_quaternion(msg):
    """A geometry_msgs Quaternion as (x, y, z, w)."""
    return np.array([msg.x, msg.y, msg.z, msg.w])
