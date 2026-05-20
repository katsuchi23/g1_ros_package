#!/usr/bin/env python3

"""Minimal tf_transformations-compatible helpers used by this package.

This module is used only when `tf_transformations` is unavailable.
It implements the small subset of APIs required by FAST_LIO_LOCALIZATION2.
"""

from __future__ import annotations

import math
import numpy as np


def quaternion_matrix(quaternion):
    x, y, z, w = quaternion
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(4, dtype=float)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s

    m = np.eye(4, dtype=float)
    m[0, 0] = 1.0 - (yy + zz)
    m[0, 1] = xy - wz
    m[0, 2] = xz + wy
    m[1, 0] = xy + wz
    m[1, 1] = 1.0 - (xx + zz)
    m[1, 2] = yz - wx
    m[2, 0] = xz - wy
    m[2, 1] = yz + wx
    m[2, 2] = 1.0 - (xx + yy)
    return m


def quaternion_from_matrix(matrix):
    m = np.array(matrix, dtype=float, copy=False)[:4, :4]
    q = np.empty((4,), dtype=float)
    t = np.trace(m[:3, :3])

    if t > 0.0:
        t = math.sqrt(t + 1.0)
        q[3] = 0.5 * t
        t = 0.5 / t
        q[0] = (m[2, 1] - m[1, 2]) * t
        q[1] = (m[0, 2] - m[2, 0]) * t
        q[2] = (m[1, 0] - m[0, 1]) * t
    else:
        i = 0
        if m[1, 1] > m[0, 0]:
            i = 1
        if m[2, 2] > m[i, i]:
            i = 2
        j = (i + 1) % 3
        k = (j + 1) % 3
        t = math.sqrt(m[i, i] - m[j, j] - m[k, k] + 1.0)
        q[i] = 0.5 * t
        t = 0.5 / t
        q[3] = (m[k, j] - m[j, k]) * t
        q[j] = (m[j, i] + m[i, j]) * t
        q[k] = (m[k, i] + m[i, k]) * t

    return q


def translation_from_matrix(matrix):
    m = np.array(matrix, dtype=float, copy=False)
    return m[:3, 3].copy()


def euler_matrix(ai, aj, ak):
    """Return rotation matrix for roll(ai), pitch(aj), yaw(ak), sxyz convention."""
    si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
    ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)

    m = np.eye(4, dtype=float)
    m[0, 0] = cj * ck
    m[0, 1] = sj * si * ck - ci * sk
    m[0, 2] = sj * ci * ck + si * sk
    m[1, 0] = cj * sk
    m[1, 1] = sj * si * sk + ci * ck
    m[1, 2] = sj * ci * sk - si * ck
    m[2, 0] = -sj
    m[2, 1] = cj * si
    m[2, 2] = cj * ci
    return m


def euler_from_matrix(matrix):
    """Return roll, pitch, yaw from homogeneous transform, sxyz convention."""
    m = np.array(matrix, dtype=float, copy=False)
    r = m[:3, :3]

    sy = -r[2, 0]
    sy_clamped = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy_clamped)
    cy = math.cos(pitch)

    if abs(cy) > 1e-8:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:
        # Gimbal lock fallback
        roll = math.atan2(-r[1, 2], r[1, 1])
        yaw = 0.0

    return roll, pitch, yaw


def quaternion_from_euler(ai, aj, ak):
    return quaternion_from_matrix(euler_matrix(ai, aj, ak))
