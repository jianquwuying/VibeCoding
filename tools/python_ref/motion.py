"""运动学层：飞镖相机轨迹插值与无人机速度/加速度受限的质心运动。

设计约束（来自需求）：

* 代理必须同时满足 ``|v| <= Vmax`` 与 ``|Δv| <= Amax * dt``；
* 不允许用瞬时速度跳变实现 ``move_towards()``；
* 位置必须通过可配置边界裁剪（机身安全边界，由 ``SimConfig.body_safe_bounds`` 给出）。
"""

from __future__ import annotations

import math
from typing import Dict, Mapping, Optional, Sequence, Tuple, Union

import numpy as np

from config import (
    CAMERA_UP_REF,
    LAMP_MOVE_END_S,
    LAMP_MOVE_START_S,
    LAMP_SPHERE_RADIUS_M,
    TRACKING_OMEGA_RAD_S,
    TRACKING_ZETA,
)
from geometry import normalize

AXES: Tuple[str, str, str] = ("x", "y", "z")
BoundsLike = Mapping[str, Tuple[float, float]]


# ---------------------------------------------------------------------------
# 边界
# ---------------------------------------------------------------------------


def clamp_to_bounds(P: Sequence[float], bounds: BoundsLike) -> np.ndarray:
    """按字典形式的轴对齐边界裁剪位置。"""
    p = np.asarray(P, dtype=float).reshape(-1).copy()
    if p.size != 3:
        raise ValueError(f"位置必须是 3 维，收到 {p.size} 维")
    for i, axis in enumerate(AXES):
        if axis in bounds:
            lo, hi = bounds[axis]
            p[i] = float(min(max(p[i], lo), hi))
    return p








# ---------------------------------------------------------------------------
# 飞镖相机轨迹 C(t)
# ---------------------------------------------------------------------------


def interp_keypoints(t: float, keypoints: Sequence[Tuple[float, Sequence[float]]]) -> np.ndarray:
    """关键点线性插值；区间外按端点保持（clamp）。"""
    if len(keypoints) == 0:
        raise ValueError("keypoints 不能为空")
    times = [float(k[0]) for k in keypoints]
    points = [np.asarray(k[1], dtype=float).reshape(-1) for k in keypoints]
    if len(times) == 1:
        return points[0].copy()
    if t <= times[0]:
        return points[0].copy()
    if t >= times[-1]:
        return points[-1].copy()
    for i in range(len(times) - 1):
        t0, t1 = times[i], times[i + 1]
        if t0 <= t <= t1:
            span = t1 - t0
            w = 0.0 if span <= 0.0 else (t - t0) / span
            return points[i] + w * (points[i + 1] - points[i])
    return points[-1].copy()


# ---------------------------------------------------------------------------
# 速度约束
# ---------------------------------------------------------------------------


def limit_velocity_change(
    v: Sequence[float],
    v_desired: Sequence[float],
    a_max: float,
    dt: float,
) -> np.ndarray:
    """把速度变化量限制在 ``a_max * dt`` 之内（球面半径约束）。"""
    v_now = np.asarray(v, dtype=float).reshape(3)
    dv = np.asarray(v_desired, dtype=float).reshape(3) - v_now
    max_dv = max(0.0, float(a_max) * float(dt))
    norm_dv = float(np.linalg.norm(dv))
    if norm_dv > max_dv and norm_dv > 0.0:
        dv = dv * (max_dv / norm_dv)
    return v_now + dv


def limit_speed(v: Sequence[float], v_max: float) -> np.ndarray:
    """把速度模长限制到不超过 ``v_max``。"""
    vec = np.asarray(v, dtype=float).reshape(3)
    speed = float(np.linalg.norm(vec))
    if speed > v_max > 0.0:
        vec = vec * (float(v_max) / speed)
    return vec


def limit_acceleration(a: Sequence[float], a_max: float) -> np.ndarray:
    """把加速度（或速度增量指令）模长限制到不超过 ``a_max``。"""
    vec = np.asarray(a, dtype=float).reshape(3)
    norm = float(np.linalg.norm(vec))
    if norm > a_max > 0.0:
        vec = vec * (float(a_max) / norm)
    return vec


def tracking_gains(
    omega: Optional[float] = None,
    zeta: Optional[float] = None,
) -> Tuple[float, float]:
    """由自然频率与阻尼比给出 PD 增益 ``(Kp, Kd) = (omega^2, 2 * zeta * omega)``。"""
    omega = TRACKING_OMEGA_RAD_S if omega is None else float(omega)
    zeta = TRACKING_ZETA if zeta is None else float(zeta)
    if omega <= 0.0 or zeta <= 0.0:
        raise ValueError("omega / zeta 必须为正")
    return omega ** 2, 2.0 * zeta * omega


def update_uav_state(
    P: Sequence[float],
    v: Sequence[float],
    target: Optional[Sequence[float]],
    v_max: float,
    a_max: float,
    dt: float,
    bounds: BoundsLike,
    target_velocity: Optional[Sequence[float]] = None,
    kp: Optional[float] = None,
    kd: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """单步更新无人机位置与速度（满足 Vmax / Amax 约束）。

    采用**加速度受限的 PD 律**（ENGINEERING_ASSUMPTION）::

        e      = target - P                       # 位置误差
        ev     = target_velocity - v              # 速度误差（前馈补偿移动目标）
        a_cmd  = Kp * e + Kd * ev                 # 期望加速度
        a_cmd  = clamp(|a_cmd| <= a_max)          # 满足 Amax
        v_next = v + a_cmd * dt                   # => |Δv| <= a_max * dt（构造保证）
        v_next = clamp(|v_next| <= v_max)         # 满足 Vmax
        P_next = clamp_to_bounds(P + v_next * dt) # 机身安全边界

    ``Kp = omega^2``、``Kd = 2 * zeta * omega``，默认 ``omega = 2.0 rad/s``、
    ``zeta = 1.0``（临界阻尼，避免过冲后长时间偏离视线）。
    前馈项 ``target_velocity`` 用于跟踪移动视线上的点：缺少它会出现与目标速度
    成正比的稳态滞后。``target_velocity=None`` 时按静止目标处理。

    ``kp`` / ``kd`` 可显式覆盖默认增益。

    数值稳定性：以步长 ``dt`` 离散化时建议 ``kd * dt <= 1.5`` 且 ``kp * dt^2 <= 1``；
    默认参数（omega = 5.0 rad/s、zeta = 1.0、dt = 0.02 s）满足该条件。

    返回 ``(P_next, v_next)``。
    """
    p = np.asarray(P, dtype=float).reshape(3)
    v_now = np.asarray(v, dtype=float).reshape(3)

    kp_def, kd_def = tracking_gains()
    kp_val = float(kp_def if kp is None else kp)
    kd_val = float(kd_def if kd is None else kd)

    e = np.zeros(3) if target is None else np.asarray(target, dtype=float).reshape(3) - p
    v_ff = np.zeros(3)
    if target_velocity is not None:
        v_ff = limit_speed(np.asarray(target_velocity, dtype=float).reshape(3), v_max)

    a_cmd = limit_acceleration(kp_val * e + kd_val * (v_ff - v_now), a_max)
    v_next = limit_velocity_change(v_now, v_now + a_cmd * dt, a_max, dt)
    v_next = limit_speed(v_next, v_max)
    p_next = clamp_to_bounds(p + v_next * dt, bounds)
    return p_next, v_next


# ---------------------------------------------------------------------------
# M2 模型（v2）：灯横向平移 / J2 锁定判据 / 六步积分 / 线段-点距离 / 无人机一维规划
# ---------------------------------------------------------------------------


def lamp_y_at(t: float, y_target: float) -> float:
    """t 时刻灯的 Y 坐标 [m]。分段线性：t<1.2 → 0；1.2≤t<1.8 → 匀速；t≥1.8 → y_target。"""
    if t < LAMP_MOVE_START_S:
        return 0.0
    if t >= LAMP_MOVE_END_S:
        return float(y_target)
    return float(y_target) * (t - LAMP_MOVE_START_S) / (LAMP_MOVE_END_S - LAMP_MOVE_START_S)


def lamp_vy_at(t: float, y_target: float) -> float:
    """t 时刻灯的 Y 方向速度 [m/s]（移动区间内为常数，其余为 0）。"""
    if t < LAMP_MOVE_START_S or t >= LAMP_MOVE_END_S:
        return 0.0
    return float(y_target) / (LAMP_MOVE_END_S - LAMP_MOVE_START_S)


def lamp_center_at(t: float, y_target: float) -> np.ndarray:
    """t 时刻灯心世界坐标 = (0, y(t), 0)。"""
    return np.array([0.0, lamp_y_at(t, y_target), 0.0])


def is_fully_visible(
    P: Sequence[float],
    d: Sequence[float],
    L: Sequence[float],
    camera_cfg,
    up_ref: Sequence[float] = CAMERA_UP_REF,
) -> bool:
    """J2 锁定判据：整颗 55 mm 灯球是否完整落在像面内。

    球投影半径用 ``r_px = f * r / c.z``（小孔近似，与 TS 端同式）。
    """
    from camera import pose_from_position_and_axis  # 延迟导入，避免与 camera 循环依赖

    pose = pose_from_position_and_axis(P, d, up_ref)
    c = pose.to_camera_frame(L)
    if c[2] <= 1e-9:
        return False
    width, height = int(camera_cfg.resolution[0]), int(camera_cfg.resolution[1])
    f = float(camera_cfg.focal_px)
    u0 = width / 2.0 + f * c[0] / c[2]
    v0 = height / 2.0 - f * c[1] / c[2]
    r_px = f * LAMP_SPHERE_RADIUS_M / c[2]
    return bool(
        (u0 - r_px >= 0.0)
        and (u0 + r_px <= width)
        and (v0 - r_px >= 0.0)
        and (v0 + r_px <= height)
    )


def step_dart_m2(
    P: Sequence[float],
    V: Sequence[float],
    L: Sequence[float],
    dt: float,
    k: float,
    g: float,
    omega_max_rad: float,
    camera_cfg,
    up_ref: Sequence[float] = CAMERA_UP_REF,
) -> Tuple[np.ndarray, np.ndarray, bool]:
    """M2 六步积分一步。返回 ``(P_next, V_next, locked)``。"""
    p = np.asarray(P, dtype=float)
    v = np.asarray(V, dtype=float)
    lamp = np.asarray(L, dtype=float)

    # 1) 头部朝向 = 速度方向（速度退化时用 (1,0,0) 兜底）
    v_norm = float(np.linalg.norm(v))
    d = v / v_norm if v_norm >= 1e-12 else np.array([1.0, 0.0, 0.0])

    # 2) 锁定判定（每帧独立）
    locked = is_fully_visible(p, d, lamp, camera_cfg, up_ref)

    # 3) 重力 + 线性阻力
    a_phys = np.array([0.0, 0.0, -float(g)]) - float(k) * v

    # 4) 无约束速度
    v_phys = v + a_phys * dt

    # 5) 追踪段：保持速率、方向以 ω_max·dt 为步长转向灯心
    if locked:
        v_mag = float(np.linalg.norm(v_phys))
        d_aim = normalize(lamp - p)
        d_theta_max = float(omega_max_rad) * dt
        cos_theta = float(np.clip(float(np.dot(d, d_aim)), -1.0, 1.0))
        theta = math.acos(cos_theta)
        if theta <= d_theta_max:
            d_new = d_aim
        else:
            axis = np.cross(d, d_aim)
            axis_norm = float(np.linalg.norm(axis))
            if axis_norm < 1e-12:
                d_new = d
            else:
                k_hat = axis / axis_norm
                d_new = d * math.cos(d_theta_max) + np.cross(k_hat, d) * math.sin(d_theta_max)
        v_new = v_mag * d_new
    else:
        v_new = v_phys

    # 6) 位置积分
    p_next = p + v_new * dt
    return p_next, v_new, bool(locked)


def segment_to_point_distance(
    A: Sequence[float], B: Sequence[float], Q: Sequence[float]
) -> float:
    """线段 AB 到点 Q 的最近距离 [m]（退化线段直接取 |Q-A|）。"""
    a = np.asarray(A, dtype=float)
    b = np.asarray(B, dtype=float)
    q = np.asarray(Q, dtype=float)
    d = b - a
    ddot = float(np.dot(d, d))
    if ddot < 1e-24:
        return float(np.linalg.norm(q - a))
    t_param = float(np.dot(q - a, d)) / ddot
    t_param = min(max(t_param, 0.0), 1.0)
    closest = a + t_param * d
    return float(np.linalg.norm(closest - q))


def uav_step(
    y0: float,
    v0: float,
    y1: float,
    v_max: float,
    a_max: float,
    dt: float,
) -> Tuple[float, float]:
    """一维带初速度的时间最优规划：从 y0（初速度 v0）到 y1（末速度 0），步进 dt。

    ⚠️ 本函数的分支顺序与表达式顺序必须与 TS 端 ``uavStep`` 逐行一致，
    否则跨语言 parity（≤1e-12）无法对齐。
    """
    d = float(y1) - float(y0)
    sgn = 1.0 if d > 0.0 else (-1.0 if d < 0.0 else 0.0)
    xt = abs(d)

    # 已在目标位置
    if xt < 1e-12:
        if abs(float(v0)) <= a_max * dt:
            return float(y1), 0.0
        s_v = 1.0 if v0 > 0.0 else -1.0
        v_new = float(v0) - s_v * a_max * dt
        if v_new * s_v < 0.0:
            v_new = 0.0
        y_new = float(y0) + float(v0) * dt - 0.5 * s_v * a_max * dt * dt
        lo, hi = (float(y0), float(y1)) if y0 <= y1 else (float(y1), float(y0))
        y_new = min(max(y_new, lo), hi)
        return y_new, v_new

    v = sgn * float(v0)
    x = 0.0

    # 正在远离目标：先满减速到 0
    if v < 0.0:
        t_stop = -v / a_max
        if t_stop >= dt:
            x_step = v * dt + 0.5 * a_max * dt * dt
            v_new = v + a_max * dt
            return float(y0) + sgn * x_step, sgn * v_new
        x += v * t_stop + 0.5 * a_max * t_stop * t_stop
        t_rem = dt - t_stop
        v = 0.0
    else:
        t_rem = dt

    # 目标峰值速度（三角解，梯形时封顶 v_max）
    vp2 = a_max * (xt - x) + v * v / 2.0
    vp = min(math.sqrt(max(vp2, 0.0)), v_max)
    t_a = (vp - v) / a_max
    x_a = (vp * vp - v * v) / (2.0 * a_max)

    if t_rem <= t_a:  # 仍在加速段
        x += v * t_rem + 0.5 * a_max * t_rem * t_rem
        v += a_max * t_rem
        return float(y0) + sgn * x, sgn * v

    t_rem -= t_a
    x += x_a
    v = vp

    denom = max(vp, 1e-12)
    x_c = max(0.0, (xt - x - vp * vp / (2.0 * a_max)) / denom)  # 匀速段距离
    if t_rem <= x_c / denom:
        x += vp * t_rem
        return float(y0) + sgn * x, sgn * vp

    t_rem -= x_c / denom
    x += x_c

    # 减速段：解 0.5·a·t² − vp·t + 剩余距离 = 0
    x_rem = xt - x
    disc = vp * vp - 2.0 * a_max * x_rem
    if disc < 0.0:
        disc = 0.0
    t3 = (vp - math.sqrt(disc)) / a_max
    if t3 > t_rem:
        t3 = t_rem
    x += vp * t3 - 0.5 * a_max * t3 * t3
    v = vp - a_max * t3
    if v < 0.0:
        v = 0.0
    if x > xt:
        x = xt
        v = 0.0
    return float(y0) + sgn * x, sgn * v




