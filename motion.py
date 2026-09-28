"""运动学层：飞镖相机轨迹插值与无人机速度/加速度受限的质心运动。

设计约束（来自需求）：

* 代理必须同时满足 ``|v| <= Vmax`` 与 ``|Δv| <= Amax * dt``；
* 不允许用瞬时速度跳变实现 ``move_towards()``；
* 位置必须通过可配置边界裁剪（机身安全边界，由 ``SimConfig.body_safe_bounds`` 给出）。
"""

from __future__ import annotations

from typing import Dict, Mapping, Optional, Sequence, Tuple, Union

import numpy as np

from config import (
    DART_MODE_PARABOLIC,
    DART_MODE_SIMPLE,
    TRACKING_OMEGA_RAD_S,
    TRACKING_ZETA,
    DartConfig,
)

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


def dart_camera_trajectory(t: Union[float, Sequence[float]], dart_cfg: DartConfig) -> np.ndarray:
    """返回 ``C(t)``；``t`` 为标量时返回 (3,)，为序列时返回 (N, 3)。"""
    if dart_cfg.mode == DART_MODE_PARABOLIC:
        # 抛物线用解析式求值，避免分段线性近似。
        if np.isscalar(t):
            return dart_cfg.parabolic_position(float(t))
        times = np.asarray(t, dtype=float).reshape(-1)
        return np.asarray([dart_cfg.parabolic_position(float(ti)) for ti in times], dtype=float)


    if dart_cfg.mode == DART_MODE_SIMPLE:
        # 最简抛物线：发射点 + 落点 + v0 + 仰角 + 减速率
        if np.isscalar(t):
            return dart_cfg.simple_position(float(t))
        times = np.asarray(t, dtype=float).reshape(-1)
        return np.asarray([dart_cfg.simple_position(float(ti)) for ti in times], dtype=float)

    keypoints = dart_cfg.keypoints
    if np.isscalar(t):
        return interp_keypoints(float(t), keypoints)
    times = np.asarray(t, dtype=float).reshape(-1)
    return np.asarray([interp_keypoints(float(ti), keypoints) for ti in times], dtype=float)


def dart_camera_velocity(
    t: float,
    dart_cfg: DartConfig,
    dt: float = 1e-3,
) -> np.ndarray:
    """相机速度 ``dC/dt``（中心差分；首尾用单侧差分）。

    轨迹是分段线性插值，除关键点处外速度分段恒定，因此中心差分是精确的
    （关键点处给出单侧平均，属可接受近似）。
    """
    if dt <= 0.0:
        raise ValueError("dt 必须为正")
    if dart_cfg.mode == DART_MODE_PARABOLIC:
        return dart_cfg.parabolic_velocity(float(t))
    if dart_cfg.mode == DART_MODE_SIMPLE:
        return dart_cfg.simple_velocity(float(t))
    t0 = float(dart_cfg.t_start)
    t1 = float(dart_cfg.t_end)
    if t - dt >= t0 and t + dt <= t1:
        c_prev = dart_camera_trajectory(t - dt, dart_cfg)
        c_next = dart_camera_trajectory(t + dt, dart_cfg)
        return (c_next - c_prev) / (2.0 * dt)
    if t + dt <= t1:
        return (dart_camera_trajectory(t + dt, dart_cfg) - dart_camera_trajectory(t, dart_cfg)) / dt
    if t - dt >= t0:
        return (dart_camera_trajectory(t, dart_cfg) - dart_camera_trajectory(t - dt, dart_cfg)) / dt
    return np.zeros(3)


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




