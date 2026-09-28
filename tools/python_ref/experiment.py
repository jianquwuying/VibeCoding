"""M2 三体联动仿真（v2）：灯横向平移 + 飞镖六步制导 + 无人机实时追踪。

本模块是 Web 端 ``src/core/simulate.ts`` 的 Python 真值来源（vendored copy；
原项目 ``../rm_uav_occlusion`` 不再同步）。只保留 M2 需要的部分：

* :func:`hover_station_position_at` —— 灯位于某个 Y 时的无人机悬停站位
* :func:`run_m2`                    —— 主仿真循环（六步积分 + J2 锁定 + 命中截断）
* :func:`occlusion_margin`          —— 灯心到阴影多边形最近边距离 − 灯球半径
* :func:`compute_metrics`           —— 遮挡时间统计

v1 的静态网格扫描 / 站位扫描 / 相机统计等分析函数未随本副本迁移。
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence

import numpy as np
from shapely.geometry.base import BaseGeometry

from config import (
    CAMERA_UP_REF,
    GRAVITY_MPS2,
    HIT_DISTANCE_M,
    LAMP_MOVE_START_S,
    LAMP_SPHERE_RADIUS_M,
    MAX_TURN_RATE_DPS,
    SIM_T_END_S,
    CameraConfig,
    DartConfig,
    HoverStationConfig,
    LampConfig,
    SimConfig,
)
from camera import fov_margin_deg, image_space_occlusion, pose_from_position_and_axis
from geometry import all_vertices_between_camera_and_plane, plane_basis, uav_corners
from motion import (
    is_fully_visible,
    lamp_center_at,
    lamp_y_at,
    segment_to_point_distance,
    step_dart_m2,
    uav_step,
)
from occlusion import compute_occlusion, is_effective, lamp_disc_polygon


def hover_station_position_at(
    lamp: LampConfig,
    standoff_m: float,
    offset_u_m: float,
    offset_v_m: float,
    lamp_y: float,
    uav_size_m: Sequence[float],
    sim_cfg: Optional[SimConfig] = None,
) -> np.ndarray:
    """灯位于 ``(0, lamp_y, 0)`` 时的无人机悬停站位（并裁剪到机身安全边界）。

    ``站位 = L + standoff·n̂ + offset_u·û + offset_v·v̂``；对本场景
    （n̂=(-1,0,0)、û=(0,-1,0)、v̂=(0,0,1)）等价于 ``(-standoff, lamp_y − offset_u, offset_v)``。
    """
    center = np.asarray(lamp.center_array, dtype=float) + np.array([0.0, float(lamp_y), 0.0])
    n_hat = np.asarray(lamp.normal_array, dtype=float)
    u_hat, v_hat = plane_basis(n_hat)
    p = (
        center
        + float(standoff_m) * n_hat
        + float(offset_u_m) * u_hat
        + float(offset_v_m) * v_hat
    )
    if sim_cfg is not None:
        bounds = sim_cfg.body_safe_bounds(uav_size_m)
        for i, axis in enumerate(("x", "y", "z")):
            lo, hi = bounds[axis]
            p[i] = float(min(max(p[i], lo), hi))
    return p


def occlusion_margin(shadow_polygon: BaseGeometry, lamp_radius_m: float) -> float:
    """遮挡余量 [m] = 灯心（平面局部原点）到阴影多边形最近边的距离 − 灯球半径。

    ``>= 0`` 表示灯球被阴影完全覆盖（此时 ``R_occ`` 取满值 0.999994）；
    ``< 0`` 表示灯球外露，暴露量为 ``|margin|``。
    """
    if shadow_polygon is None or shadow_polygon.is_empty:
        return float("-inf")
    ring = list(shadow_polygon.exterior.coords)
    best = float("inf")
    for a, b in zip(ring[:-1], ring[1:]):
        ex, ey = b[0] - a[0], b[1] - a[1]
        seg_len = math.hypot(ex, ey)
        if seg_len < 1e-15:
            continue
        # 原点到直线 ab 的距离（平面局部坐标下灯心即原点）
        dist = abs(ex * (0.0 - a[1]) - ey * (0.0 - a[0])) / seg_len
        if dist < best:
            best = dist
    if best == float("inf"):
        return float("-inf")
    return float(best - float(lamp_radius_m))


def run_m2(
    dart: DartConfig,
    lamp: LampConfig,
    camera: CameraConfig,
    sim: SimConfig,
    station: HoverStationConfig,
    uav_size_m: Sequence[float],
    uav_max_speed: float = 5.0,
    uav_max_accel: float = 2.0,
    omega_max_dps: float = MAX_TURN_RATE_DPS,
    g: float = GRAVITY_MPS2,
    dt: Optional[float] = None,
    lamp_target_y_m: Optional[float] = None,
    t_end_s: float = SIM_T_END_S,
    min_frames: int = 2,
    lamp_quad_segs: int = 256,
    warn: bool = False,
) -> Dict[str, object]:
    """M2 主仿真循环。

    返回字典键：``t / Pd / Vd / R / imageR / fovMarginDeg / uavPos / lampPos /
    lampY / uavY / uavVy / locked / valid / hitLamp / tHit / tLock / n /
    marginMin / marginMinT``。``R`` 中无效帧为 ``nan``。
    """
    step = float(sim.dt if dt is None else dt)
    y_target = float(sim.lamp_target_y_m if lamp_target_y_m is None else lamp_target_y_m)
    omega_max_rad = math.radians(float(omega_max_dps))
    r_lamp = float(lamp.radius_m)
    lamp_poly = lamp_disc_polygon(lamp, quad_segs=int(lamp_quad_segs))
    area_lamp = float(lamp.area_m2)

    n_full = int(round(float(t_end_s) / step)) + 1
    uav_size = np.asarray(uav_size_m, dtype=float)

    # --- 初始状态 ---------------------------------------------------------
    p = np.asarray(dart.launch, dtype=float)
    l0 = lamp_center_at(0.0, y_target)
    h_hat = np.array([l0[0] - p[0], l0[1] - p[1], 0.0])
    h_hat = h_hat / float(np.linalg.norm(h_hat))
    theta0 = math.radians(float(dart.theta0_deg))
    v = float(dart.v0_mps) * (
        math.cos(theta0) * h_hat + math.sin(theta0) * np.array([0.0, 0.0, 1.0])
    )

    uav_y = -float(station.offset_u_m)
    uav_v = 0.0

    t_arr: List[float] = []
    pd: List[float] = []
    vd: List[float] = []
    r_arr: List[float] = []
    image_r: List[float] = []
    fov_margin: List[float] = []
    uav_pos: List[float] = []
    lamp_pos: List[float] = []
    lamp_y_arr: List[float] = []
    uav_y_arr: List[float] = []
    uav_vy_arr: List[float] = []
    locked_arr: List[int] = []
    valid_arr: List[int] = []

    hit_index = -1
    t_lock = -1.0
    margin_min = float("inf")
    margin_min_t = -1.0

    for i in range(n_full):
        t_i = i * step
        lamp_center = lamp_center_at(t_i, y_target)

        # --- 无人机：t>=1.2 s 起追踪灯的当前位置 ---------------------------
        if t_i >= LAMP_MOVE_START_S and i > 0:
            target_y = lamp_y_at(t_i, y_target) - float(station.offset_u_m)
            uav_y, uav_v = uav_step(
                uav_y, uav_v, target_y, float(uav_max_speed), float(uav_max_accel), step
            )
        uav_center = hover_station_position_at(
            lamp, station.standoff_m, station.offset_u_m, station.offset_v_m,
            uav_y + float(station.offset_u_m), uav_size, sim,
        )

        # --- 记录帧 i -------------------------------------------------------
        los_hat = lamp_center - p
        los_norm = float(np.linalg.norm(los_hat))
        los_hat = los_hat / los_norm if los_norm > 1e-12 else np.array([1.0, 0.0, 0.0])

        t_arr.append(t_i)
        pd.extend([float(p[0]), float(p[1]), float(p[2])])
        vd.extend([float(v[0]), float(v[1]), float(v[2])])
        lamp_pos.extend([float(lamp_center[0]), float(lamp_center[1]), float(lamp_center[2])])
        lamp_y_arr.append(float(lamp_center[1]))
        uav_pos.extend([float(uav_center[0]), float(uav_center[1]), float(uav_center[2])])
        uav_y_arr.append(float(uav_y))
        uav_vy_arr.append(float(uav_v))

        # --- 遮挡与相机指标 -------------------------------------------------
        res = compute_occlusion(
            p, uav_center, uav_size, lamp_center, los_hat, lamp_poly, area_lamp, warn=warn
        )
        # valid 判据（§2.6）：与 project_uav_shadow 内部前提**完全一致**——
        # 无人机 8 个顶点必须整体位于飞镖相机与灯平面之间；否则该帧几何无定义。
        valid = all_vertices_between_camera_and_plane(
            p, uav_corners(uav_center, uav_size), lamp_center, los_hat
        )
        valid_arr.append(1 if valid else 0)
        r_arr.append(float(res.R_occ) if valid else float("nan"))

        if res.shadow_polygon is not None:
            m = occlusion_margin(res.shadow_polygon, r_lamp)
            if m < margin_min:
                margin_min = m
                margin_min_t = t_i

        v_norm = float(np.linalg.norm(v))
        d_hat = v / v_norm if v_norm > 1e-12 else np.array([1.0, 0.0, 0.0])
        pose = pose_from_position_and_axis(p, d_hat, CAMERA_UP_REF)
        fov_margin.append(float(fov_margin_deg(pose, lamp_center, camera)))
        ratio, _, _ = image_space_occlusion(
            pose, lamp, los_hat, uav_center, uav_size, camera, 128
        )
        image_r.append(float(ratio))

        # --- 飞镖推进 + 命中判定 --------------------------------------------
        if i < n_full - 1:
            p_next, v_next, locked = step_dart_m2(
                p, v, lamp_center, step, float(dart.speed_decay_per_s),
                float(g), omega_max_rad, camera, CAMERA_UP_REF,
            )
            locked_arr.append(1 if locked else 0)
            if locked and t_lock < 0.0:
                t_lock = t_i
            d_min = segment_to_point_distance(p, p_next, lamp_center)
            if hit_index < 0 and d_min <= HIT_DISTANCE_M:
                hit_index = i
            p, v = p_next, v_next
        else:
            locked_arr.append(
                1 if is_fully_visible(p, d_hat, lamp_center, camera, CAMERA_UP_REF) else 0
            )

    n = max(int(min_frames), hit_index) if hit_index >= 0 else n_full
    hit_lamp = hit_index >= 0
    t_hit: Optional[float] = float(hit_index * step) if hit_lamp else None

    def cut(values: List[float], width: int = 1) -> np.ndarray:
        return np.asarray(values[: n * width], dtype=float)

    return {
        "n": int(n),
        "dt": step,
        "t": cut(t_arr),
        "Pd": cut(pd, 3),
        "Vd": cut(vd, 3),
        "R": cut(r_arr),
        "imageR": cut(image_r),
        "fovMarginDeg": cut(fov_margin),
        "uavPos": cut(uav_pos, 3),
        "lampPos": cut(lamp_pos, 3),
        "lampY": cut(lamp_y_arr),
        "uavY": cut(uav_y_arr),
        "uavVy": cut(uav_vy_arr),
        "locked": np.asarray(locked_arr[:n], dtype=int),
        "valid": np.asarray(valid_arr[:n], dtype=int),
        "hitLamp": bool(hit_lamp),
        "tHit": t_hit,
        "tLock": float(t_lock),
        "marginMin": float(margin_min),
        "marginMinT": float(margin_min_t),
    }


def _longest_true_run(mask: np.ndarray) -> int:
    best = 0
    cur = 0
    for flag in mask:
        if flag:
            cur += 1
            best = max(best, cur)
        else:
            cur = 0
    return int(best)


def compute_metrics(
    t: Sequence[float],
    R: Sequence[float],
    R_th: float = 0.7,
) -> Dict[str, float]:
    """遮挡时间统计：``T_occ`` = sum(R >= R_th) · dt（NaN 帧不计入有效遮挡）。"""
    t_arr = np.asarray(t, dtype=float).reshape(-1)
    R_arr = np.asarray(R, dtype=float).reshape(-1)
    if t_arr.size != R_arr.size:
        raise ValueError("t 与 R 长度必须一致")
    if t_arr.size == 0:
        raise ValueError("t / R 不能为空")
    dt = float(np.mean(np.diff(t_arr))) if t_arr.size > 1 else float("nan")
    eff = np.isfinite(R_arr) & (R_arr >= float(R_th))
    T_occ = float(np.sum(eff) * dt) if t_arr.size > 1 else 0.0
    T_cont_max = float(_longest_true_run(eff) * dt) if t_arr.size > 1 else 0.0
    first_idx = int(np.argmax(eff)) if bool(np.any(eff)) else -1
    first_t = float(t_arr[first_idx]) if first_idx >= 0 else float("nan")
    finite = R_arr[np.isfinite(R_arr)]
    return {
        "T_occ": T_occ,
        "T_cont_max": T_cont_max,
        "first_occlusion_t": first_t,
        "R_max": float(np.max(finite)) if finite.size else float("nan"),
        "R_mean": float(np.mean(finite)) if finite.size else float("nan"),
        "R_min": float(np.min(finite)) if finite.size else float("nan"),
    }
