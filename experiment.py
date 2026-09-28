"""实验层：静态网格扫描、动态仿真、悬停站位距离扫描与遮挡指标计算。"""

from __future__ import annotations

import warnings
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple, Union

import numpy as np
import pandas as pd

from config import (
    DEFAULT_HOVER_STATION,
    HOVER_STATION_OFFSET_V_M,
    INITIAL_NEAR_LAMP,
    DartConfig,
    HoverStationConfig,
    LampConfig,
    SimConfig,
    UAVConfig,
)
from geometry import as_vector
from motion import clamp_to_bounds, dart_camera_trajectory, update_uav_state
from occlusion import compute_occlusion, is_effective, lamp_disc_polygon
from strategies import (
    HoverStationPolicy,
    TargetPolicy,
    make_policy,
)


# ---------------------------------------------------------------------------
# 初始工况
# ---------------------------------------------------------------------------


def initial_uav_position(
    condition: str,
    dart_cfg: DartConfig,
    lamp: LampConfig,
    sim_cfg: SimConfig,
    uav_cfg: UAVConfig,
    station: HoverStationConfig = DEFAULT_HOVER_STATION,
) -> np.ndarray:
    """按工况名生成初始无人机位置（并裁剪到机身安全边界内）。

    * ``initial_hover_near_lamp``：**悬停站位**（距灯特定位置，当前主场景）。
    * ``initial_on_LOS`` ：位于 ``t = 0`` 的相机 -> 灯视线点上。
    * ``initial_off_LOS``：在该视线点基础上叠加 ``OFF_LOS_OFFSET_M`` 偏移。
    """
    if condition != INITIAL_NEAR_LAMP:
        raise ValueError(f"当前只支持 {INITIAL_NEAR_LAMP}，收到 {condition!r}")
    # 悬停站位：由“距灯的距离 + 灯平面内横向偏置”给出，与视线无关。
    p = station.position(lamp)

    bounds = sim_cfg.body_safe_bounds(uav_cfg.size_m)
    p_clamped = clamp_to_bounds(p, bounds)
    if not np.allclose(p_clamped, p):
        warnings.warn(
            f"初始位置 {np.round(p, 3).tolist()} 超出机身安全边界，已裁剪为 "
            f"{np.round(p_clamped, 3).tolist()}",
            stacklevel=2,
        )
    return p_clamped


# ---------------------------------------------------------------------------
# 静态网格扫描
# ---------------------------------------------------------------------------


def static_grid_scan_xz(
    cam_pos: Sequence[float],
    uav_size: Sequence[float],
    lamp: LampConfig,
    y: float = -2.5,
    x_range: Tuple[float, float, int] = (-6.0, 0.2, 41),
    z_range: Tuple[float, float, int] = (-1.5, 2.0, 31),
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """在 ``y = 常数`` 的 XZ 切片上扫描遮挡率。

    （本场景弹道沿 +X 前进、灯平面为 ``X = 0``，因此该切片是一个“侧视”切片，
    默认取横向 ``y = -2.5 m``，与发射点横向 -2.87 m 接近。）

    返回 ``(xs, zs, heat)``，其中 ``heat[i_z, j_x] = R_occ(x_j, y, z_i)``。
    """
    xs = np.linspace(float(x_range[0]), float(x_range[1]), int(x_range[2]))
    zs = np.linspace(float(z_range[0]), float(z_range[1]), int(z_range[2]))
    heat = _scan_grid(
        cam_pos, uav_size, lamp,
        a_vals=xs, b_vals=zs, fixed_axis="y", fixed_value=float(y),
    )
    return xs, zs, heat


def static_grid_scan_xy(
    cam_pos: Sequence[float],
    uav_size: Sequence[float],
    lamp: LampConfig,
    z: float = 0.0,
    x_range: Tuple[float, float, int] = (-6.0, 0.2, 41),
    y_range: Tuple[float, float, int] = (-3.0, 3.0, 41),
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """在 ``z = 常数`` 的 XY 切片上扫描遮挡率。

    默认取 ``z = 0``（灯心高度水平面）。

    返回 ``(xs, ys, heat)``，其中 ``heat[i_y, j_x] = R_occ(x_j, y_i, z)``。
    """
    xs = np.linspace(float(x_range[0]), float(x_range[1]), int(x_range[2]))
    ys = np.linspace(float(y_range[0]), float(y_range[1]), int(y_range[2]))
    heat = _scan_grid(
        cam_pos, uav_size, lamp,
        a_vals=xs, b_vals=ys, fixed_axis="z", fixed_value=float(z),
    )
    return xs, ys, heat


def static_grid_scan_lamp_parallel(
    cam_pos: Sequence[float],
    uav_size: Sequence[float],
    lamp: LampConfig,
    standoff_m: float = 0.15,
    y_range: Tuple[float, float, int] = (-0.6, 0.6, 31),
    z_range: Tuple[float, float, int] = (-0.6, 0.6, 31),
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """在**平行于灯平面**、位于灯前方 ``standoff_m`` 的平面上扫描遮挡率。

    灯平面为 ``X = 0``（法线 ``-X``），因此扫描平面为 ``X = -standoff_m``，
    扫描坐标为场地横向 ``Y`` 与竖直 ``Z``。

    返回 ``(ys, zs, heat)``，其中 ``heat[i_z, j_y] = R_occ(-standoff_m, y_j, z_i)``。
    """
    ys = np.linspace(float(y_range[0]), float(y_range[1]), int(y_range[2]))
    zs = np.linspace(float(z_range[0]), float(z_range[1]), int(z_range[2]))
    heat = _scan_grid(
        cam_pos, uav_size, lamp,
        a_vals=ys, b_vals=zs, fixed_axis="x", fixed_value=-float(standoff_m),
    )
    return ys, zs, heat


def _scan_grid(
    cam_pos: Sequence[float],
    uav_size: Sequence[float],
    lamp: LampConfig,
    a_vals: np.ndarray,
    b_vals: np.ndarray,
    fixed_axis: str,
    fixed_value: float,
) -> np.ndarray:
    """通用栅格扫描：固定一个轴，另外两轴按 ``X -> Y -> Z`` 顺序取 ``(a, b)``。

    ``fixed_axis="x"`` -> ``(a, b) = (y, z)``；``"y"`` -> ``(x, z)``；``"z"`` -> ``(x, y)``。
    返回 ``heat[i_b, j_a]``。
    """
    lamp_poly = lamp_disc_polygon(lamp)
    cam = as_vector(cam_pos)
    n_invalid = 0
    total = len(a_vals) * len(b_vals)

    axis_map = {
        "x": ("y", "z"),
        "y": ("x", "z"),
        "z": ("x", "y"),
    }
    if fixed_axis not in axis_map:
        raise ValueError(f"不支持的 fixed_axis={fixed_axis!r}，可选 {tuple(axis_map)}")
    a_axis, b_axis = axis_map[fixed_axis]

    def _point(a_val: float, b_val: float) -> np.ndarray:
        coords = {fixed_axis: float(fixed_value), a_axis: float(a_val), b_axis: float(b_val)}
        return np.array([coords["x"], coords["y"], coords["z"]], dtype=float)

    rows: List[np.ndarray] = []
    for b_val in b_vals:
        row = np.zeros(len(a_vals), dtype=float)
        for j, a_val in enumerate(a_vals):
            point = _point(a_val, b_val)
            res = compute_occlusion(
                cam, point, uav_size, lamp, warn=False, lamp_polygon=lamp_poly
            )
            row[j] = res.R_occ
            if not res.valid:
                n_invalid += 1
        rows.append(row)

    heat = np.vstack(rows)
    if n_invalid > 0:
        warnings.warn(
            f"{n_invalid}/{total} 个扫描点的几何退化（无人机不在相机与灯平面之间等），"
            "对应 R_occ 记为 0.0",
            stacklevel=2,
        )
    return heat


# ---------------------------------------------------------------------------
# 动态仿真
# ---------------------------------------------------------------------------


def run_dynamic(
    strategy: Union[str, TargetPolicy],
    dart_cfg: DartConfig,
    uav_cfg: UAVConfig,
    lamp: LampConfig,
    sim_cfg: SimConfig,
    initial_uav_pos: Sequence[float],
    initial_uav_vel: Sequence[float] = (0.0, 0.0, 0.0),
    P_star: Optional[Sequence[float]] = None,
    warn: bool = False,
    kp: Optional[float] = None,
    kd: Optional[float] = None,
) -> Dict[str, np.ndarray]:
    """在 Vmax / Amax / 边界约束下运行一次动态仿真。

    ``strategy`` 可以是策略名（``fixed`` / ``los`` / ``predictive``），也可以是
    :class:`strategies.TargetPolicy` 实例（如 ``HoverStationPolicy``）。
    后者即“无人机机动”的预留接口：只要实现该接口，运动层与实验层无需改动。

    返回 ``dict``，键为 ``t, R, P_d, uav, v, loss``：

    * ``t``   ：时间序列 [s] (N,)
    * ``R``   ：几何遮挡率 ``R_occ(t)`` (N,)
    * ``P_d`` ：飞镖相机位置 ``C(t)`` (N, 3)
    * ``uav`` ：无人机中心位置 (N, 3)
    * ``v``   ：无人机速度 (N, 3)
    * ``loss``：该帧**引导灯仍可见（遮挡失效）**的指示量 (N,)，
      即 ``loss[k] = 0 if R_occ >= R_th else 1``。
    """
    policy: Optional[TargetPolicy] = strategy if isinstance(strategy, TargetPolicy) else None
    if policy is None and isinstance(strategy, str):
        # 允许直接传策略名（例如 "hover"）：内部自动构造对应策略对象。
        policy = make_policy(strategy)

    t = sim_cfg.time_array
    n = int(t.size)
    bounds = sim_cfg.body_safe_bounds(uav_cfg.size_m)
    lamp_poly = lamp_disc_polygon(lamp)

    P = clamp_to_bounds(initial_uav_pos, bounds)
    v = np.asarray(initial_uav_vel, dtype=float).reshape(3).copy()
    if float(np.linalg.norm(v)) > uav_cfg.max_speed:
        v = v * (uav_cfg.max_speed / float(np.linalg.norm(v)))
    P_star_arr = as_vector(P_star) if P_star is not None else P.copy()

    P_d = np.zeros((n, 3))
    uav = np.zeros((n, 3))
    vel = np.zeros((n, 3))
    R = np.zeros(n)
    loss = np.zeros(n)

    for k in range(n):
        C = dart_camera_trajectory(float(t[k]), dart_cfg)
        P_d[k] = C
        uav[k] = P
        vel[k] = v

        res = compute_occlusion(C, P, uav_cfg.size_m, lamp, warn=warn, lamp_polygon=lamp_poly)
        R[k] = res.R_occ
        loss[k] = 0.0 if is_effective(res.R_occ, sim_cfg.R_th) else 1.0

        if k < n - 1:
            target = policy.target(float(t[k]), dart_cfg, lamp, P_current=P, P_star=P_star_arr)
            target_vel = policy.target_velocity(float(t[k]), dart_cfg)
            P, v = update_uav_state(
                P, v, target, uav_cfg.max_speed, uav_cfg.max_accel, sim_cfg.dt, bounds,
                target_velocity=target_vel, kp=kp, kd=kd,
            )

    return {"t": t, "R": R, "P_d": P_d, "uav": uav, "v": vel, "loss": loss}


def truncate_run_before_contact(
    result: Dict[str, np.ndarray],
    stop_distance_m: float = 0.45,
    min_frames: int = 2,
) -> Dict[str, np.ndarray]:
    """在相机与无人机距离小于 ``stop_distance_m`` 之前截断仿真序列。

    抛物线飞镖的落点即绿灯中心，而悬停无人机位于灯前，因此末端两者必然相遇。
    截断可避免“相机与灯心重合”的奇异帧，也避免把撞机过程当作遮挡过程展示；
    飞镖完整弹道仍由 ``DartConfig.parabolic_position`` 给出，可视化中照常绘制。

    返回截断后的新字典（不修改入参）；若全程未低于阈值则原样返回。
    """
    if stop_distance_m <= 0.0:
        raise ValueError("stop_distance_m 必须为正")
    C = np.asarray(result["P_d"], dtype=float)
    U = np.asarray(result["uav"], dtype=float)
    if C.shape != U.shape:
        raise ValueError("P_d 与 uav 形状必须一致")
    dist = np.linalg.norm(C - U, axis=1)
    hit = np.where(dist <= float(stop_distance_m))[0]
    if hit.size == 0:
        return dict(result)
    k = max(int(min_frames), int(hit[0]))
    return {key: value[:k] for key, value in result.items()}


def camera_frame_metrics(
    result: Dict[str, np.ndarray],
    dart_cfg: DartConfig,
    camera,
    lamp: LampConfig,
    uav_size: Sequence[float],
    R_th: float = 0.7,
) -> Dict[str, float]:
    """相机**朝向/成像**相关统计量；光轴严格取弹道切线 ``dC/dt``。

    返回键：

    * ``fov_in_fraction``  ：灯心落在视场内的帧占比
    * ``fov_margin_min_deg``：灯心相对视场边缘的最小余量 [deg]（<0 即出画）
    * ``off_axis_max_deg`` ：灯心相对光轴的最大偏角 [deg]
    * ``image_R_min`` / ``image_R_mean``：像面遮挡比统计（无人机轮廓 ∩ 灯盘投影）
    * ``image_agreement``  ：像面判定与几何 ``R_occ`` 判定（``>= R_th``）一致的帧占比
    """
    from camera import camera_pose, fov_margin_deg, image_space_occlusion

    t = np.asarray(result["t"], dtype=float)
    uav = np.asarray(result["uav"], dtype=float)
    R = np.asarray(result["R"], dtype=float)

    margins: List[float] = []
    off_axis: List[float] = []
    image_R: List[float] = []
    for k in range(int(t.size)):
        pose = camera_pose(float(t[k]), dart_cfg)
        margins.append(fov_margin_deg(pose, lamp.center_array, camera))
        off_axis.append(pose.off_axis_angle_deg(lamp.center_array))
        ratio, _, _ = image_space_occlusion(pose, lamp, uav[k], uav_size, camera)
        image_R.append(float(ratio))

    margins_arr = np.asarray(margins, dtype=float)
    off_axis_arr = np.asarray(off_axis, dtype=float)
    image_arr = np.asarray(image_R, dtype=float)
    eff_geom = R >= float(R_th)
    eff_image = image_arr >= float(R_th)
    return {
        "fov_in_fraction": float(np.mean(margins_arr > 0.0)),
        "fov_margin_min_deg": float(margins_arr.min()),
        "off_axis_max_deg": float(off_axis_arr.max()),
        "image_R_min": float(image_arr.min()),
        "image_R_mean": float(image_arr.mean()),
        "image_agreement": float(np.mean(eff_geom == eff_image)),
    }


HOVER_SCAN_COLUMNS: Tuple[str, ...] = (
    "standoff_m",
    "distance_to_lamp_m",
    "offset_u_m",
    "offset_v_m",
    "clipped_to_bounds",
    "R_min",
    "R_mean",
    "R_max",
    "T_occ",
    "T_cont_max",
    "first_occlusion_t",
    "effective",
)


def hover_standoff_scan(
    lamp: LampConfig,
    uav_cfg: UAVConfig,
    dart_cfg: DartConfig,
    sim_cfg: SimConfig,
    standoffs_m: Sequence[float],
    offset_u_m: float = 0.0,
    offset_v_m: float = HOVER_STATION_OFFSET_V_M,
    stop_distance_m: float = 0.45,
    warn: bool = False,
) -> pd.DataFrame:
    """扫描**悬停站位到灯平面的距离**，输出每个距离下的遮挡指标。

    无人机在所有工况下都用 :class:`~strategies.HoverStationPolicy`（站住不动，
    不机动）——这正是当前阶段的要求；扫描回答的问题是“站位应当选在离灯多远处”。

    每一行的含义：

    * ``standoff_m``          ：站位沿灯法线到灯平面的距离（扫描变量）
    * ``distance_to_lamp_m``  ：站位到灯心的直线距离
    * ``clipped_to_bounds``   ：该站位是否被机身安全边界裁剪
    * ``R_min`` / ``R_mean`` / ``R_max``：整个飞行段的遮挡率统计
    * ``T_occ`` / ``T_cont_max`` / ``first_occlusion_t``：按 ``R_th`` 统计的遮挡时间
    * ``effective``           ：``R_min >= R_th``，即**全程**有效遮挡
    """
    rows: List[Dict[str, object]] = []
    bounds = sim_cfg.body_safe_bounds(uav_cfg.size_m)
    for standoff in standoffs_m:
        station = HoverStationConfig(
            standoff_m=float(standoff), offset_u_m=float(offset_u_m), offset_v_m=float(offset_v_m)
        )
        p_raw = station.position(lamp)
        p0 = clamp_to_bounds(p_raw, bounds)
        clipped = bool(not np.allclose(p0, p_raw))
        if clipped and warn:
            warnings.warn(
                f"站位 {np.round(p_raw, 4).tolist()} 超出机身安全边界，已裁剪为 "
                f"{np.round(p0, 4).tolist()}",
                stacklevel=2,
            )

        run = run_dynamic(
            HoverStationPolicy(station=station),
            dart_cfg,
            uav_cfg,
            lamp,
            sim_cfg,
            p0,
            P_star=p0,
            warn=warn,
        )
        run = truncate_run_before_contact(run, stop_distance_m=stop_distance_m)
        metrics = compute_metrics(run["t"], run["R"], sim_cfg.R_th)
        R = np.asarray(run["R"], dtype=float)
        rows.append(
            {
                "standoff_m": float(standoff),
                "distance_to_lamp_m": station.distance_to_lamp_m,
                "offset_u_m": float(offset_u_m),
                "offset_v_m": float(offset_v_m),
                "clipped_to_bounds": clipped,
                "R_min": float(R.min()),
                "R_mean": metrics["R_mean"],
                "R_max": metrics["R_max"],
                "T_occ": metrics["T_occ"],
                "T_cont_max": metrics["T_cont_max"],
                "first_occlusion_t": metrics["first_occlusion_t"],
                "effective": bool(R.min() >= float(sim_cfg.R_th)),
            }
        )

    return pd.DataFrame(rows, columns=list(HOVER_SCAN_COLUMNS))


def feasible_standoff_window(scan: pd.DataFrame) -> Optional[Tuple[float, float]]:
    """从扫描结果中取出**全程有效遮挡**的站位距离区间 ``[min, max]``（无解返回 None）。"""
    if scan.empty or "effective" not in scan.columns:
        return None
    ok = scan.loc[scan["effective"].astype(bool), "standoff_m"]
    if ok.empty:
        return None
    return (float(ok.min()), float(ok.max()))


# ---------------------------------------------------------------------------
# 指标
# ---------------------------------------------------------------------------


def _longest_true_run(mask: np.ndarray) -> int:
    """布尔序列中最长的连续 True 长度。"""
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
    """统计遮挡指标。

    * ``T_occ``             ：累计有效遮挡时间 ``sum(eff) * dt`` [s]
    * ``T_cont_max``        ：最长连续有效遮挡时长 [s]
    * ``first_occlusion_t`` ：首次有效遮挡时刻 [s]（无遮挡为 ``nan``）
    * ``R_max`` / ``R_mean``：遮挡率最大值 / 平均值
    """
    t_arr = np.asarray(t, dtype=float).reshape(-1)
    R_arr = np.asarray(R, dtype=float).reshape(-1)
    if t_arr.size != R_arr.size:
        raise ValueError("t 与 R 长度必须一致")
    if t_arr.size == 0:
        raise ValueError("t / R 不能为空")

    dt = float(np.mean(np.diff(t_arr))) if t_arr.size > 1 else float("nan")
    eff = R_arr >= float(R_th)
    T_occ = float(np.sum(eff) * dt) if t_arr.size > 1 else 0.0
    T_cont_max = float(_longest_true_run(eff) * dt) if t_arr.size > 1 else 0.0
    first_idx = int(np.argmax(eff)) if bool(np.any(eff)) else -1
    first_t = float(t_arr[first_idx]) if first_idx >= 0 else float("nan")

    return {
        "T_occ": T_occ,
        "T_cont_max": T_cont_max,
        "first_occlusion_t": first_t,
        "R_max": float(np.max(R_arr)),
        "R_mean": float(np.mean(R_arr)),
    }


# ---------------------------------------------------------------------------
# 实验计划与结果
# ---------------------------------------------------------------------------

