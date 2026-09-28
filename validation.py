"""一致性检查模块。

所有检查函数返回 ``ValidationIssue``（或 ``None`` 表示通过），由 :func:`build_report`
汇总为 :class:`ValidationReport`。硬性问题抛 :class:`ValidationError`，其余以
``warning`` 形式显式提示，绝不静默通过异常几何。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence

import numpy as np

from config import CameraConfig, DartConfig, HoverStationConfig, LampConfig, SimConfig, UAVConfig
from geometry import (
    GeometryError,
    all_vertices_between_camera_and_plane,
    as_vector,
    project_uav_shadow,
    uav_corners,
)

ERROR = "error"
WARNING = "warning"


class ValidationError(ValueError):
    """硬性校验失败。"""


@dataclass(frozen=True)
class ValidationIssue:
    """单条校验结果。"""

    severity: str
    code: str
    message: str

    def __str__(self) -> str:  # pragma: no cover - 仅用于打印
        return f"[{self.severity.upper()}] {self.code}: {self.message}"


@dataclass
class ValidationReport:
    """校验汇总报告。"""

    issues: List[ValidationIssue] = field(default_factory=list)

    @property
    def errors(self) -> List[ValidationIssue]:
        return [i for i in self.issues if i.severity == ERROR]

    @property
    def warnings(self) -> List[ValidationIssue]:
        return [i for i in self.issues if i.severity == WARNING]

    @property
    def ok(self) -> bool:
        return len(self.errors) == 0

    def add(self, issue: Optional[ValidationIssue]) -> None:
        if issue is not None:
            self.issues.append(issue)

    def extend(self, issues: Iterable[Optional[ValidationIssue]]) -> None:
        for issue in issues:
            self.add(issue)

    def raise_if_errors(self) -> None:
        if not self.ok:
            joined = "\n".join(str(i) for i in self.errors)
            raise ValidationError("校验失败：\n" + joined)

    def format(self) -> str:
        lines = [
            f"校验结果：{len(self.errors)} 个错误 / {len(self.warnings)} 个告警"
        ]
        lines.extend(f"  {issue}" for issue in self.issues)
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# 单项检查
# ---------------------------------------------------------------------------


def check_lamp_diameter(lamp: LampConfig) -> Optional[ValidationIssue]:
    """灯直径必须 > 0（并在合理范围内）。"""
    if lamp.diameter_mm <= 0.0:
        return ValidationIssue(ERROR, "lamp.diameter", f"灯直径必须 > 0，当前 {lamp.diameter_mm} mm")
    if not (5.0 <= lamp.diameter_mm <= 500.0):
        return ValidationIssue(
            WARNING, "lamp.diameter.range",
            f"灯直径 {lamp.diameter_mm} mm 超出常规取值范围 [5, 500] mm，请确认输入",
        )
    return None


def check_camera_lamp_separation(cam_pos: Sequence[float], lamp: LampConfig) -> Optional[ValidationIssue]:
    """相机不能与灯心重合。"""
    d = float(np.linalg.norm(as_vector(cam_pos) - lamp.center_array))
    if d <= 1e-9:
        return ValidationIssue(ERROR, "camera.position", "相机位置与灯心重合，无法定义投影射线")
    if d < lamp.radius_m:
        return ValidationIssue(
            WARNING, "camera.position.close",
            f"相机与灯心距离 {d:.4f} m 小于灯半径 {lamp.radius_m:.4f} m，几何意义有限",
        )
    return None


def check_uav_between_camera_and_plane(
    cam_pos: Sequence[float],
    uav_center: Sequence[float],
    uav_size: Sequence[float],
    lamp: LampConfig,
    severity: str = ERROR,
) -> Optional[ValidationIssue]:
    """无人机必须整体位于相机与灯平面之间。

    ``severity`` 可设为 ``WARNING``：动态仿真中该情形已被显式处理为 ``R_occ = 0``
    并给出告警，属于受控退化，而非程序错误。
    """
    corners = uav_corners(uav_center, uav_size)
    ok = all_vertices_between_camera_and_plane(
        cam_pos, corners, lamp.center_array, lamp.normal_array
    )
    if not ok:
        return ValidationIssue(
            severity, "uav.between",
            "无人机未整体位于相机与灯平面之间，该帧遮挡率应记为 0.0",
        )
    return None


def check_bounds_valid(bounds: dict, uav_size_m: Sequence[float]) -> Optional[ValidationIssue]:
    """边界必须合法，且机身安全边界非空。"""
    for axis in ("x", "y", "z"):
        if axis not in bounds:
            return ValidationIssue(ERROR, "bounds.missing", f"边界缺少轴 {axis!r}")
        lo, hi = bounds[axis]
        if not (math.isfinite(lo) and math.isfinite(hi)) or lo >= hi:
            return ValidationIssue(ERROR, "bounds.interval", f"边界 {axis} 非法：({lo}, {hi})")

    half = np.asarray(uav_size_m, dtype=float) / 2.0
    for i, axis in enumerate(("x", "y", "z")):
        lo, hi = bounds[axis]
        if lo + half[i] > hi - half[i]:
            return ValidationIssue(
                ERROR, "bounds.body_safe",
                f"轴 {axis} 无法容纳机身尺寸 {2 * half[i]:.4f} m（范围 {hi - lo:.4f} m）",
            )
    return None


def check_uav_body_in_bounds(
    trajectory: Sequence[Sequence[float]],
    bounds: dict,
    uav_size_m: Sequence[float],
    tol: float = 1e-6,
) -> Optional[ValidationIssue]:
    """无人机机身在整个轨迹中都不得越界（按中心坐标 + 半尺寸复核）。"""
    arr = np.asarray(trajectory, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 3:
        return ValidationIssue(ERROR, "uav.trajectory.shape", f"轨迹形状非法：{arr.shape}")
    half = np.asarray(uav_size_m, dtype=float) / 2.0
    for i, axis in enumerate(("x", "y", "z")):
        lo, hi = bounds[axis]
        if np.any(arr[:, i] - half[i] < lo - tol) or np.any(arr[:, i] + half[i] > hi + tol):
            return ValidationIssue(
                ERROR, "uav.out_of_bounds",
                f"轴 {axis} 上机身越界（中心范围 [{arr[:, i].min():.4f}, {arr[:, i].max():.4f}]，"
                f"机身尺寸 {2 * half[i]:.4f} m，边界 ({lo}, {hi})）",
            )
    return None


def check_R_occ_range(R: Sequence[float]) -> Optional[ValidationIssue]:
    """遮挡率必须落在 [0, 1]。"""
    arr = np.asarray(R, dtype=float).reshape(-1)
    if arr.size == 0:
        return ValidationIssue(ERROR, "R_occ.empty", "遮挡率序列为空")
    if not np.all(np.isfinite(arr)):
        return ValidationIssue(ERROR, "R_occ.finite", "遮挡率序列包含 NaN / inf")
    lo, hi = float(arr.min()), float(arr.max())
    if lo < -1e-9 or hi > 1.0 + 1e-9:
        return ValidationIssue(ERROR, "R_occ.range", f"遮挡率越界：[{lo:.4f}, {hi:.4f}]")
    if hi <= 0.0:
        return ValidationIssue(
            WARNING, "R_occ.zero",
            "整个序列遮挡率恒为 0，请确认无人机是否始终位于相机与灯之间并靠近视线",
        )
    return None




def check_velocity_limits(
    t: Sequence[float],
    v: Sequence[Sequence[float]],
    v_max: float,
    a_max: float,
    rtol: float = 1e-6,
) -> Optional[ValidationIssue]:
    """速度模长不超过 Vmax，且相邻速度变化不超过 Amax * dt。"""
    t_arr = np.asarray(t, dtype=float).reshape(-1)
    v_arr = np.asarray(v, dtype=float)
    if v_arr.ndim != 2 or v_arr.shape[1] != 3:
        return ValidationIssue(ERROR, "uav.velocity.shape", f"速度形状非法：{v_arr.shape}")

    speed = np.linalg.norm(v_arr, axis=1)
    if float(speed.max()) > v_max * (1.0 + rtol) + 1e-9:
        idx = int(np.argmax(speed))
        return ValidationIssue(
            ERROR, "uav.v_max",
            f"t = {t_arr[idx]:.3f} s 处速度 {speed[idx]:.4f} m/s 超过 Vmax = {v_max:.4f} m/s",
        )
    if v_arr.shape[0] >= 2:
        dt = float(t_arr[1] - t_arr[0])
        if dt <= 0.0:
            return ValidationIssue(ERROR, "uav.dt", "时间步长必须为正")
        dv = np.linalg.norm(np.diff(v_arr, axis=0), axis=1)
        limit = a_max * dt * (1.0 + rtol) + 1e-9
        if float(dv.max()) > limit:
            idx = int(np.argmax(dv))
            return ValidationIssue(
                ERROR, "uav.a_max",
                f"t ≈ {t_arr[idx + 1]:.3f} s 处速度增量 {dv[idx]:.4f} m/s 超过 "
                f"Amax*dt = {a_max * dt:.4f} m/s",
            )
    return None


def check_camera_config(camera: CameraConfig) -> List[Optional[ValidationIssue]]:
    """相机内参相关的提示性检查。"""
    issues: List[Optional[ValidationIssue]] = []
    # 缺少靶面尺寸，焦距不可用于 FOV 推导。
    issues.append(
        ValidationIssue(
            WARNING, "camera.sensor_size",
            f"传感器靶面尺寸来源为 UNKNOWN，焦距 {camera.focal_length_mm} mm 不参与 FOV 推导；"
            f"FOV 采用 HFOV = {camera.fov_h_deg} deg（ENGINEERING_ASSUMPTION）",
        )
    )
    return issues


def check_hover_station(
    station: HoverStationConfig,
    lamp: LampConfig,
    uav: UAVConfig,
    sim: SimConfig,
    tol: float = 1e-9,
) -> Optional[ValidationIssue]:
    """悬停站位检查：必须在灯前方、在机身安全边界内，且不与灯心重合。

    站位是当前阶段唯一的“机动方式”，因此这里的硬性错误会直接终止运行；
    若想探索越界站位，请用 ``experiment.hover_standoff_scan``（它会记录裁剪标志）。
    """
    if station.standoff_m <= 0.0:
        return ValidationIssue(ERROR, "hover.standoff", f"站位法向距离必须为正：{station.standoff_m!r} m")

    p = station.position(lamp)
    signed = float(np.dot(p - lamp.center_array, lamp.normal_array))
    if signed <= tol:
        return ValidationIssue(
            ERROR, "hover.behind_lamp",
            f"站位 {np.round(p, 4).tolist()} 不在灯前方（沿灯法线距离 {signed:.4f} m <= 0）",
        )
    if station.distance_to_lamp_m <= lamp.radius_m:
        return ValidationIssue(
            WARNING, "hover.too_close",
            f"站位距灯心 {station.distance_to_lamp_m:.4f} m 不大于灯半径 {lamp.radius_m:.4f} m，"
            "几何意义上机身已与灯盘重叠",
        )

    bounds = sim.body_safe_bounds(uav.size_m)
    for i, axis in enumerate(("x", "y", "z")):
        lo, hi = bounds[axis]
        if p[i] < lo - tol or p[i] > hi + tol:
            return ValidationIssue(
                ERROR, "hover.out_of_bounds",
                f"站位 {axis} = {p[i]:.4f} m 超出机身安全边界 ({lo:.4f}, {hi:.4f})；"
                f"请调整 --standoff / --offset-v",
            )
    return None


def validate_scene(
    lamp: LampConfig,
    camera: CameraConfig,
    uav: UAVConfig,
    dart: DartConfig,
    sim: SimConfig,
    station: Optional[HoverStationConfig] = None,
) -> ValidationReport:
    """静态场景校验（灯、相机、无人机、飞镖轨迹、边界、悬停站位）。"""
    report = ValidationReport()
    report.add(check_lamp_diameter(lamp))
    report.extend(check_camera_config(camera))
    report.add(check_bounds_valid(dict(sim.uav_bounds), uav.size_m))
    if station is not None:
        report.add(check_hover_station(station, lamp, uav, sim))

    # 初始时刻的几何检查：相机 -> 灯 与 无人机站位。
    from motion import dart_camera_trajectory

    c0 = dart_camera_trajectory(0.0, dart)
    report.add(check_camera_lamp_separation(c0, lamp))
    return report


def validate_run(
    result: dict,
    uav: UAVConfig,
    sim: SimConfig,
    lamp: LampConfig,
    check_frames: int = 25,
) -> ValidationReport:
    """动态结果校验：指标范围、边界、速度/加速度约束、抽样帧几何。"""
    report = ValidationReport()
    t = np.asarray(result["t"], dtype=float)
    R = np.asarray(result["R"], dtype=float)
    uav_traj = np.asarray(result["uav"], dtype=float)
    vel = np.asarray(result["v"], dtype=float)
    P_d = np.asarray(result["P_d"], dtype=float)

    report.add(check_R_occ_range(R))
    report.add(check_uav_body_in_bounds(uav_traj, dict(sim.uav_bounds), uav.size_m))
    report.add(check_velocity_limits(t, vel, uav.max_speed, uav.max_accel))

    if not report.ok:
        return report

    # 抽样帧的几何校验（避免 54 x 151 帧全量校验带来的开销）。
    n = int(t.size)
    step = max(1, n // max(1, int(check_frames)))
    for k in range(0, n, step):
        report.add(
            check_uav_between_camera_and_plane(
                P_d[k], uav_traj[k], uav.size_m, lamp, severity=WARNING
            )
        )
    return report


