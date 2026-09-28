"""飞镖相机姿态与成像：光轴**严格取弹道切线方向**，全程随弹道变化。

模型说明
--------
* **光轴**：``forward = dC/dt`` 的单位向量，即飞镖“朝哪飞就看哪”；
  ``right`` / ``up`` 由 ``CAMERA_UP_REF``（默认世界 +Z）构造正交基。
* **光心**：``C(t)``（弹道位置）。几何遮挡率 ``R_occ`` 只取决于光心位置
  （针孔/点光源投影），相机朝向不影响阴影形状；但朝向决定**灯是否落在视场内**，
  以及第一人称画面长什么样，因此两者都要算。
* **内参**：``f_x = (W/2) / tan(HFOV/2)``，``f_y = f_x``，主点取画面中心。

坐标系（相机系）：``x_c`` 向右、``y_c`` 向上、``z_c`` 沿光轴向前。
像素坐标：``u = cx + f * x_c / z_c``、``v = cy - f * y_c / z_c``（``z_c > 0`` 才在相机前方）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np
from shapely.geometry import MultiPoint

from config import CAMERA_UP_REF, CameraConfig, LampConfig
from geometry import as_vector, normalize

EPS = 1e-9


@dataclass(frozen=True)
class CameraPose:
    """相机位姿（光心 + 正交基）。``forward`` 即光轴。"""

    position: np.ndarray
    forward: np.ndarray
    right: np.ndarray
    up: np.ndarray

    def to_camera_frame(self, point: Sequence[float]) -> np.ndarray:
        """把世界点变换到相机系 ``(x_c, y_c, z_c)``。"""
        d = as_vector(point) - self.position
        return np.array(
            [float(np.dot(d, self.right)), float(np.dot(d, self.up)), float(np.dot(d, self.forward))]
        )

    def distance(self, point: Sequence[float]) -> float:
        return float(np.linalg.norm(as_vector(point) - self.position))

    def off_axis_angle_deg(self, point: Sequence[float]) -> float:
        """目标点相对光轴的夹角 [deg]（0 = 正对光轴）。"""
        d = as_vector(point) - self.position
        norm = float(np.linalg.norm(d))
        if norm <= EPS:
            return 0.0
        cos_t = float(np.clip(np.dot(d / norm, self.forward), -1.0, 1.0))
        return float(np.degrees(np.arccos(cos_t)))

    def contains(self, point: Sequence[float], camera: CameraConfig) -> bool:
        """目标点是否落在视场内（水平/垂直半视场角同时满足）。"""
        c = self.to_camera_frame(point)
        if c[2] <= EPS:
            return False
        h_half = np.tan(camera.fov_h_rad / 2.0)
        v_half = h_half / camera.aspect
        return (abs(c[0]) <= c[2] * h_half) and (abs(c[1]) <= c[2] * v_half)

    def project_pixel(self, point: Sequence[float], camera: CameraConfig) -> Optional[np.ndarray]:
        """投影到像素坐标 ``(u, v)``；目标在相机后方（``z_c <= 0``）时返回 None。"""
        c = self.to_camera_frame(point)
        if c[2] <= EPS:
            return None
        f = camera.focal_px
        cx = camera.resolution[0] / 2.0
        cy = camera.resolution[1] / 2.0
        return np.array([cx + f * c[0] / c[2], cy - f * c[1] / c[2]])


def pose_from_position_and_axis(
    position: Sequence[float],
    axis: Sequence[float],
    up_ref: Sequence[float] = CAMERA_UP_REF,
) -> CameraPose:
    """由光心与光轴构造位姿；``up_ref`` 仅用于确定滚转（roll）。"""
    pos = as_vector(position)
    forward = normalize(axis)
    up_hint = np.asarray(up_ref, dtype=float).reshape(3)
    right = np.cross(forward, up_hint)
    if float(np.linalg.norm(right)) <= EPS:  # 光轴与 up_ref 平行，换一个参考轴
        alt = np.array([0.0, 1.0, 0.0])
        right = np.cross(forward, alt)
        if float(np.linalg.norm(right)) <= EPS:
            alt = np.array([1.0, 0.0, 0.0])
            right = np.cross(forward, alt)
    right = normalize(right)
    up = normalize(np.cross(right, forward))
    return CameraPose(position=pos, forward=forward, right=right, up=up)


# ---------------------------------------------------------------------------
# 视场与成像
# ---------------------------------------------------------------------------


def fov_margin_deg(pose: CameraPose, point: Sequence[float], camera: CameraConfig) -> float:
    """目标点相对视场边缘的余量 [deg]：>0 在视场内，<0 已出画。

    取水平/垂直两个方向里**更紧**的一个。
    """
    c = pose.to_camera_frame(point)
    if c[2] <= EPS:
        return -90.0
    half_h = camera.fov_h_deg / 2.0
    half_v = camera.fov_v_deg / 2.0
    az = float(np.degrees(np.arctan2(c[0], c[2])))   # 水平偏角
    el = float(np.degrees(np.arctan2(c[1], c[2])))   # 垂直偏角
    return float(min(half_h - abs(az), half_v - abs(el)))


def project_circle_polygon(
    pose: CameraPose,
    center: Sequence[float],
    radius_m: float,
    basis_u: Sequence[float],
    basis_v: Sequence[float],
    camera: CameraConfig,
    n_segments: int = 128,
) -> Optional[list]:
    """把三维圆盘投影到像面，返回像素多边形顶点列表（不可见时返回 None）。"""
    c = as_vector(center)
    u = as_vector(basis_u)
    v = as_vector(basis_v)
    theta = np.linspace(0.0, 2.0 * np.pi, int(n_segments), endpoint=False)
    pts = []
    for a in theta:
        world = c + radius_m * (np.cos(a) * u + np.sin(a) * v)
        px = pose.project_pixel(world, camera)
        if px is None:
            return None
        pts.append(px)
    return pts


def project_points_hull(
    pose: CameraPose,
    points: Sequence[Sequence[float]],
    camera: CameraConfig,
):
    """把一组世界点投影到像面并取凸包（shapely 多边形）；不可见返回 None。"""
    px = []
    for p in points:
        q = pose.project_pixel(p, camera)
        if q is None:
            return None
        px.append(q)
    return MultiPoint(px).convex_hull


def image_space_occlusion(
    pose: CameraPose,
    lamp: LampConfig,
    los_hat: Sequence[float],
    uav_center: Sequence[float],
    uav_size: Sequence[float],
    camera: CameraConfig,
    n_segments: int = 128,
) -> Tuple[float, object, object]:
    """像面遮挡比：无人机轮廓与灯球像面投影的交集面积 / 灯球投影面积。

    返回 ``(ratio, lamp_polygon_px, uav_polygon_px)``；任一不可见时 ratio 为 0。

    注意：这只是**成像平面上的一个补充指标**（便于直观核对与出图），
    主指标仍然是灯平面上的几何遮挡率 ``R_occ``。
    """
    from geometry import plane_basis, uav_corners

    # 灯是球体（无朝向），像面近似圆盘的径向基取"当前视线方向的平面基"，
    # 不再使用固定的灯面法线（lamp.normal 仅作占位保留）。
    u_hat, v_hat = plane_basis(as_vector(los_hat))
    lamp_px = project_circle_polygon(
        pose, lamp.center_array, lamp.radius_m, u_hat, v_hat, camera, n_segments
    )
    corners = uav_corners(uav_center, uav_size)
    uav_poly = project_points_hull(pose, corners, camera)
    if lamp_px is None or uav_poly is None or uav_poly.is_empty:
        return 0.0, None, None

    lamp_poly = MultiPoint(lamp_px).convex_hull
    if lamp_poly.is_empty or lamp_poly.area <= 0.0:
        return 0.0, None, None
    inter = lamp_poly.intersection(uav_poly)
    area = 0.0 if inter.is_empty else float(inter.area)
    return float(min(1.0, max(0.0, area / lamp_poly.area))), lamp_poly, uav_poly




def frustum_corner_rays(
    pose: CameraPose,
    camera: CameraConfig,
    length_m: float,
) -> np.ndarray:
    """视锥四条角射线在 ``length_m`` 处的端点（用于 3D 绘制）。"""
    h_half = np.tan(camera.fov_h_rad / 2.0) * float(length_m)
    v_half = h_half / camera.aspect
    corners = []
    for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
        corners.append(pose.position + length_m * pose.forward + sx * h_half * pose.right + sy * v_half * pose.up)
    return np.asarray(corners, dtype=float)

