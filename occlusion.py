"""遮挡率计算 ``R_occ``。

定义（灯平面上的几何遮挡率）::

    R_occ = A_blocked / A_lamp

其中 ``A_blocked`` 是无人机 8 顶点阴影凸包与灯发光圆盘的**交集面积**，
``A_lamp = pi r^2`` 为灯的解析面积。``R_occ`` 恒被限制在 ``[0, 1]``。

重要说明
--------
``R_occ`` 只是**几何遮挡率**，不得被称为“视觉识别失败概率”，也不等价于官方
识别失效阈值；``R_th`` 是本项目定义的分析阈值（ENGINEERING_ASSUMPTION）。
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry

from config import LampConfig
from geometry import GeometryError, project_uav_shadow

#: 灯圆盘的多边形逼近精度：每象限 256 段 -> 总面积相对误差约 6e-6。
#: 由于分母使用解析面积 ``pi r^2``，“完全遮挡”时 R_occ 约为 0.999994（而非严格 1.0）。
LAMP_DISC_QUAD_SEGS: int = 256


class OcclusionGeometryWarning(UserWarning):
    """遮挡计算中遇到几何退化时的告警类型。"""


_MSG_NOT_BETWEEN = "无人机并非整体位于相机与灯平面之间，该帧 R_occ 记为 0.0"
_MSG_DEGENERATE = "无人机阴影投影退化（共线/重合/为空），该帧 R_occ 记为 0.0"
_MSG_NON_AREA = "阴影与灯的交集不是面状几何，该帧 R_occ 记为 0.0"


@dataclass(frozen=True)
class OcclusionResult:
    """遮挡计算的完整结果（``R_occ`` 之外还保留几何，便于可视化与校验）。"""

    R_occ: float
    valid: bool
    message: str = ""
    shadow_polygon: Optional[BaseGeometry] = None
    shadow_points_3d: Optional[np.ndarray] = None
    lamp_polygon: Optional[BaseGeometry] = None
    intersection_area_m2: float = 0.0


# ---------------------------------------------------------------------------
# 灯圆盘
# ---------------------------------------------------------------------------


def lamp_disc_polygon(lamp: LampConfig, quad_segs: int = LAMP_DISC_QUAD_SEGS) -> BaseGeometry:
    """灯平面局部坐标系（原点为灯心）下的灯圆盘多边形。"""
    return Point(0.0, 0.0).buffer(lamp.radius_m, quad_segs=quad_segs)


# ---------------------------------------------------------------------------
# 核心计算
# ---------------------------------------------------------------------------


def _bounds_overlap(a: BaseGeometry, b: BaseGeometry) -> bool:
    """快速排除：两个几何的轴对齐包围盒是否相交。"""
    if a.is_empty or b.is_empty:
        return False
    a_minx, a_miny, a_maxx, a_maxy = a.bounds
    b_minx, b_miny, b_maxx, b_maxy = b.bounds
    return not (a_maxx < b_minx or b_maxx < a_minx or a_maxy < b_miny or b_maxy < a_miny)


def compute_occlusion(
    C: Sequence[float],
    uav_center: Sequence[float],
    uav_size: Sequence[float],
    lamp: LampConfig,
    warn: bool = True,
    lamp_polygon: Optional[BaseGeometry] = None,
) -> OcclusionResult:
    """计算一帧的几何遮挡率及其几何对象。

    参数
    ----
    C : 相机位置 [m]
    uav_center : 无人机中心 [m]
    uav_size : 无人机三轴尺寸 [m]
    lamp : 灯配置
    warn : 遇到几何退化时是否发出 :class:`OcclusionGeometryWarning`
    lamp_polygon : 可选的灯圆盘缓存（批量扫描时复用，避免重复构造）
    """
    lamp_poly = lamp_polygon if lamp_polygon is not None else lamp_disc_polygon(lamp)

    try:
        shadow, points_3d = project_uav_shadow(
            C, uav_center, uav_size, lamp.center_array, lamp.normal_array
        )
    except GeometryError as exc:
        detail = str(exc)
        message = _MSG_NOT_BETWEEN if "之间" in detail else _MSG_DEGENERATE
        if warn:
            warnings.warn(f"{message}：{detail}", OcclusionGeometryWarning, stacklevel=2)
        return OcclusionResult(
            R_occ=0.0, valid=False, message=detail, lamp_polygon=lamp_poly,
            shadow_points_3d=None,
        )

    # 退化几何：空 / 非面状（LineString、Point 等）一律记为 0。
    if shadow.is_empty:
        if warn:
            warnings.warn(_MSG_DEGENERATE, OcclusionGeometryWarning, stacklevel=2)
        return OcclusionResult(
            R_occ=0.0, valid=False, message=_MSG_DEGENERATE, shadow_polygon=shadow,
            shadow_points_3d=points_3d, lamp_polygon=lamp_poly,
        )
    if shadow.geom_type not in {"Polygon", "MultiPolygon"}:
        if warn:
            warnings.warn(_MSG_DEGENERATE, OcclusionGeometryWarning, stacklevel=2)
        return OcclusionResult(
            R_occ=0.0, valid=False, message=_MSG_DEGENERATE, shadow_polygon=shadow,
            shadow_points_3d=points_3d, lamp_polygon=lamp_poly,
        )

    # 包围盒不相交 -> 交集面积为 0（几何有效，只是没挡住）。
    if not _bounds_overlap(shadow, lamp_poly):
        return OcclusionResult(
            R_occ=0.0, valid=True, message="", shadow_polygon=shadow,
            shadow_points_3d=points_3d, lamp_polygon=lamp_poly,
        )

    inter = shadow.intersection(lamp_poly)
    if inter.is_empty:
        area = 0.0
    elif inter.geom_type not in {"Polygon", "MultiPolygon"}:
        if warn:
            warnings.warn(_MSG_NON_AREA, OcclusionGeometryWarning, stacklevel=2)
        return OcclusionResult(
            R_occ=0.0, valid=False, message=_MSG_NON_AREA, shadow_polygon=shadow,
            shadow_points_3d=points_3d, lamp_polygon=lamp_poly,
        )
    else:
        area = float(inter.area)

    ratio = area / lamp.area_m2
    R_occ = float(min(1.0, max(0.0, ratio)))
    return OcclusionResult(
        R_occ=R_occ,
        valid=True,
        message="",
        shadow_polygon=shadow,
        shadow_points_3d=points_3d,
        lamp_polygon=lamp_poly,
        intersection_area_m2=area,
    )




def is_effective(R_occ: float, R_th: float) -> bool:
    """判断该帧是否记为**有效遮挡**：``R_occ >= R_th``。"""
    return bool(R_occ >= R_th)








