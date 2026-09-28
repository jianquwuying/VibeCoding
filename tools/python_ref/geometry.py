"""几何核心：射线与平面求交、平面局部正交基、无人机线框顶点与投影阴影。

约定
----
* 世界坐标为右手系，单位为米；绿色引导灯位于 ``X = 0`` 平面上的圆盘薄片，
  灯心即局部坐标原点 ``L = (0, 0, 0)``。
* 阴影投影方向由相机位置 ``C`` 指向灯的每个顶点（点光源 / 针孔近似）。
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np
from shapely.geometry import MultiPoint
from shapely.geometry.base import BaseGeometry

EPS = 1e-12


class GeometryError(ValueError):
    """几何退化或前提不满足。

    调用方可以选择把该异常降级为 ``R_occ = 0`` 并给出显式告警（见 ``occlusion``）。
    """


# ---------------------------------------------------------------------------
# 基础向量运算
# ---------------------------------------------------------------------------


def as_vector(v: Sequence[float]) -> np.ndarray:
    """把任意长度 3 的序列转换为 float64 一维数组。"""
    arr = np.asarray(v, dtype=float).reshape(-1)
    if arr.size != 3:
        raise GeometryError(f"需要 3 维向量，收到 {arr.size} 维：{v!r}")
    return arr


def normalize(v: Sequence[float]) -> np.ndarray:
    """返回单位向量；零向量视为几何异常。"""
    arr = as_vector(v)
    norm = float(np.linalg.norm(arr))
    if norm <= EPS:
        raise GeometryError("零向量无法归一化")
    return arr / norm




# ---------------------------------------------------------------------------
# 平面与投影
# ---------------------------------------------------------------------------


def project_point_to_plane(
    C: Sequence[float],
    P: Sequence[float],
    plane_center: Sequence[float],
    plane_normal: Sequence[float],
) -> Optional[np.ndarray]:
    """求从 ``C`` 出发、经过 ``P`` 的射线与平面的交点。

    返回交点坐标；若射线与平面平行（|投影分量| < EPS）或交点落在相机后方
    （参数 ``t < 0``），返回 ``None``（视为受控几何退化，由调用方处理）。
    """
    c = as_vector(C)
    p = as_vector(P)
    pc = as_vector(plane_center)
    n = normalize(plane_normal)

    direction = p - c
    denom = float(np.dot(direction, n))
    if abs(denom) < EPS:
        return None
    t = float(np.dot(pc - c, n)) / denom
    if t < 0.0:
        return None
    return c + t * direction


def plane_basis(plane_normal: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
    """返回平面内的一组右手正交单位基 ``(u, v)``。

    对灯平面法线 ``(-1, 0, 0)``，返回 ``u = (1,0,0)``、``v = (0,0,1)``，
    即灯平面局部坐标为 ``(x, z)``。
    """
    n = normalize(plane_normal)
    ref = np.array([0.0, 0.0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = normalize(np.cross(ref, n))
    v = normalize(np.cross(n, u))
    return u, v


def plane_local_coords(
    point: Sequence[float],
    plane_center: Sequence[float],
    u: Sequence[float],
    v: Sequence[float],
) -> np.ndarray:
    """把平面上的三维点转换为以 ``plane_center`` 为原点的平面局部二维坐标。"""
    d = as_vector(point) - as_vector(plane_center)
    return np.array([float(np.dot(d, u)), float(np.dot(d, v))])


def plane_local_to_world(
    coords_2d: Sequence[float],
    plane_center: Sequence[float],
    u: Sequence[float],
    v: Sequence[float],
) -> np.ndarray:
    """平面局部二维坐标 -> 世界三维坐标。"""
    c2 = np.asarray(coords_2d, dtype=float).reshape(-1)
    if c2.size != 2:
        raise GeometryError(f"平面局部坐标必须是 2 维，收到 {c2.size} 维")
    return as_vector(plane_center) + c2[0] * as_vector(u) + c2[1] * as_vector(v)


# ---------------------------------------------------------------------------
# 无人机线框
# ---------------------------------------------------------------------------


def uav_corners(center: Sequence[float], size: Sequence[float]) -> np.ndarray:
    """返回轴对齐长方体的 8 个顶点，形状 (8, 3)。

    ``size`` 三轴可以不同（长=X / 宽=Y / 高=Z），半尺寸逐轴取 ``|size[i]| / 2``。

    顶点顺序由符号元组 ``(sx, sy, sz) ∈ {-1, +1}^3`` 生成，索引满足
    ``idx = 4*a + 2*b + c``（a 对应 x，b 对应 y，c 对应 z），
    因此 ``idx ^ 1``、``idx ^ 2``、``idx ^ 4`` 一定是相邻顶点（用于画线框）。
    """
    c = as_vector(center)
    half = np.abs(as_vector(size)) / 2.0
    signs = np.array(
        [[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)],
        dtype=float,
    )
    return c + signs * half


def uav_edges() -> List[Tuple[int, int]]:
    """返回长方体线框的 12 条棱（顶点索引对，与 :func:`uav_corners` 顺序一致）。"""
    edges: List[Tuple[int, int]] = []
    for i in range(8):
        for bit in (1, 2, 4):
            j = i ^ bit
            if i < j:
                edges.append((i, j))
    return edges


# ---------------------------------------------------------------------------
# 阴影投影
# ---------------------------------------------------------------------------


def all_vertices_between_camera_and_plane(
    C: Sequence[float],
    corners: Sequence[Sequence[float]],
    plane_center: Sequence[float],
    plane_normal: Sequence[float],
    tol: float = 1e-9,
) -> bool:
    """判断全部顶点是否整体位于相机与灯平面之间。

    判据（沿平面法线的有符号距离 ``d``）：

    * 相机不在平面上：``|d_C| > tol``；
    * 每个顶点与相机同侧：``d_P * d_C > 0``；
    * 每个顶点不比相机更远：``|d_P| <= |d_C| + tol``。
    """
    n = normalize(plane_normal)
    pc = as_vector(plane_center)
    d_cam = float(np.dot(as_vector(C) - pc, n))
    if abs(d_cam) <= tol:
        return False

    pts = np.asarray(corners, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] == 0:
        raise GeometryError(f"顶点数组形状非法：{pts.shape}")

    d_pts = (pts - pc) @ n
    if np.any(d_pts * d_cam <= 0.0):
        return False
    if np.any(np.abs(d_pts) > abs(d_cam) + tol):
        return False
    return True


def project_uav_shadow(
    C: Sequence[float],
    uav_center: Sequence[float],
    uav_size: Sequence[float],
    plane_center: Sequence[float],
    plane_normal: Sequence[float],
) -> Tuple[BaseGeometry, np.ndarray]:
    """把无人机 8 个顶点从相机投到灯平面，得到阴影多边形。

    返回
    ----
    ``(polygon_2d, points_3d)``

    * ``polygon_2d``：灯平面局部坐标系下的 shapely 几何（凸包），以 ``plane_center``
      为原点。若 8 个投影点共线或重合，凸包会退化为 ``LineString`` / ``Point``，
      本函数**不隐藏**该退化，交由调用方按 ``geom_type`` 判定为 0。
    * ``points_3d``：形状 (8, 3) 的投影点世界坐标，可用于 3D 可视化。

    异常
    ----
    :class:`GeometryError`
        * 无人机未整体位于相机与灯平面之间；
        * 某条射线与平面平行，无法求交。
    """
    corners = uav_corners(uav_center, uav_size)
    if not all_vertices_between_camera_and_plane(C, corners, plane_center, plane_normal):
        raise GeometryError(
            "无人机未整体位于相机与灯平面之间（可能已穿过灯平面或落后于相机）"
        )

    projected: List[np.ndarray] = []
    for corner in corners:
        p = project_point_to_plane(C, corner, plane_center, plane_normal)
        if p is None:
            raise GeometryError("射线与灯平面平行或交点位于相机后方，投影退化")
        projected.append(p)
    points_3d = np.asarray(projected, dtype=float)

    u, v = plane_basis(plane_normal)
    points_2d = np.asarray(
        [plane_local_coords(p, plane_center, u, v) for p in points_3d], dtype=float
    )
    polygon_2d = MultiPoint(points_2d).convex_hull
    return polygon_2d, points_3d


# ---------------------------------------------------------------------------
# M2（v2）：视线方向的平面基
# ---------------------------------------------------------------------------


def los_basis(
    C: Sequence[float], L: Sequence[float]
) -> Tuple[np.ndarray, np.ndarray]:
    """视线方向的平面基：法线 = ``normalize(L - C)``。

    M2 中灯是球体，遮挡计算不再使用固定的灯面法线，而是把无人机阴影投影到
    "过灯心且垂直于当前视线"的平面上（§2.6 / §1.5 的视线方向圆盘近似）。
    """
    los_hat = normalize(as_vector(L) - as_vector(C))
    return plane_basis(los_hat)


