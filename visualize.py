"""可视化模块（matplotlib，Agg 后端，dpi = 200，全部保存到 ``out/``）。

输出：

* :func:`plot_heatmap`        —— 遮挡率热图 + ``R_th`` 红色等值线
* :func:`plot_R_occ_curves`   —— 多条 ``R_occ(t)`` 曲线 + ``R_th`` 虚线
* :func:`plot_3d_scene`       —— 灯盘、相机、无人机线框、LOS、阴影多边形与投影线
"""

from __future__ import annotations

import shutil
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple, Union

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rcParams
from matplotlib.animation import FFMpegWriter, PillowWriter
from matplotlib.ticker import MaxNLocator
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Polygon as MplPolygon, Rectangle
from shapely.geometry import box as shapely_box
from shapely.geometry.base import BaseGeometry

from camera import camera_pose, fov_margin_deg, image_space_occlusion
from config import LampConfig
from geometry import (
    as_vector,
    plane_basis,
    plane_local_coords,
    plane_local_to_world,
    uav_corners,
    uav_edges,
)
from occlusion import compute_occlusion

# 中文字体回退（Windows 常见字体优先）；缺失时 matplotlib 会退回默认字体。
rcParams["font.sans-serif"] = [
    "Microsoft YaHei",
    "SimHei",
    "Noto Sans CJK SC",
    "Source Han Sans SC",
    "DejaVu Sans",
]
rcParams["axes.unicode_minus"] = False

DPI = 200

#: 全程弹道面板的 Z 向视觉放大倍数（仅影响显示，不参与任何几何计算）。
WIDE_Z_EXAGGERATION: float = 3.0

#: 动画输出支持的格式。
VIDEO_SUFFIXES = (".mp4", ".m4v", ".mov")
ANIMATION_SUFFIXES = (".gif",) + VIDEO_SUFFIXES


def _ensure_parent(save_path: Union[str, Path]) -> Path:
    path = Path(save_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


# ---------------------------------------------------------------------------
# 动画后端（GIF / MP4）
# ---------------------------------------------------------------------------


def resolve_ffmpeg_path() -> Optional[str]:
    """返回可用的 ffmpeg 可执行文件路径；找不到返回 ``None``。

    查找顺序：

    1. ``matplotlib.rcParams["animation.ffmpeg_path"]`` 若被显式设置为真实路径；
    2. 系统 ``PATH`` 中的 ``ffmpeg``；
    3. ``imageio-ffmpeg`` 自带的内置二进制（``pip install imageio-ffmpeg``）。
    """
    configured = str(rcParams.get("animation.ffmpeg_path", "ffmpeg") or "ffmpeg")
    if configured != "ffmpeg":
        if Path(configured).exists():
            return configured

    found = shutil.which("ffmpeg")
    if found:
        return found

    try:  # 可选依赖：自带 ffmpeg 二进制，无需系统安装、无需管理员权限
        import imageio_ffmpeg  # type: ignore

        exe = imageio_ffmpeg.get_ffmpeg_exe()
        if exe and Path(exe).exists():
            return str(exe)
    except Exception:
        pass
    return None


def describe_animation_backend() -> str:
    """返回当前环境可用的动画后端说明（用于日志）。"""
    ffmpeg = resolve_ffmpeg_path()
    if ffmpeg:
        return f"GIF (Pillow) + MP4 (ffmpeg: {ffmpeg})"
    return "GIF (Pillow)；MP4 不可用（未找到 ffmpeg，可 pip install imageio-ffmpeg）"


def save_animation(
    anim,
    save_path: Union[str, Path],
    fps: int = 25,
    dpi: int = 80,
    bitrate: int = 4000,
) -> Path:
    """保存动画：``.gif`` 用 PillowWriter，``.mp4/.m4v/.mov`` 用 FFMpegWriter。

    MP4 需要 ffmpeg（系统安装或 ``imageio-ffmpeg`` 内置二进制），否则抛出带有
    安装指引的 ``RuntimeError``。为保证播放器兼容性，MP4 使用 H.264 + yuv420p，
    并把宽高强制为偶数（libx264 要求）。
    """
    path = _ensure_parent(save_path)
    suffix = path.suffix.lower()

    if suffix == ".gif":
        anim.save(path, writer=PillowWriter(fps=int(fps)), dpi=dpi)
        return path

    if suffix in VIDEO_SUFFIXES:
        ffmpeg = resolve_ffmpeg_path()
        if ffmpeg is None:
            raise RuntimeError(
                "未找到 ffmpeg，无法输出 MP4。三种解决方式：\n"
                "  1) pip install imageio-ffmpeg   # 自带 ffmpeg 二进制，免管理员权限\n"
                "  2) 安装系统 ffmpeg 并加入 PATH   # winget install Gyan.FFmpeg\n"
                "  3) python main.py --frames      # 导出 PNG 帧序列，自行用 ffmpeg 合成"
            )
        rcParams["animation.ffmpeg_path"] = ffmpeg
        writer = FFMpegWriter(
            fps=int(fps),
            bitrate=bitrate,
            codec="libx264",
            extra_args=[
                "-pix_fmt", "yuv420p",
                "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
                "-movflags", "+faststart",
            ],
        )
        anim.save(path, writer=writer, dpi=dpi)
        return path

    raise ValueError(f"不支持的动画后缀 {suffix!r}，可选 {ANIMATION_SUFFIXES}")






# ---------------------------------------------------------------------------
# 热图
# ---------------------------------------------------------------------------


def plot_heatmap(
    heat: np.ndarray,
    x_arr: Sequence[float],
    y_arr: Sequence[float],
    xlabel: str,
    ylabel: str,
    title: str,
    R_th: float,
    save_path: Union[str, Path],
) -> Path:
    """绘制遮挡率热图，并叠加 ``R_th`` 红色等值线与最大遮挡点。"""
    heat = np.asarray(heat, dtype=float)
    xs = np.asarray(x_arr, dtype=float)
    ys = np.asarray(y_arr, dtype=float)
    if heat.shape != (ys.size, xs.size):
        raise ValueError(f"heat 形状 {heat.shape} 与 (len(y), len(x)) = {(ys.size, xs.size)} 不一致")

    fig, ax = plt.subplots(figsize=(7.2, 5.4))
    extent = [float(xs[0]), float(xs[-1]), float(ys[0]), float(ys[-1])]
    im = ax.imshow(
        heat,
        origin="lower",
        extent=extent,
        aspect="auto",
        cmap="viridis",
        vmin=0.0,
        vmax=1.0,
    )
    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label("$R_{occ}$ (几何遮挡率)")

    if float(heat.max()) >= float(R_th):
        ax.contour(
            xs, ys, heat, levels=[float(R_th)], colors="red", linewidths=1.6, linestyles="--"
        )
        ax.plot([], [], color="red", ls="--", lw=1.6, label=f"$R_{{th}}$ = {R_th:g}")
        ax.legend(loc="upper right", fontsize=8)

    iy, ix = np.unravel_index(int(np.argmax(heat)), heat.shape)
    ax.plot(
        xs[ix], ys[iy], marker="x", color="white", markersize=7, markeredgewidth=1.6,
        label="_nolegend_",
    )
    ax.annotate(
        f"max $R_{{occ}}$ = {heat[iy, ix]:.3f}",
        (xs[ix], ys[iy]),
        textcoords="offset points",
        xytext=(10, -14),
        color="white",
        fontsize=8,
    )

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    fig.tight_layout()

    path = _ensure_parent(save_path)
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    return path


# ---------------------------------------------------------------------------
# R_occ(t) 曲线
# ---------------------------------------------------------------------------


def plot_R_occ_curves(
    t: Sequence[float],
    series: Sequence[Sequence[float]],
    labels: Sequence[str],
    R_th: float,
    save_path: Union[str, Path],
    title: str = "无人机遮挡率随时间变化",
) -> Path:
    """绘制多条 ``R_occ(t)`` 曲线，并画出 ``R_th`` 虚线。"""
    t_arr = np.asarray(t, dtype=float)
    if len(series) != len(labels):
        raise ValueError("series 与 labels 数量必须一致")

    fig, ax = plt.subplots(figsize=(8.2, 5.0))
    for values, label in zip(series, labels):
        ax.plot(t_arr, np.asarray(values, dtype=float), lw=1.6, label=label)
    ax.axhline(float(R_th), color="red", ls="--", lw=1.4, label=f"$R_{{th}}$ = {R_th:g}")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("$R_{occ}$")
    ax.set_ylim(-0.03, 1.05)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="upper left")
    ax.set_title(title)
    fig.tight_layout()

    path = _ensure_parent(save_path)
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    return path


# ---------------------------------------------------------------------------
# 3D 场景
# ---------------------------------------------------------------------------


def _shadow_polygon_3d(polygon_2d: BaseGeometry, plane_center, u, v) -> np.ndarray:
    """把平面局部二维多边形转换为世界三维顶点（取外环）。"""
    geom = polygon_2d
    if geom.geom_type == "MultiPolygon":
        geom = max(geom.geoms, key=lambda g: g.area)
    coords = np.asarray(geom.exterior.coords, dtype=float)
    return np.asarray([plane_local_to_world(c, plane_center, u, v) for c in coords], dtype=float)


def plot_3d_scene(
    C: Sequence[float],
    uav_center: Sequence[float],
    uav_size: Sequence[float],
    lamp: LampConfig,
    save_path: Union[str, Path],
    title: str = "3D 场景：灯 / 相机 / 无人机 / 阴影",
    pose=None,
    camera_cfg=None,
) -> Dict[str, object]:
    """绘制三维场景：灯盘、相机、无人机线框、LOS、阴影多边形与投影线。

    返回包含 ``R_occ``、``valid`` 等信息的小字典，便于日志记录。
    """
    cam = as_vector(C)
    center = as_vector(uav_center)
    size = as_vector(uav_size)
    plane_center = lamp.center_array
    plane_normal = lamp.normal_array
    u, v = plane_basis(plane_normal)
    r = lamp.radius_m

    result = compute_occlusion(cam, center, size, lamp, warn=False)

    fig = plt.figure(figsize=(8.6, 7.0))
    ax = fig.add_subplot(111, projection="3d")

    # --- 灯盘 -----------------------------------------------------------
    theta = np.linspace(0.0, 2.0 * np.pi, 181)
    disc = (
        plane_center
        + np.outer(np.cos(theta) * r, u)
        + np.outer(np.sin(theta) * r, v)
    )
    ax.plot(disc[:, 0], disc[:, 1], disc[:, 2], color="lime", lw=2.0)
    ax.add_collection3d(
        Poly3DCollection([disc], facecolor="lime", alpha=0.35, edgecolor="green")
    )
    ax.scatter(
        [plane_center[0]], [plane_center[1]], [plane_center[2]],
        color="green", marker="o", s=28, label="灯心",
    )

    # --- 灯平面参考框 ---------------------------------------------------
    half = 0.35
    rect = np.array(
        [
            plane_center + (-half) * u + (-half) * v,
            plane_center + (half) * u + (-half) * v,
            plane_center + (half) * u + (half) * v,
            plane_center + (-half) * u + (half) * v,
            plane_center + (-half) * u + (-half) * v,
        ]
    )
    ax.plot(rect[:, 0], rect[:, 1], rect[:, 2], color="gray", lw=0.8, alpha=0.6, ls=":")

    # --- 相机 -----------------------------------------------------------
    ax.scatter([cam[0]], [cam[1]], [cam[2]], color="red", marker="^", s=48, label="相机 C(t)")
    ax.plot(
        [cam[0], plane_center[0]], [cam[1], plane_center[1]], [cam[2], plane_center[2]],
        color="orange", lw=1.4, ls="-", label="LOS (相机 -> 灯)",
    )
    frustum_pts = None
    if pose is not None:
        # 光轴 = 弹道切线；同时画出视锥（水平/垂直 FOV）
        axis_len = max(0.30 * float(np.linalg.norm(cam - plane_center)), 0.05)
        tip = pose.position + axis_len * pose.forward
        ax.plot(
            [cam[0], tip[0]], [cam[1], tip[1]], [cam[2], tip[2]],
            color="crimson", lw=1.6, label="相机光轴（弹道切线）",
        )
        if camera_cfg is not None:
            from camera import frustum_corner_rays

            frustum_pts = frustum_corner_rays(pose, camera_cfg, axis_len)
            for cn in frustum_pts:
                ax.plot(
                    [cam[0], cn[0]], [cam[1], cn[1]], [cam[2], cn[2]],
                    color="crimson", lw=0.6, alpha=0.5,
                )
            loop = np.vstack([frustum_pts, frustum_pts[:1]])
            ax.plot(loop[:, 0], loop[:, 1], loop[:, 2], color="crimson", lw=0.9, alpha=0.7)

    # --- 无人机线框 -----------------------------------------------------
    corners = uav_corners(center, size)
    for i, j in uav_edges():
        ax.plot(
            [corners[i, 0], corners[j, 0]],
            [corners[i, 1], corners[j, 1]],
            [corners[i, 2], corners[j, 2]],
            color="blue", lw=1.4,
        )
    ax.scatter([center[0]], [center[1]], [center[2]], color="blue", s=18, label="无人机中心")

    # --- 阴影与投影线 ---------------------------------------------------
    info: Dict[str, object] = {"R_occ": result.R_occ, "valid": result.valid, "message": result.message}
    if result.shadow_polygon is not None and not result.shadow_polygon.is_empty:
        if result.shadow_polygon.geom_type in {"Polygon", "MultiPolygon"}:
            shadow3d = _shadow_polygon_3d(result.shadow_polygon, plane_center, u, v)
            ax.add_collection3d(
                Poly3DCollection(
                    [shadow3d], facecolor="red", alpha=0.35, edgecolor="darkred"
                )
            )
            ax.plot(
                shadow3d[:, 0], shadow3d[:, 1], shadow3d[:, 2],
                color="darkred", lw=1.2, label="阴影多边形",
            )
        if result.shadow_points_3d is not None:
            pts = np.asarray(result.shadow_points_3d, dtype=float)
            for corner, p in zip(corners, pts):
                ax.plot(
                    [corner[0], p[0]], [corner[1], p[1]], [corner[2], p[2]],
                    color="gray", lw=0.6, alpha=0.7,
                )
            ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], color="darkred", s=6)
            ax.plot([], [], color="gray", lw=0.6, label="顶点投影线")
    else:
        ax.text2D(
            0.02, 0.95,
            "投影退化：阴影不可用\n" + str(result.message),
            transform=ax.transAxes, color="red", fontsize=8,
        )

    # --- 坐标轴与视角 ---------------------------------------------------
    all_pts = np.vstack([corners, cam[None, :], plane_center[None, :], disc[::10]])
    if frustum_pts is not None:
        all_pts = np.vstack([all_pts, frustum_pts])
    lo = all_pts.min(axis=0)
    hi = all_pts.max(axis=0)
    span = np.maximum(hi - lo, 1e-3)
    pad = 0.15 * float(np.max(span))
    ax.set_xlim(lo[0] - pad, hi[0] + pad)
    ax.set_ylim(lo[1] - pad, hi[1] + pad)
    ax.set_zlim(lo[2] - pad, hi[2] + pad)
    ax.set_box_aspect((span[0] + 2 * pad, span[1] + 2 * pad, span[2] + 2 * pad))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    # 3D 默认刻度过密（几何尺度小），限制为每个轴最多 5 个主刻度。
    for setter in (ax.xaxis, ax.yaxis, ax.zaxis):
        setter.set_major_locator(MaxNLocator(nbins=5))
    ax.set_title(f"{title}\n$R_{{occ}}$ = {result.R_occ:.3f}   |   相机 = ({cam[0]:.2f}, {cam[1]:.2f}, {cam[2]:.2f}) m")
    ax.legend(fontsize=7, loc="upper left")
    fig.tight_layout()

    path = _ensure_parent(save_path)
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    info["path"] = str(path)
    return info


# ---------------------------------------------------------------------------
# 双面板场景动画（飞镖飞行 + 悬停无人机）
# ---------------------------------------------------------------------------


def fixed_axis_limits(
    points: np.ndarray,
    pad_ratio: float = 0.08,
    min_span: float = 0.5,
) -> List[Tuple[float, float]]:
    """由点集一次性算出**固定**的三轴范围（动画全程不再变化）。

    ``min_span`` 保证任一轴至少有该跨度，避免细长场景被压扁成一条线。
    """
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] == 0:
        raise ValueError(f"points 形状非法：{pts.shape}")
    lo = pts.min(axis=0).astype(float)
    hi = pts.max(axis=0).astype(float)
    pad = pad_ratio * float(np.max(hi - lo)) + 1e-6
    lo, hi = lo - pad, hi + pad
    for i in range(3):
        if hi[i] - lo[i] < min_span:
            mid = 0.5 * (lo[i] + hi[i])
            lo[i], hi[i] = mid - min_span / 2.0, mid + min_span / 2.0
    return [(float(lo[i]), float(hi[i])) for i in range(3)]


def apply_axis_limits(
    ax,
    limits: Sequence[Tuple[float, float]],
    equal_box: bool = True,
    n_ticks: int = 5,
    label_size: float = 8.0,
    aspect_scale: Sequence[float] = (1.0, 1.0, 1.0),
    zoom: float = 1.0,
) -> None:
    """给 3D 轴施加固定范围，并按数据跨度设置 box aspect（避免几何形变）。"""
    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    ax.set_zlim(*limits[2])
    if equal_box:
        scales = list(aspect_scale) + [1.0] * 3
        ax.set_box_aspect(
            [max(hi - lo, 1e-3) * float(scales[i]) for i, (lo, hi) in enumerate(limits)],
            zoom=float(zoom),
        )
    # 细长场景下 3D 默认刻度过密，限制主刻度数量，避免标签互相压叠。
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_major_locator(MaxNLocator(nbins=int(n_ticks)))
    ax.tick_params(labelsize=label_size, pad=1)


def build_scene_animation(
    result: Dict[str, np.ndarray],
    lamp: LampConfig,
    uav_size_m: Sequence[float],
    full_path: Optional[np.ndarray] = None,
    title_prefix: str = "",
    figsize: Tuple[float, float] = (13.4, 9.6),
    zoom_half_width_m: Optional[float] = None,
    r_th: float = 0.7,
    dart_cfg=None,
    camera=None,
    simple_layout: bool = False,
) -> Tuple[object, Callable[[int], tuple], int]:
    """构造上下两层场景动画。

    * **上层（横跨整幅）**：全程 3D 弹道，``+X`` 横向铺开，一眼看到 25 m 全流程；
    * **下层左**：灯区放大（灯盘 / 悬停无人机 / 阴影多边形）；
    * **下层右**：飞镖相机第一人称画面（光轴 = 弹道切线；绿=灯盘投影、
      蓝=无人机轮廓、红=被遮住的灯面积）。

    三个面板的坐标范围在整个动画中**固定不变**，因此飞镖是真实在飞行，
    而灯盘 / 悬停无人机 / 阴影保持相对静止，便于判断遮挡是否成立。

    参数
    ----
    result : ``run_dynamic`` 的返回值（``t, R, P_d, uav, v, loss``）
    full_path : 可选，完整弹道（含尚未飞到的下降段），用于绘制虚线弹道与落点
    zoom_half_width_m : 放大面板半宽，默认按机身尺寸自适应

    返回 ``(fig, update, n_frames)``。
    """
    t = np.asarray(result["t"], dtype=float)
    P_d = np.asarray(result["P_d"], dtype=float)
    uav = np.asarray(result["uav"], dtype=float)
    R = np.asarray(result["R"], dtype=float)
    n = int(t.size)

    center = lamp.center_array
    u_basis, v_basis = plane_basis(lamp.normal_array)
    r_lamp = lamp.radius_m
    theta = np.linspace(0.0, 2.0 * np.pi, 121)
    disc = (
        center
        + np.outer(np.cos(theta) * r_lamp, u_basis)
        + np.outer(np.sin(theta) * r_lamp, v_basis)
    )

    path = np.asarray(full_path, dtype=float) if full_path is not None else P_d

    # 沿弹道的“光轴梳齿”：在每个采样点画出该时刻的弹道切线方向，
    # 一眼看出相机光轴是**随弹道实时变化**的（爬升段朝上、顶点转平、末端俯冲）。
    comb_segments = []
    if dart_cfg is not None:
        from motion import dart_camera_trajectory, dart_camera_velocity

        comb_len = 0.10 * float(np.max(np.linalg.norm(path - center, axis=1)))
        for tc in np.linspace(0.0, float(t[-1]), 21):
            try:
                p_c = dart_camera_trajectory(float(tc), dart_cfg)
                v_c = dart_camera_velocity(float(tc), dart_cfg)
            except Exception:
                continue
            nrm = float(np.linalg.norm(v_c))
            if nrm <= 1e-9:
                continue
            comb_segments.append((p_c, p_c + comb_len * v_c / nrm))

    # ---- 固定视域：只计算一次，动画全程不变 -----------------------------
    wide_limits = fixed_axis_limits(np.vstack([path, uav, disc, center[None, :]]))
    size_arr = np.asarray(uav_size_m, dtype=float)
    # 放大窗口以“机体 + 灯心”的中点为心，保证机体和灯盘都在窗口内（否则机体会被切一半）
    # 构图与静态 3D 场景一致：相机 -> 机体 -> 灯 都在窗口内（沿来向展开）
    approach = np.asarray(P_d[0], dtype=float) - center
    approach_len = float(np.linalg.norm(approach))
    approach_dir = approach / approach_len if approach_len > 1e-9 else np.array([-1.0, 0.0, 0.0])
    zoom_half = float(zoom_half_width_m) if zoom_half_width_m else 0.55
    uav_mid = center + 0.55 * zoom_half * approach_dir
    zoom_limits = [
        (uav_mid[0] - zoom_half, uav_mid[0] + zoom_half),
        (uav_mid[1] - zoom_half, uav_mid[1] + zoom_half),
        (uav_mid[2] - zoom_half, uav_mid[2] + zoom_half),
    ]

    n_panels = 3 if (dart_cfg is not None and camera is not None) else 2
    fig = plt.figure(figsize=figsize)
    if simple_layout and dart_cfg is not None and camera is not None:
        # 最简版：上=全程弹道（3D 横铺），下=灯区放大（遮挡效果）；不含第一人称画面
        grid = fig.add_gridspec(
            2, 1, height_ratios=(1.3, 1.0), hspace=0.12,
            left=0.05, right=0.95, top=0.92, bottom=0.05,
        )
        ax_wide = fig.add_subplot(grid[0, 0], projection="3d")   # 上：全程弹道
        ax_zoom = fig.add_subplot(grid[1, 0], projection="3d")   # 下：灯区放大（遮挡）
        ax_fpv = None
    elif n_panels == 3:
        grid = fig.add_gridspec(
            2, 2, height_ratios=(0.72, 1.0), hspace=0.12, wspace=0.06,
            left=0.015, right=0.985, top=0.90, bottom=0.035,
        )
        ax_wide = fig.add_subplot(grid[0, :], projection="3d")   # 上层：全程弹道
        ax_zoom = fig.add_subplot(grid[1, 0], projection="3d")   # 下层左：灯区放大
        ax_fpv = fig.add_subplot(grid[1, 1])                     # 下层右：第一人称
    else:
        ax_wide = fig.add_subplot(1, 2, 1, projection="3d")
        ax_zoom = fig.add_subplot(1, 2, 2, projection="3d")
        ax_fpv = None
    # 视角：上层把弹道所在平面摆成“横向展开”（X 向右、Z 向上），能一眼看到全程；
    # 下层左接近灯平面正视（看得清灯盘、机体与阴影的覆盖关系）。
    ax_wide.view_init(elev=18, azim=-88)
    # 灯区放大：从相机一侧偏上看，能同时看到机体、灯盘和阴影多边形
    # 灯区放大：抬到 30° 俯视，避免立方体被压扁成“长条”（z 向投影被压缩）
    # 等轴测视角：立方体三个面都能看到，不会被压成“长条”
    ax_zoom.view_init(elev=22, azim=-105)
    # 全程弹道用正交投影：25 m 细长盒两端不再被透视压缩，全程等比例可见
    ax_wide.set_proj_type("ortho")

    ref = 1.2 * zoom_half
    rect_local = [(-ref, -ref), (ref, -ref), (ref, ref), (-ref, ref), (-ref, -ref)]
    rect = np.asarray([center + a * u_basis + b * v_basis for a, b in rect_local], dtype=float)
    # 放大面板的裁剪框（灯平面局部坐标）：阴影很大时只画落在视域内的部分，
    # 否则 k 很大（相机很远）时阴影多边形会溢出面板。
    zoom_cube = uav_corners(center, (2.0 * zoom_half, 2.0 * zoom_half, 2.0 * zoom_half))
    local_pts = np.asarray(
        [plane_local_coords(p, center, u_basis, v_basis) for p in zoom_cube], dtype=float
    )
    zoom_clip_box = shapely_box(
        float(local_pts[:, 0].min()), float(local_pts[:, 1].min()),
        float(local_pts[:, 0].max()), float(local_pts[:, 1].max()),
    )

    def update(k: int):
        C = P_d[k]
        U = uav[k]
        res = compute_occlusion(C, U, uav_size_m, lamp, warn=False)

        # ---------------- 左：全程视角 ----------------
        ax_wide.clear()
        # 光轴梳齿（静态叠加，展示光轴沿弹道的实时转向）
        for p_a, p_b in comb_segments:
            ax_wide.plot(
                [p_a[0], p_b[0]], [p_a[1], p_b[1]], [p_a[2], p_b[2]],
                color="crimson", lw=0.9, alpha=0.45, zorder=1,
            )
        ax_wide.plot(path[:, 0], path[:, 1], path[:, 2],
                     color="gray", lw=1.0, ls=":", label="弹道（全程）")
        # 明确标出“起始位置”与“末端位置”，并用竖直虚线锚定
        p_start, p_end = path[0], path[-1]
        ax_wide.scatter([p_start[0]], [p_start[1]], [p_start[2]],
                        color="black", marker="o", s=45, zorder=7, label="起始位置（发射点）")
        ax_wide.scatter([p_end[0]], [p_end[1]], [p_end[2]],
                        color="darkgreen", marker="X", s=95, zorder=7, label="末端位置（灯心）")
        for p_anchor in (p_start, p_end):
            ax_wide.plot(
                [p_anchor[0], p_anchor[0]], [p_anchor[1], p_anchor[1]],
                [wide_limits[2][0], wide_limits[2][1]],
                color="dimgray", lw=0.8, ls=":", alpha=0.8, zorder=1,
            )
        # 每 0.5 s 打一个点，便于在“全程视角”里读出进度
        step_marks = max(1, int(round(path.shape[0] / max(float(t[-1]), 1e-6) * 0.5)))
        ax_wide.scatter(
            path[::step_marks, 0], path[::step_marks, 1], path[::step_marks, 2],
            s=7, color="dimgray", alpha=0.75, depthshade=False, label="0.5 s 间隔",
        )
        if k > 0:
            ax_wide.plot(
                P_d[: k + 1, 0], P_d[: k + 1, 1], P_d[: k + 1, 2],
                color="orange", lw=2.0, label="已飞过",
            )
        ax_wide.plot(disc[:, 0], disc[:, 1], disc[:, 2], color="lime", lw=2.0)
        ax_wide.scatter([center[0]], [center[1]], [center[2]],
                        color="green", marker="*", s=90, label="绿灯（落点）")
        corners_w = uav_corners(U, uav_size_m)
        for i, j in uav_edges():
            ax_wide.plot(
                [corners_w[i, 0], corners_w[j, 0]],
                [corners_w[i, 1], corners_w[j, 1]],
                [corners_w[i, 2], corners_w[j, 2]],
                color="blue", lw=1.0,
            )
        ax_wide.scatter([C[0]], [C[1]], [C[2]], color="red", marker="^", s=70, label="飞镖相机")
        ax_wide.plot(
            [C[0], center[0]], [C[1], center[1]], [C[2], center[2]],
            color="orange", lw=1.0, ls="--", alpha=0.8, label="LOS",
        )
        pose = None
        if dart_cfg is not None:
            pose = camera_pose(float(t[k]), dart_cfg)
            # 光轴 + 视锥必须**完整画在盒子里**：长度按盒子的横向尺度反算，避免被裁掉
            span_y = float(wide_limits[1][1] - wide_limits[1][0])
            span_z = float(wide_limits[2][1] - wide_limits[2][0])
            half_ang = float(np.tan(np.radians(camera.fov_h_deg / 2.0))) if camera is not None else 0.7
            axis_len = min(
                0.30 * float(np.linalg.norm(path.max(axis=0) - path.min(axis=0))),
                0.42 * min(span_y, span_z) / max(half_ang, 1e-6),
            )
            axis_len = max(axis_len, 0.5)
            tip = pose.position + axis_len * pose.forward
            ax_wide.plot(
                [C[0], tip[0]], [C[1], tip[1]], [C[2], tip[2]],
                color="red", lw=2.6, label="相机光轴（= 弹道切线）", zorder=6,
            )
            if camera is not None:
                from camera import frustum_corner_rays

                for cn in frustum_corner_rays(pose, camera, axis_len):
                    ax_wide.plot(
                        [C[0], cn[0]], [C[1], cn[1]], [C[2], cn[2]],
                        color="crimson", lw=0.5, alpha=0.45,
                    )
        # 25 m 弹道的竖直起伏只有 ~1 m，若不放大在宽幅面板里几乎是一条直线；
        # 这里只对**显示**做 Z 向放大（标题中标注），几何计算不受影响。
        apply_axis_limits(
            ax_wide, wide_limits, n_ticks=4, label_size=7.5,
            aspect_scale=(1.0, 0.35, 2.4), zoom=1.0,
        )
        ax_wide.set_xlabel("x [m]")
        ax_wide.set_ylabel("y [m]")
        ax_wide.set_zlabel("z [m]")
        v_now = np.asarray(result["v"][k], dtype=float) if "v" in result else None
        if v_now is not None and float(np.linalg.norm(v_now)) > 1e-9:
            v_unit = v_now / float(np.linalg.norm(v_now))
            pitch = float(np.degrees(np.arcsin(np.clip(v_unit[2], -1.0, 1.0))))
            head = float(np.degrees(np.arctan2(v_unit[1], v_unit[0])))
            angle_txt = f"  光轴: 俯仰 {pitch:+.1f}°, 航向 {head:+.1f}°"
        else:
            angle_txt = ""
        ax_wide.set_title(
            f"全程弹道视角（真实比例细长长方体）   t = {t[k]:.2f} s{angle_txt}",
            fontsize=9,
        )
        ax_wide.legend(fontsize=6.5, loc="upper left", framealpha=0.85)

        # ---------------- 右：灯区放大 ----------------
        ax_zoom.clear()
        ax_zoom.plot(rect[:, 0], rect[:, 1], rect[:, 2], color="gray", lw=0.8, ls=":")
        from camera import frustum_corner_rays
        ax_zoom.scatter([C[0]], [C[1]], [C[2]], color="red", marker="^", s=70,
                        depthshade=False, zorder=8, label="camera C(t)")
        ax_zoom.plot([C[0], center[0]], [C[1], center[1]], [C[2], center[2]],
                     color="orange", lw=1.6, alpha=0.9, zorder=7, label="LOS")
        if pose is not None and camera is not None:
            flen = float(np.linalg.norm(C - center)) * 1.05
            flen = min(max(flen, 0.3), 1.2)
            for cn in frustum_corner_rays(pose, camera, flen):
                ax_zoom.plot([C[0], cn[0]], [C[1], cn[1]], [C[2], cn[2]],
                             color="crimson", lw=0.7, alpha=0.45, zorder=5)
        ax_zoom.plot(disc[:, 0], disc[:, 1], disc[:, 2], color="lime", lw=3.0)
        ax_zoom.add_collection3d(
            Poly3DCollection([disc], facecolor="lime", alpha=0.55, edgecolor="green", lw=1.5)
        )
        corners = uav_corners(U, uav_size_m)
        for i, j in uav_edges():
            ax_zoom.plot([corners[i, 0], corners[j, 0]], [corners[i, 1], corners[j, 1]],
                         [corners[i, 2], corners[j, 2]], color="blue", lw=2.2)
        ax_zoom.scatter([U[0]], [U[1]], [U[2]], color="blue", s=30,
                        depthshade=False, zorder=6, label="UAV")
        if res.shadow_polygon is not None and res.shadow_polygon.geom_type in {"Polygon", "MultiPolygon"}:
            clipped = res.shadow_polygon.intersection(zoom_clip_box)
            parts = [clipped] if clipped.geom_type == "Polygon" else list(getattr(clipped, "geoms", []))
            for part in parts:
                if part.is_empty or part.geom_type != "Polygon":
                    continue
                coords = np.asarray(part.exterior.coords, dtype=float)
                shadow3d = np.asarray(
                    [plane_local_to_world(c, center, u_basis, v_basis) for c in coords], dtype=float
                )
                ax_zoom.add_collection3d(
                    Poly3DCollection([shadow3d], facecolor="red", alpha=0.35, edgecolor="darkred")
                )
            if res.shadow_points_3d is not None:
                for corner, pp in zip(corners, np.asarray(res.shadow_points_3d, dtype=float)):
                    ax_zoom.plot([corner[0], pp[0]], [corner[1], pp[1]], [corner[2], pp[2]],
                                 color="gray", lw=0.7, alpha=0.65, zorder=2)
                pts3 = np.asarray(res.shadow_points_3d, dtype=float)
                ax_zoom.scatter(pts3[:, 0], pts3[:, 1], pts3[:, 2], color="darkred", s=8,
                                depthshade=False, zorder=3)
        # 加密点位：灯盘圆周采样点 + 阴影边界等弧长采样点，便于看清遮挡范围
        theta_pts = np.linspace(0.0, 2.0 * np.pi, 41, endpoint=False)
        lamp_pts = (
            center
            + np.outer(np.cos(theta_pts) * r_lamp, u_basis)
            + np.outer(np.sin(theta_pts) * r_lamp, v_basis)
        )
        ax_zoom.scatter(lamp_pts[:, 0], lamp_pts[:, 1], lamp_pts[:, 2],
                        color="lime", s=5, depthshade=False, zorder=8)
        if res.shadow_polygon is not None and res.shadow_polygon.geom_type in {"Polygon", "MultiPolygon"}:
            _clip = res.shadow_polygon.intersection(zoom_clip_box)
            if not _clip.is_empty and _clip.geom_type == "Polygon":
                _per = float(_clip.exterior.length)
                if _per > 1e-9:
                    _ts = np.linspace(0.0, _per, 49, endpoint=False)
                    _pts2 = np.asarray(
                        [_clip.exterior.interpolate(float(tt)).coords[0] for tt in _ts], dtype=float
                    )
                    _pts3 = np.asarray(
                        [plane_local_to_world(c, center, u_basis, v_basis) for c in _pts2], dtype=float
                    )
                    ax_zoom.scatter(_pts3[:, 0], _pts3[:, 1], _pts3[:, 2],
                                    color="darkred", s=6, depthshade=False, zorder=4)
        ax_zoom.scatter([center[0]], [center[1]], [center[2]], color="green", marker="o", s=40,
                        edgecolors="darkgreen", depthshade=False, zorder=9, label="lamp")
        if pose is not None:
            axis_len_zoom = 1.4 * zoom_half
            tip_zoom = pose.position + axis_len_zoom * pose.forward
            ax_zoom.plot([C[0], tip_zoom[0]], [C[1], tip_zoom[1]], [C[2], tip_zoom[2]],
                         color="crimson", lw=2.0, zorder=6, label="optical axis")
        apply_axis_limits(ax_zoom, zoom_limits, n_ticks=3, label_size=7.5, zoom=1.0)
        ax_zoom.set_xlabel("x [m]")
        ax_zoom.set_ylabel("y [m]")
        ax_zoom.set_zlabel("z [m]")
        ax_zoom.set_title(f"灯区放大   $R_{{occ}}$ = {R[k]:.3f}")
        ax_zoom.legend(fontsize=6, loc="upper right", framealpha=0.85)

        state = "有效遮挡" if R[k] >= float(r_th) else "遮挡不足"
        if ax_fpv is not None and pose is not None:
            draw_camera_view(
                ax_fpv, pose, camera, lamp, U, uav_size_m,
                R_occ=float(R[k]),
                margin_deg=fov_margin_deg(pose, lamp.center_array, camera),
                time_s=float(t[k]),
            )
        fig.suptitle(
            f"{title_prefix}  |  t = {t[k]:.2f} s  |  无人机 = ({U[0]:.2f}, {U[1]:.2f}, {U[2]:.2f}) m"
            f"  |  飞镖 = ({C[0]:.2f}, {C[1]:.2f}, {C[2]:.2f}) m"
            f"  |  $R_{{occ}}$ = {R[k]:.3f}（{state}）"
        )
        return ()

    return fig, update, n


# ---------------------------------------------------------------------------
# 悬停站位距离扫描
# ---------------------------------------------------------------------------


def plot_hover_standoff_scan(
    scan,
    R_th: float,
    save_path: Union[str, Path],
    current_standoff_m: Optional[float] = None,
    title: str = "悬停站位距离扫描（无人机不机动）",
) -> Path:
    """绘制“站位到灯平面距离”与遮挡率的关系，并标出全程有效遮挡区间。

    ``scan`` 为 :func:`experiment.hover_standoff_scan` 返回的 DataFrame。
    """
    import pandas as pd  # 仅用于类型判断的轻量依赖

    if not isinstance(scan, pd.DataFrame) or scan.empty:
        raise ValueError("scan 必须是非空 DataFrame")

    x = np.asarray(scan["standoff_m"], dtype=float)
    r_min = np.asarray(scan["R_min"], dtype=float)
    r_mean = np.asarray(scan["R_mean"], dtype=float)
    effective = np.asarray(scan["effective"], dtype=bool)

    fig, ax = plt.subplots(figsize=(8.6, 5.0))
    # 全程有效遮挡区间
    if bool(np.any(effective)):
        ax.axvspan(float(x[effective].min()), float(x[effective].max()),
                   color="green", alpha=0.10, label="全程有效遮挡区间")

    ax.plot(x, r_mean, "-o", color="tab:blue", ms=4, lw=1.6, label="$R_{occ}$ 均值")
    ax.plot(x, r_min, "-s", color="tab:orange", ms=4, lw=1.6, label="$R_{occ}$ 最小值")
    ax.axhline(float(R_th), color="red", ls="--", lw=1.4, label=f"$R_{{th}}$ = {R_th:g}")

    if current_standoff_m is not None:
        ax.axvline(float(current_standoff_m), color="black", ls=":", lw=1.4,
                   label=f"当前站位 {current_standoff_m:g} m")

    ax.set_xlabel("站位到灯平面的距离 standoff [m]")
    ax.set_ylabel("$R_{occ}$")
    ax.set_ylim(-0.03, 1.05)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="lower right")
    ax.set_title(title)

    # 顶部副坐标轴：换算成“到灯心的直线距离”
    lateral = float(np.hypot(float(scan["offset_u_m"].iloc[0]), float(scan["offset_v_m"].iloc[0])))

    def _to_distance(standoff):
        return np.sqrt(np.asarray(standoff, dtype=float) ** 2 + lateral ** 2)

    def _to_standoff(distance):
        return np.sqrt(np.maximum(np.asarray(distance, dtype=float) ** 2 - lateral ** 2, 0.0))

    secax = ax.secondary_xaxis("top", functions=(_to_distance, _to_standoff))
    secax.set_xlabel("站位到灯心的直线距离 [m]")

    fig.tight_layout()
    path = _ensure_parent(save_path)
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    return path


# ---------------------------------------------------------------------------
# 第一人称相机视角（光轴 = 弹道切线方向）
# ---------------------------------------------------------------------------


def _fill_shapely(ax, geom, facecolor, edgecolor, alpha=0.45, lw=1.2, zorder=3) -> None:
    """把 shapely 面状几何填充到 2D 轴上（支持 Polygon / MultiPolygon / 集合）。"""
    if geom is None or getattr(geom, "is_empty", True):
        return
    if geom.geom_type == "Polygon":
        parts = [geom]
    elif geom.geom_type == "MultiPolygon":
        parts = list(geom.geoms)
    elif hasattr(geom, "geoms"):
        parts = [g for g in geom.geoms if getattr(g, "geom_type", "") == "Polygon"]
    else:
        parts = []
    for poly in parts:
        xy = np.asarray(poly.exterior.coords, dtype=float)
        ax.add_patch(
            MplPolygon(
                xy, closed=True, facecolor=facecolor, edgecolor=edgecolor,
                alpha=alpha, lw=lw, zorder=zorder,
            )
        )


def draw_camera_view(
    ax,
    pose,
    camera,
    lamp: LampConfig,
    uav_center: Sequence[float],
    uav_size: Sequence[float],
    R_occ: float = float("nan"),
    margin_deg: Optional[float] = None,
    time_s: Optional[float] = None,
) -> float:
    """绘制**飞镖相机第一人称画面**（像平面）。

    绿色 = 灯盘投影；蓝色 = 无人机轮廓投影；红色 = 两者交集（即被遮住的灯面积）。
    返回像面遮挡比。
    """
    width, height = camera.resolution
    ratio, lamp_poly, uav_poly = image_space_occlusion(
        pose, lamp, uav_center, uav_size, camera
    )

    ax.clear()
    ax.set_facecolor("#101014")
    ax.add_patch(
        Rectangle((0, 0), width, height, facecolor="#101014", edgecolor="gray", lw=1.0, zorder=1)
    )
    # 主点（光轴中心）十字线
    ax.plot([width / 2.0, width / 2.0], [0, height], color="white", lw=0.5, alpha=0.25, zorder=2)
    ax.plot([0, width], [height / 2.0, height / 2.0], color="white", lw=0.5, alpha=0.25, zorder=2)
    _fill_shapely(ax, lamp_poly, "#3ddc84", "lime", alpha=0.55, zorder=3)
    _fill_shapely(ax, uav_poly, "#4c7dff", "blue", alpha=0.40, zorder=4)
    if lamp_poly is not None and uav_poly is not None:
        _fill_shapely(ax, lamp_poly.intersection(uav_poly), "red", "darkred", alpha=0.75, zorder=5)

    ax.set_xlim(0, width)
    ax.set_ylim(height, 0)  # 图像坐标：原点在左上
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    if lamp_poly is None:
        ax.text(width / 2.0, height / 2.0, "灯不在视场内", color="white",
                ha="center", va="center", fontsize=9)

    # 主视图自动放大到灯盘附近：25 m 外整幅画面里灯只有约 1 px，直接看整幅什么也看不出。
    # 右下角逐图给出全幅画面（含放大框），便于同时判断“在画面哪里”和“是否被遮住”。
    lamp_px = float("nan")
    if lamp_poly is not None:
        minx, miny, maxx, maxy = lamp_poly.bounds
        lamp_px = max(maxx - minx, maxy - miny)
        cx_px, cy_px = 0.5 * (minx + maxx), 0.5 * (miny + maxy)
        span = max(5.0 * lamp_px, 40.0)
        half = span / 2.0
        ax.set_xlim(cx_px - half, cx_px + half)
        ax.set_ylim(cy_px + half, cy_px - half)
        ax.set_title(ax.get_title() + f"  |  画面放大 {span / max(lamp_px, 1e-6):.0f}×", fontsize=7.5)

    line1 = []
    if time_s is not None:
        line1.append(f"t = {time_s:.2f} s")
    if np.isfinite(R_occ):
        line1.append(f"$R_{{occ}}$ = {R_occ:.3f}")
    line2 = [f"像面遮挡 {ratio:.3f}"]
    if np.isfinite(lamp_px):
        line2.append(f"灯 Ø {lamp_px:.1f} px")
    if margin_deg is not None:
        line2.append(f"视场余量 {margin_deg:.1f}°")
    ax.set_title("  ".join(line1) + "\n" + " | ".join(line2), fontsize=7.5)
    return float(ratio)


# ---------------------------------------------------------------------------
# 单张融合 3D 视图：分段非线性 x 轴（25 m 全程 + 末端细节同图）
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# 双视频：A 全程弹道（线性 x 轴）/ B 末端细化
# ---------------------------------------------------------------------------

# ---- 调参：两段视频的公共规格（改这里会影响 A/B 两段视频）------------------------
# 帧率 [fps]：prompt 锁定 30 fps；帧数 = round(t_end·fps)。
#   ⚠ t_end 是**截断后**的时长（main.py 先 truncate_run_before_contact）：round(1.86·30) = **56 帧**，
#     不是整条弹道 1.92 s 对应的 58 帧；调 DART_STOP_DISTANCE_M 会连带改变帧数。
#   调小 → 帧少、动作更"跳"；调大 → 更顺但渲染更慢（CLI 可用 --fps 临时覆盖）。
ANIM_FPS_DEFAULT: int = 30
# 画布尺寸 [inch]：19.2×10.8 配合 dpi=100 → 1920×1080（CLI --video-dpi 可覆盖）。
#   分辨率 = figsize × dpi；改 figsize 会改变 3D 轴的有效像素密度与标注密度。
ANIM_FIGSIZE: Tuple[float, float] = (19.2, 10.8)   # 1920x1080 @ dpi=100

# ---- 调参（★）：视频 A 的"拉长 + 铺满画布"参数 ------------------------------
# 画面有多满由三个互不干扰的旋钮决定，想改观感先看这三个：
#   ① OVERVIEW_RECT   轴在画布里的缩放：横纵**同时**放大/缩小（"拉长"就调它）。
#   ② OVERVIEW_Z_BOX  盒子形状：z 越大越"高瘦"、越小越"扁平修长"（改它会改变宽高比）。
#   ③ OVERVIEW_*LIM   数据范围：只影响弹道在盒子里占多满，不影响盒子在画布上的大小。
#
# OVERVIEW_RECT：3D 轴在画布里的位置与大小（figure 分数 [left, bottom, w, h]）。
#   允许负值与 >1 让轴"外扩"以放大内容；外扩后标题/图例必须用画布层 API（见下）。
#   ★ "拉长一点"就调这里：它是等比放大，长度与高度同时增长。
#   实测非白像素占比（配合 z=13.5，1920×1080 画布，边距 = 左/右/上/下 %）：
#     1.68 → 93.5%×95.1%（1.1/5.4/0.4/4.5）   ← 旧值，右侧还空一条
#     1.76 → 95.7%×97.5%（1.1/3.2/0.0/2.5）
#     1.80 → 96.7%×98.4%（1.1/2.2/0.0/1.6）
#     1.82 → 97.5%×98.9%（0.8/1.7/0.0/1.1）   ← 当前值：等比拉长后仍留 ~1~2% 呼吸位
#     1.84 → 98.4%×99.5%（0.4/1.1/0.0/0.5）   ← 已贴边
#   ⚠ 上限：再往上就会把红色视锥、x 轴刻度数字切到画布外（画布就 1920×1080 那么大）。
OVERVIEW_RECT: Tuple[float, float, float, float] = (-0.41, -0.41, 1.82, 1.82)
# OVERVIEW_Z_BOX：z 轴在盒子里的视觉长度（x=26、y=4 固定）。
#   它与 elev 一起决定投影宽高比：宽高比 = 26 / (y·sin(elev) + z·cos(elev))。
#   实测投影宽高比：z=10→2.20、12→1.90、13→1.78、13.5→1.80、14→1.68、16→1.51。
#   画布是 1920×1080 = 1.78，所以 **z≈13.5 时横纵同时铺满**（比例最接近真实、无额外压扁）；
#   增大 z（如 16）会让弹道更陡更高，但横向只剩 75%（需要更大的 rect 才铺满，且更"夸张"）；
#   减小 z（如 11）会让弹道更"长扁"，但纵向只剩 ~90%（上下会空出白边）。
OVERVIEW_Z_BOX: float = 13.5
# OVERVIEW_XLIM / YLIM / ZLIM：视频 A 的数据范围（★ 弹道"占满盒子"就靠这里）。
#   盒子在画布上的大小只由 ①+② 决定；这里的范围收得越紧，弹道在盒子里占得越满、越"长"。
#   取值 = 弹道真实包络 + 少量余量（包络：x −25.04→0、y −2.87→0、z −0.57→2.65）。
#   收窄后 x 占盒子 97.8%（旧值 96.3%）、y 占 84.4%（旧值 71.7%）、z 占 84.8%（旧值 76.7%）。
#   ⚠ 不要比包络还小：起点/终点会贴到盒子侧壁上（mplot3d 不裁切，但看着像"贴墙"）。
OVERVIEW_XLIM: Tuple[float, float] = (-25.30, 0.30)
OVERVIEW_YLIM: Tuple[float, float] = (-3.10, 0.30)
OVERVIEW_ZLIM: Tuple[float, float] = (-0.90, 2.90)
# OVERVIEW_FRUSTUM_LEN_M：视频 A 里相机视锥的角射线长度 [m]（视场可视化，不参与遮挡计算）。
#   ⚠ 它是"画布溢出"的唯一来源：rect>1.7 时，顶点附近的射线会顶出画布顶部、压住标题。
#   解决办法不是缩短它，而是打开下面的 OVERVIEW_FRUSTUM_CLIP（射线画到盒子壁为止）。
OVERVIEW_FRUSTUM_LEN_M: float = 1.5
# OVERVIEW_FRUSTUM_CLIP：是否把视锥射线裁剪到 OVERVIEW_*LIM 盒内。
#   True  → 射线止于盒子壁：画布永不被切边、标题不被压（推荐，配合 rect≥1.76 铺满）。
#   False → 恢复"射线伸出盒子"的旧观感（rect 需 ≤1.70，否则顶部会被画布切掉）。
OVERVIEW_FRUSTUM_CLIP: bool = True


def frame_indices(n_sim: int, n_frames: int) -> np.ndarray:
    """把 n_frames 输出帧重采样到 n_sim 仿真帧（首尾对齐）。"""
    if n_frames <= 1:
        return np.zeros(1, dtype=int)
    k = np.arange(int(n_frames), dtype=float)
    idx = np.round(k * (int(n_sim) - 1) / float(int(n_frames) - 1)).astype(int)
    return np.clip(idx, 0, int(n_sim) - 1)


def _frame_count(t_end: float, fps: int = ANIM_FPS_DEFAULT) -> int:
    """1.92 s @ 30 fps -> 58 帧。"""
    return max(2, int(round(float(t_end) * float(fps))))


def clip_segment_to_box(
    origin: Sequence[float],
    tip: Sequence[float],
    xlim: Sequence[float],
    ylim: Sequence[float],
    zlim: Sequence[float],
) -> np.ndarray:
    """把线段 origin→tip 裁剪到坐标盒（xlim/ylim/zlim）内，返回裁剪后的端点。

    视频 A 的视锥角射线默认长度 1.5 m，在弹道顶点附近会伸出盒子、顶出画布并压住
    标题；这里直接按盒子裁剪（射线画到盒壁为止）。origin 在盒外时返回 origin。
    """
    p0 = np.asarray(origin, dtype=float)
    d = np.asarray(tip, dtype=float) - p0
    length = float(np.linalg.norm(d))
    if length < 1e-12:
        return np.asarray(tip, dtype=float)
    u = d / length
    t_max = length
    for o, ui, (lo, hi) in zip(p0, u, (xlim, ylim, zlim)):
        if ui > 1e-12:
            t_max = min(t_max, (float(hi) - float(o)) / float(ui))
        elif ui < -1e-12:
            t_max = min(t_max, (float(lo) - float(o)) / float(ui))
    return p0 + u * max(0.0, min(t_max, length))


def build_overview_animation(
    result: Dict[str, np.ndarray],
    lamp: LampConfig,
    uav_size_m: Sequence[float],
    full_path: Optional[np.ndarray] = None,
    title_prefix: str = "",
    figsize: Tuple[float, float] = ANIM_FIGSIZE,
    r_th: float = 0.7,
    dart_cfg=None,
    camera=None,
    fps: int = ANIM_FPS_DEFAULT,
) -> Tuple[object, Callable[[int], tuple], int]:
    """视频 A：全程 25 m 弹道，线性 x 轴，侧视偏转 + 正交投影。"""
    from camera import camera_pose, frustum_corner_rays

    t = np.asarray(result["t"], dtype=float)
    P_d = np.asarray(result["P_d"], dtype=float)
    uav = np.asarray(result["uav"], dtype=float)
    R = np.asarray(result["R"], dtype=float)
    n_sim = int(t.size)
    n_frames = _frame_count(float(t[-1]), fps)
    idx_map = frame_indices(n_sim, n_frames)
    path = np.asarray(full_path, dtype=float) if full_path is not None else P_d
    center = lamp.center_array
    u_basis, v_basis = plane_basis(lamp.normal_array)
    r_lamp = lamp.radius_m
    theta = np.linspace(0.0, 2.0 * np.pi, 121)
    disc = center + np.outer(np.cos(theta) * r_lamp, u_basis) + np.outer(
        np.sin(theta) * r_lamp, v_basis
    )
    mark_every = max(1, int(round(path.shape[0] / max(float(t[-1]), 1e-6) * 0.5)))
    pt_marks = path[::mark_every]

    # 光轴梳齿：沿弹道每约 1.2 m 一条 0.3 m 短切线
    comb = []
    if dart_cfg is not None:
        from motion import dart_camera_trajectory, dart_camera_velocity
        total = float(np.linalg.norm(path[-1] - path[0]))
        # 光轴梳齿密度：每约 1.2 m 一条；想更密就调小 1.2（下面 0.3 是梳齿长度 [m]）。
        n_comb = max(2, int(total / 1.2) + 1)
        for tc in np.linspace(0.0, float(t[-1]), n_comb):
            pc = dart_camera_trajectory(float(tc), dart_cfg)
            vc = dart_camera_velocity(float(tc), dart_cfg)
            nv = float(np.linalg.norm(vc))
            if nv > 1e-9:
                comb.append((pc, pc + 0.3 * vc / nv))

    # 轴铺满画布：轴矩形由 OVERVIEW_RECT 给出（值 >1 即"外扩"），顶部留给画布层标题（suptitle），
    # 不占用 3D 内容空间。
    fig = plt.figure(figsize=figsize, dpi=100)
    # 当前 rect=1.82 → 内容实测占画布 97.5%×98.9%；弹道两端贴边但**不裁**（OVERVIEW_*LIM 留了余量）。
    # ⚠ 外扩后标题/图例必须用画布层 API（fig.suptitle / fig.legend），见 update() 末尾。
    ax = fig.add_axes(list(OVERVIEW_RECT), projection="3d")

    def update(k: int):
        j = int(idx_map[k])
        C, U, Rk, tk = P_d[j], uav[j], float(R[j]), float(t[j])
        ax.clear()
        ax.set_proj_type("ortho")
        ax.view_init(elev=18, azim=-100)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color="gray", lw=1.0, ls=":",
                alpha=0.7, label="\u5f39\u9053\uff08\u5168\u7a0b\uff09")
        ax.scatter(pt_marks[:, 0], pt_marks[:, 1], pt_marks[:, 2], s=8, color="gray",
                   alpha=0.75, depthshade=False, label="0.5 s \u95f4\u9694")
        if k > 0:
            ax.plot(P_d[: j + 1, 0], P_d[: j + 1, 1], P_d[: j + 1, 2],
                    color="orange", lw=2.4, label="\u5df2\u98de\u8fc7")
        ax.scatter([path[0, 0]], [path[0, 1]], [path[0, 2]], s=60, color="black",
                   depthshade=False, zorder=7, label="\u8d77\u59cb\u4f4d\u7f6e\uff08\u53d1\u5c04\u70b9\uff09")
        ax.scatter([path[-1, 0]], [path[-1, 1]], [path[-1, 2]], s=110, color="darkgreen",
                   marker="X", depthshade=False, zorder=7, label="\u672b\u7aef\u4f4d\u7f6e\uff08\u706f\u5fc3\uff09")
        pose = camera_pose(float(tk), dart_cfg) if dart_cfg is not None else None
        ax.scatter([C[0]], [C[1]], [C[2]], s=90, color="red", marker="^",
                   depthshade=False, zorder=8, label="\u98de\u9556\u76f8\u673a")
        corners = uav_corners(U, uav_size_m)
        for i, jj in uav_edges():
            ax.plot([corners[i, 0], corners[jj, 0]], [corners[i, 1], corners[jj, 1]],
                    [corners[i, 2], corners[jj, 2]], color="blue", lw=1.2)
        ax.scatter([U[0]], [U[1]], [U[2]], s=25, color="blue", depthshade=False, label="\u65e0\u4eba\u673a")
        ax.plot(disc[:, 0], disc[:, 1], disc[:, 2], color="lime", lw=2.5, label="\u706f\u76d8")
        ax.add_collection3d(Poly3DCollection([disc], facecolor="lime", alpha=0.5,
                                             edgecolor="green"))
        ax.plot([C[0], center[0]], [C[1], center[1]], [C[2], center[2]],
                color="orange", lw=1.2, ls="--", alpha=0.85, label="LOS")
        for p_a, p_b in comb:
            ax.plot([p_a[0], p_b[0]], [p_a[1], p_b[1]], [p_a[2], p_b[2]],
                    color="crimson", lw=0.9, alpha=0.4)
        if pose is not None:
            tip = pose.position + 0.6 * pose.forward
            ax.plot([C[0], tip[0]], [C[1], tip[1]], [C[2], tip[2]], color="red", lw=2.6,
                    zorder=6, label="\u76f8\u673a\u5149\u8f74\uff08= \u5f39\u9053\u5207\u7ebf\uff09")
            if camera is not None:
                # 调参（★）：视锥 4 条角射线 + 顶点环；长度与"是否按盒子裁剪"见 OVERVIEW_FRUSTUM_*。
                cns = np.asarray(
                    frustum_corner_rays(pose, camera, float(OVERVIEW_FRUSTUM_LEN_M)), dtype=float
                )
                if OVERVIEW_FRUSTUM_CLIP:
                    cns = np.asarray(
                        [clip_segment_to_box(C, cn, OVERVIEW_XLIM, OVERVIEW_YLIM, OVERVIEW_ZLIM)
                         for cn in cns],
                        dtype=float,
                    )
                for cn in cns:
                    ax.plot([C[0], cn[0]], [C[1], cn[1]], [C[2], cn[2]],
                            color="crimson", lw=0.7, alpha=0.5)
                # \u7aef\u9762\u6846\u5fc5\u987b\u95ed\u5408\uff1ac0->c1->c2->c3->c0\uff08\u4e4b\u524d\u5c11\u4e86 c3->c0 \u4e00\u6761\u8fb9\uff09
                loop = np.vstack([cns, cns[:1]])
                ax.plot(loop[:, 0], loop[:, 1], loop[:, 2], color="crimson", lw=0.7, alpha=0.5,
                        label="\u89c6\u9525")
        # ---- 调参（★）：视频 A（全程）的坐标范围与盒子比例 ------------------------
        # 三个旋钮的取值全部集中在文件上方的 OVERVIEW_* 常量里，这里只应用，不再硬编码：
        #   OVERVIEW_RECT    → "拉长/铺满"（等比放大，长度与高度同时增长）
        #   OVERVIEW_Z_BOX   → "盒子形状"（z 越大越高瘦；13.5 对应 1.78:1，与画布同比例）
        #   OVERVIEW_*LIM    → "弹道在盒子里占多满"（收紧数据范围，不改盒子大小）
        # ⚠ 不要加 set_box_aspect(..., zoom>1)、不要改成分段/压缩 x 映射：
        #   前者会把发射段/命中段切出画框，后者会让弹道出现视觉折角（历史踩坑，见 README 附录 B）。
        ax.set_xlim(*OVERVIEW_XLIM)
        ax.set_ylim(*OVERVIEW_YLIM)
        ax.set_zlim(*OVERVIEW_ZLIM)
        ax.set_box_aspect((26.0, 4.0, float(OVERVIEW_Z_BOX)))
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        # 图例锚在**画布层**：轴已外扩 3%，若用 ax.legend 会有一部分被顶出画布。
        fig.legend(fontsize=10, loc="upper left", bbox_to_anchor=(0.008, 0.945),
                   framealpha=0.85, ncol=2)
        state = "\u6709\u6548\u906e\u6321" if Rk >= float(r_th) else "\u906e\u6321\u4e0d\u8db3"
        # 标题放在画布层：轴已铺满画布，ax.set_title 会被顶出可视区。
        # y=0.985（不是 0.995）：给中文字形顶部留 2~3 px，避免放大后标题被画布上边缘切掉。
        fig.suptitle(
            f"\u5168\u7a0b\u5f39\u9053\u6a21\u62df  |  t = {tk:.2f} s  |  "
            f"\u76f8\u673a = ({C[0]:.2f}, {C[1]:.2f}, {C[2]:.2f}) m  |  "
            f"$R_{{occ}}$ = {Rk:.3f}\uff08{state}\uff09",
            fontsize=13, y=0.985,
        )
        return ()

    return fig, update, n_frames


def build_terminal_animation(
    result: Dict[str, np.ndarray],
    lamp: LampConfig,
    uav_size_m: Sequence[float],
    full_path: Optional[np.ndarray] = None,
    title_prefix: str = "",
    figsize: Tuple[float, float] = ANIM_FIGSIZE,
    r_th: float = 0.7,
    dart_cfg=None,
    camera=None,
    fps: int = ANIM_FPS_DEFAULT,
) -> Tuple[object, Callable[[int], tuple], int]:
    """视频 B：末端 0.5 m 立方体窗口，保留全部精细元素。"""
    from camera import camera_pose, frustum_corner_rays
    from shapely.geometry import box as _shapely_box

    t = np.asarray(result["t"], dtype=float)
    P_d = np.asarray(result["P_d"], dtype=float)
    uav = np.asarray(result["uav"], dtype=float)
    R = np.asarray(result["R"], dtype=float)
    n_sim = int(t.size)
    n_frames = _frame_count(float(t[-1]), fps)
    idx_map = frame_indices(n_sim, n_frames)
    center = lamp.center_array
    u_basis, v_basis = plane_basis(lamp.normal_array)
    r_lamp = lamp.radius_m
    # ---- 调参：视频 B（末端）观察窗口 -------------------------------------------
    # 0.5 m 立方体窗口（prompt 锁定）。窗口越小越"精细"，但相机进入画面越晚；
    #   想更早看到相机：把 x 下限往负方向扩（如 -0.60），代价是遮挡细节占比下降。
    # 注意：阴影多边形按本窗口裁剪（zoom_clip_box），窗口外的部分不会绘制。
    window = {"x": (-0.40, 0.10), "y": (-0.25, 0.25), "z": (-0.25, 0.25)}
    theta = np.linspace(0.0, 2.0 * np.pi, 121)
    disc = center + np.outer(np.cos(theta) * r_lamp, u_basis) + np.outer(
        np.sin(theta) * r_lamp, v_basis
    )
    tpp = np.linspace(0.0, 2.0 * np.pi, 41, endpoint=False)
    lamp_pts = center + np.outer(np.cos(tpp) * r_lamp, u_basis) + np.outer(
        np.sin(tpp) * r_lamp, v_basis
    )
    # 窗口在灯平面局部坐标下的裁剪框
    quad = np.asarray([[0.0, yy, zz] for yy in window["y"] for zz in window["z"]], dtype=float)
    loc = np.asarray([plane_local_coords(q, center, u_basis, v_basis) for q in quad], dtype=float)
    clip_box = _shapely_box(loc[:, 0].min(), loc[:, 1].min(), loc[:, 0].max(), loc[:, 1].max())
    half = 0.6
    rect = np.asarray([center + a * u_basis + b * v_basis
                       for a, b in [(-half, -half), (half, -half), (half, half), (-half, half), (-half, -half)]])

    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection="3d")

    def update(k: int):
        j = int(idx_map[k])
        C, U, Rk, tk = P_d[j], uav[j], float(R[j]), float(t[j])
        res = compute_occlusion(C, U, uav_size_m, lamp, warn=False)
        pose = camera_pose(float(tk), dart_cfg) if dart_cfg is not None else None
        ax.clear()
        ax.set_proj_type("persp")
        ax.view_init(elev=35, azim=-135)
        ax.plot(rect[:, 0], rect[:, 1], rect[:, 2], color="gray", lw=0.8, ls=":",
                alpha=0.6, label="\u706f\u5e73\u9762\u53c2\u8003\u6846")
        # 1 相机
        ax.scatter([C[0]], [C[1]], [C[2]], s=90, color="red", marker="^",
                   depthshade=False, zorder=8, label="\u76f8\u673a C(t)")
        # 2 视锥
        if pose is not None and camera is not None:
            # 视锥长度 = clip(相机到灯心距离 × 1.05, 0.3, 0.7) m：
            #   下限 0.3 保证末端仍可见；上限 0.7 防止相机远时视锥撑满窗口。
            flen = min(max(float(np.linalg.norm(C - center)) * 1.05, 0.3), 0.7)
            cns = frustum_corner_rays(pose, camera, flen)
            for cn in cns:
                ax.plot([C[0], cn[0]], [C[1], cn[1]], [C[2], cn[2]],
                        color="crimson", lw=0.8, alpha=0.5)
            loop = np.vstack([cns, cns[:1]])
            ax.plot(loop[:, 0], loop[:, 1], loop[:, 2], color="crimson", lw=0.8, alpha=0.5,
                    label="\u76f8\u673a\u89c6\u9525")
        # 3 阴影（裁剪）+ 4 顶点投影线与投影点
        corners = uav_corners(U, uav_size_m)
        if res.shadow_polygon is not None and res.shadow_polygon.geom_type in {"Polygon", "MultiPolygon"}:
            clipped = res.shadow_polygon.intersection(clip_box)
            parts = [clipped] if clipped.geom_type == "Polygon" else list(getattr(clipped, "geoms", []))
            for part in parts:
                if part.is_empty or part.geom_type != "Polygon":
                    continue
                coords = np.asarray(part.exterior.coords, dtype=float)
                sh3 = np.asarray([plane_local_to_world(c, center, u_basis, v_basis)
                                  for c in coords], dtype=float)
                ax.add_collection3d(Poly3DCollection([sh3], facecolor="red", alpha=0.35,
                                                     edgecolor="darkred", lw=1.2))
                ax.plot(sh3[:, 0], sh3[:, 1], sh3[:, 2], color="darkred", lw=1.0,
                        label="\u9634\u5f71\u591a\u8fb9\u5f62")
            if res.shadow_points_3d is not None:
                pts3 = np.asarray(res.shadow_points_3d, dtype=float)
                first = True
                for corner, pp in zip(corners, pts3):
                    ax.plot([corner[0], pp[0]], [corner[1], pp[1]], [corner[2], pp[2]],
                            color="gray", lw=0.7, alpha=0.65,
                            label="\u9876\u70b9\u6295\u5f71\u7ebf" if first else None)
                    first = False
                ax.scatter(pts3[:, 0], pts3[:, 1], pts3[:, 2], color="darkred", s=5,
                           depthshade=False)
        # 5 灯盘圆周采样点
        ax.scatter(lamp_pts[:, 0], lamp_pts[:, 1], lamp_pts[:, 2], color="lime", s=6,
                   depthshade=False, label="\u706f\u76d8\u91c7\u6837\u70b9")
        # 6 阴影边界等弧长采样点
        if res.shadow_polygon is not None and res.shadow_polygon.geom_type in {"Polygon", "MultiPolygon"}:
            geom = res.shadow_polygon
            if geom.geom_type == "MultiPolygon":
                geom = max(geom.geoms, key=lambda g: g.area)
            clipped = geom.intersection(clip_box)
            if not clipped.is_empty and clipped.geom_type == "Polygon":
                per = float(clipped.exterior.length)
                if per > 1e-9:
                    ts = np.linspace(0.0, per, 49, endpoint=False)
                    p2 = np.asarray([clipped.exterior.interpolate(float(tt)).coords[0] for tt in ts], dtype=float)
                    p3 = np.asarray([plane_local_to_world(c, center, u_basis, v_basis) for c in p2], dtype=float)
                    ax.scatter(p3[:, 0], p3[:, 1], p3[:, 2], color="darkred", s=7,
                               depthshade=False, label="\u9634\u5f71\u91c7\u6837\u70b9")
        # 7 LOS
        ax.plot([C[0], center[0]], [C[1], center[1]], [C[2], center[2]],
                color="orange", lw=1.8, alpha=0.95, label="LOS")
        # 8 光轴
        if pose is not None:
            tip = pose.position + 0.45 * pose.forward
            ax.plot([C[0], tip[0]], [C[1], tip[1]], [C[2], tip[2]], color="crimson", lw=2.2,
                    zorder=6, label="\u76f8\u673a\u5149\u8f74")
        # 无人机 / 灯盘 / 灯心
        for i, jj in uav_edges():
            ax.plot([corners[i, 0], corners[jj, 0]], [corners[i, 1], corners[jj, 1]],
                    [corners[i, 2], corners[jj, 2]], color="blue", lw=2.2)
        ax.scatter([U[0]], [U[1]], [U[2]], s=30, color="blue", depthshade=False, label="\u65e0\u4eba\u673a\u4e2d\u5fc3")
        ax.plot(disc[:, 0], disc[:, 1], disc[:, 2], color="lime", lw=3.0)
        ax.add_collection3d(Poly3DCollection([disc], facecolor="lime", alpha=0.5,
                                             edgecolor="green"))
        ax.scatter([center[0]], [center[1]], [center[2]], s=45, color="green", marker="o",
                   edgecolors="darkgreen", depthshade=False, zorder=9, label="\u706f\u5fc3")
        ax.set_xlim(*window["x"])
        ax.set_ylim(*window["y"])
        ax.set_zlim(*window["z"])
        ax.set_box_aspect((0.5, 0.5, 0.5))
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        ax.legend(fontsize=8, loc="upper left", framealpha=0.85)
        # 无人机相对“绿灯圆心特征点”的坐标：世界系差值 + 灯局部系分解（法向/横向/竖直）
        d_uav = np.asarray(U, dtype=float) - center
        d_norm = float(np.dot(d_uav, lamp.normal_array))
        d_u = float(np.dot(d_uav, u_basis))
        d_v = float(np.dot(d_uav, v_basis))
        ax.text2D(
            0.02, 0.05,
            "\u65e0\u4eba\u673a\u76f8\u5bf9\u706f\u5fc3\u5750\u6807\uff1a"
            f"({d_uav[0]:+.3f}, {d_uav[1]:+.3f}, {d_uav[2]:+.3f}) m\n"
            f"\u706f\u5c40\u90e8\u7cfb\uff1a\u6cd5\u5411 {d_norm:+.3f} m | "
            f"\u6a2a\u5411 {d_u:+.3f} m | \u7ad6\u76f4 {d_v:+.3f} m",
            transform=ax.transAxes, fontsize=9,
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
        )
        state = "\u6709\u6548\u906e\u6321" if Rk >= float(r_th) else "\u906e\u6321\u4e0d\u8db3"
        ax.set_title(
            f"\u672b\u7aef\u7cbe\u7ec6\u5316\u6a21\u62df  |  t = {tk:.2f} s  |  "
            f"$R_{{occ}}$ = {Rk:.3f}\uff08{state}\uff09",
            fontsize=11,
        )
        return ()

    return fig, update, n_frames
