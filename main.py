"""主流程：一键复现全部仿真与可视化结果。

运行::

    python main.py              # 完整仿真 + 全部图表 + 实验 CSV + 视频（MP4）
    python main.py --no-video   # 只出图片与 CSV，跳过视频（最快）
    python main.py --gif        # 额外输出 GIF（默认不生成 GIF）

当前阶段：无人机只需**在距灯特定位置悬停**（``--strategy hover``，默认），不计算机动；
``--strategy fixed`` 为预留的“悬停在指定点”接口；当前主场景为 hover。

输出目录：``out/``（与 ``main.py`` 同级的 ``rm_uav_occlusion/out``）。
"""

from __future__ import annotations

import argparse
import sys
import time
import warnings
from dataclasses import replace
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np

# 允许从任意工作目录运行：把本文件所在目录加入导入路径。
BASE_DIR = Path(__file__).resolve().parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

from config import (
    A_MAX_OPTION,  # noqa: E402  (导入顺序依赖 sys.path 设置)
    DEFAULT_HOVER_STATION,
    DART_MODE_SIMPLE,
    HOVER_STATION_STANDOFF_M,
    INITIAL_NEAR_LAMP,
    LAUNCH_POINT_MECH_MM,
    LAUNCH_REL_LAMP_M,
    LAMP_CENTER_MECH_MM,
    SOURCE_LEGEND,
    CameraConfig,
    DartConfig,
    LampConfig,
    SimConfig,
    UAVConfig,
    default_camera,
    default_dart,
    default_lamp,
    default_sim,
    default_uav,
    scene_source_table,
)
from config import HoverStationConfig  # noqa: E402
from experiment import (  # noqa: E402
    camera_frame_metrics,
    compute_metrics,
    initial_uav_position,
    run_dynamic,
    static_grid_scan_xy,
    static_grid_scan_xz,
    static_grid_scan_lamp_parallel,
    truncate_run_before_contact,
)
from motion import dart_camera_trajectory  # noqa: E402
from occlusion import compute_occlusion  # noqa: E402
from params import build_catalog, print_catalog, write_reports  # noqa: E402
from camera import camera_pose  # noqa: E402
from strategies import POLICY_NAMES, make_policy  # noqa: E402
from validation import validate_run, validate_scene  # noqa: E402
from visualize import (
    build_overview_animation,
    build_terminal_animation,
    plot_3d_scene,
    plot_heatmap,
    save_animation,
)

OUT_DIR = BASE_DIR / "out"

# ---- 调参：演示截断距离（★常改）-----------------------------------------------
# 当 |相机 − 无人机| ≤ 该值时截断仿真与视频。作用：
#   1) 避免"相机贴到/穿过无人机"以及"相机与灯心重合"的奇异帧（否则会刷 uav.between 告警）；
#   2) 决定视频结束时刻：0.20 m → 约 1.86 s（弹道全程 1.92 s），即覆盖约 97% 飞行过程。
# 调大（如 0.45）→ 更早结束、末端细节看不到；调小（如 0.10）→ 更贴近命中，
#   但相机与无人机几乎重合，几何前提（无人机在相机与灯心之间）会变危险。
DART_STOP_DISTANCE_M: float = 0.20


def _hr(title: str) -> None:
    """打印分节标题。"""
    print("\n" + "=" * 78)
    print(title)
    print("=" * 78)


def _kv(label: str, value: str, source: str = "") -> None:
    suffix = f"   [{source}]" if source else ""
    print(f"  {label:<28} {value}{suffix}")


def print_config_summary(
    lamp: LampConfig,
    camera: CameraConfig,
    uav: UAVConfig,
    dart: DartConfig,
    sim: SimConfig,
    station: HoverStationConfig | None = None,
) -> None:
    """步骤 1：打印关键量（灯半径/面积、无人机质量）与全部参数来源标签。"""
    _hr("步骤 1/7：配置与参数来源")
    _kv("灯直径", f"{lamp.diameter_mm:g} mm", lamp.source_diameter)
    _kv("灯半径 r", f"{lamp.radius_m * 1000:.4f} mm = {lamp.radius_m:.6f} m", lamp.source_diameter)
    _kv("灯面积 pi r^2", f"{lamp.area_m2:.6e} m^2", lamp.source_diameter)
    _kv("无人机最大尺寸", f"{max(uav.size_m) * 1000:.1f} mm", uav.source_size)
    _kv(
        "无人机质量",
        f"{uav.mass_kg:.4f} kg  (y = -0.001225x + 0.3225, x = {max(uav.size_m) * 1000:.1f} mm)",
        uav.source_mass,
    )
    lo, hi = UAVConfig.mass_valid_size_range_mm()
    print(f"  质量公式适用范围：y ∈ [0.15, 0.249] kg  ->  x ∈ [{lo:.1f}, {hi:.1f}] mm")
    _kv(
        "发射点（机械数据）",
        f"绿灯中心 {LAMP_CENTER_MECH_MM} mm，发射点 {LAUNCH_POINT_MECH_MM} mm",
    )
    _kv(
        "发射点（灯局部坐标系）",
        f"({LAUNCH_REL_LAMP_M[0]:.5f}, {LAUNCH_REL_LAMP_M[1]:.5f}, {LAUNCH_REL_LAMP_M[2]:.5f}) m"
        f"  水平距离 {float(np.hypot(*LAUNCH_REL_LAMP_M[:2])):.3f} m",
    )
    if station is not None:
        station_pos = station.position(lamp)
        _kv(
            "悬停站位（当前主场景）",
            f"距灯心 {station.distance_to_lamp_m:.3f} m，法向 {station.standoff_m:.3f} m，"
            f"坐标 ({station_pos[0]:.3f}, {station_pos[1]:.3f}, {station_pos[2]:.3f}) m",
            station.source,
        )
        print("  说明：当前阶段无人机只在上述站位悬停、不做机动；机动策略接口已保留。")

    print("\n  来源标签总表：")
    rows = list(scene_source_table(lamp, camera, uav, dart, sim))
    if station is not None:
        rows = rows + list(station.source_rows())
    for name, value, source in rows:
        print(f"    - {name:<24} {value:<58} [{source}]")

    print("\n  标签含义：")
    for tag, desc in SOURCE_LEGEND.items():
        print(f"    - {tag:<24} {desc}")


def make_scene(
    dart_mode: str = DART_MODE_SIMPLE,
    v_max: float = 5.0,
    R_th: float = 0.7,
) -> Tuple[LampConfig, CameraConfig, UAVConfig, DartConfig, SimConfig]:
    """构造一组默认场景配置。"""
    lamp = default_lamp()
    camera = default_camera()
    uav = default_uav(max_speed=v_max, max_accel=A_MAX_OPTION)
    dart = default_dart(dart_mode)
    sim = default_sim(R_th=R_th)
    return lamp, camera, uav, dart, sim


def run_dynamic_suite(
    lamp: LampConfig,
    uav: UAVConfig,
    sim: SimConfig,
    strategy: str = "hover",
    station: HoverStationConfig = DEFAULT_HOVER_STATION,
    conditions: Sequence[str] = (INITIAL_NEAR_LAMP,),
    stop_distance_m: float | None = DART_STOP_DISTANCE_M,
) -> Dict[Tuple[str, str], Dict[str, np.ndarray]]:
    """对两种飞镖模式 x 两个初始工况运行动态仿真。

    默认策略 ``hover``（悬停站位，无人机不机动）；换成 ``los`` / ``predictive``
    即走预留的机动接口，其它代码无需改动。
    """
    policy = make_policy(strategy, station=station)
    runs: Dict[Tuple[str, str], Dict[str, np.ndarray]] = {}
    for mode in (DART_MODE_SIMPLE,):
        dart = default_dart(mode)
        for condition in conditions:
            p0 = initial_uav_position(condition, dart, lamp, sim, uav, station=station)
            runs[(mode, condition)] = run_dynamic(
                strategy=policy,
                dart_cfg=dart,
                uav_cfg=uav,
                lamp=lamp,
                sim_cfg=sim,
                initial_uav_pos=p0,
                P_star=p0,
                warn=False,
            )
            if stop_distance_m is not None:
                runs[(mode, condition)] = truncate_run_before_contact(
                    runs[(mode, condition)], stop_distance_m=stop_distance_m
                )
    return runs


def build_dart_scenario(
    lamp: LampConfig,
    uav: UAVConfig,
    sim: SimConfig,
    station: HoverStationConfig = DEFAULT_HOVER_STATION,
    stop_distance_m: float = DART_STOP_DISTANCE_M,
) -> Tuple[DartConfig, np.ndarray, Dict[str, np.ndarray]]:
    """抛物线飞镖 + 贴灯悬停无人机场景。

    返回 ``(dart_cfg, hover_point, run_result)``：

    * 无人机用 ``fixed`` 策略悬停于贴灯点 ``HOVER_NEAR_LAMP_M``，与绿灯保持相对静止；
    * 飞镖按抛物线飞行（上升 -> 顶点 -> 下降），下降段**落点为绿灯中心**；
    * 相机与无人机距离小于 ``stop_distance_m`` 时截断，避免“撞机 / 相机与灯心重合”
      的奇异帧（完整弹道仍由 ``dart_cfg.parabolic_position`` 给出）。
    """
    # 最简抛物线模型：发射点 + 落点 + v0 + 仰角 + 减速律；仿真窗口取模型给出的飞行时间。
    dart = default_dart(DART_MODE_SIMPLE)
    sim = replace(sim, t_end=float(dart.t_end))
    p0 = initial_uav_position(INITIAL_NEAR_LAMP, dart, lamp, sim, uav, station=station)
    run = run_dynamic(
        make_policy("hover", station=station), dart, uav, lamp, sim, p0, P_star=p0, warn=False
    )
    run = truncate_run_before_contact(run, stop_distance_m=stop_distance_m)
    return dart, p0, run





def render_dual_animations(
    lamp: LampConfig,
    uav: UAVConfig,
    sim: SimConfig,
    station: HoverStationConfig,
    camera_cfg: CameraConfig,
    stop_distance_m: float = DART_STOP_DISTANCE_M,
    fps: int = 30,
    dpi: int = 100,
) -> Tuple[Path, Path]:
    """同时生成视频 A（全程）与视频 B（末端）：同一仿真结果、帧对帧同步。"""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    dart = default_dart(DART_MODE_SIMPLE)
    sim = replace(sim, t_end=float(dart.t_end))
    p0 = initial_uav_position(INITIAL_NEAR_LAMP, dart, lamp, sim, uav, station=station)
    policy = make_policy("hover", station=station)
    result = run_dynamic(policy, dart, uav, lamp, sim, p0, P_star=p0, warn=False)
    result = truncate_run_before_contact(result, stop_distance_m=stop_distance_m)
    full_path = dart_camera_trajectory(np.linspace(0.0, dart.t_end, 240), dart)
    prefix = f"{station.distance_to_lamp_m:.2f} m 悬停"

    out: list = []
    for builder, name in ((build_overview_animation, "animation_overview.mp4"),
                          (build_terminal_animation, "animation_terminal.mp4")):
        fig, update, n_frames = builder(
            result, lamp, uav.size_m, full_path=full_path, title_prefix=prefix,
            r_th=sim.R_th, dart_cfg=dart, camera=camera_cfg, fps=fps,
        )
        anim = FuncAnimation(fig, update, frames=n_frames, interval=1000 // fps)
        target = OUT_DIR / name
        save_animation(anim, target, fps=fps, dpi=dpi)
        plt.close(fig)
        out.append(target)
    return out[0], out[1]


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="无人机遮挡绿色引导灯几何仿真")
    parser.add_argument(
        "--strategy", default="hover", choices=POLICY_NAMES,
        help="目标策略：hover=距灯特定位置悬停（当前阶段，默认）；"
             "fixed=悬停在固定点（预留接口）",
    )
    parser.add_argument(
        "--standoff", type=float, default=HOVER_STATION_STANDOFF_M,
        help=f"悬停站位到灯平面的距离 [m]（默认 {HOVER_STATION_STANDOFF_M:g}）",
    )
    parser.add_argument(
        "--offset-v", dest="offset_v", type=float, default=None,
        help=f"悬停站位在灯平面内的 v 偏置 [m]（默认 {DEFAULT_HOVER_STATION.offset_v_m:g}）",
    )
    parser.add_argument("--R-th", dest="R_th", type=float, default=0.7)
    parser.add_argument(
        "--no-video", action="store_true",
        help="跳过视频渲染（只出图片与 CSV，最快）",
    )
# ---- CLI 默认值也是调参入口 ----------------------------------------------------
# --fps / --video-dpi 默认 30 / 100 → 1920×1080 @30fps（prompt 锁定值）。
# 临时出片可用 --fps 25 --video-dpi 80 加速；渲染时间与 (fps × dpi²) 近似正比。
    parser.add_argument("--fps", type=int, default=30, help="视频帧率（默认 30 → 58 帧）")
    parser.add_argument("--video-dpi", dest="video_dpi", type=int, default=100,
                        help="视频 dpi（默认 100，配合 19.2x10.8 in → 1920x1080）")
    parser.add_argument("--params", action="store_true",
                        help="额外导出参数总表 out/parameters.md / .csv（默认不生成）")
    parser.add_argument("--no-progress", action="store_true", help="关闭 tqdm 进度条")
    args = parser.parse_args(argv)

    t_start = time.perf_counter()
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    lamp, camera, uav, dart, sim = make_scene(DART_MODE_SIMPLE, 5.0, args.R_th)
    station = HoverStationConfig(
        standoff_m=float(args.standoff),
        offset_u_m=float(DEFAULT_HOVER_STATION.offset_u_m),
        offset_v_m=(
            float(DEFAULT_HOVER_STATION.offset_v_m) if args.offset_v is None else float(args.offset_v)
        ),
    )

    # --- 1. 配置与来源 ---------------------------------------------------
    print_config_summary(lamp, camera, uav, dart, sim, station)

    # --- 1b. 参数总表（可选，默认关闭）-----------------------------------
    # 需要"所有参数 + 参数名 + 当前取值 + 来源 + 改动方式"的完整清单时加 --params：
    # 控制台打印 + 写出 out/parameters.md 与 out/parameters.csv（默认不生成，保持 out/ 精简）。
    if args.params:
        rows = build_catalog(lamp, camera, uav, sim, station=station, dart_modes=(dart.mode,))
        print_catalog(rows)
        md_path, csv_path = write_reports(rows, OUT_DIR)
        print(f"  -> 参数总表已写出：{md_path.name} / {csv_path.name}（共 {len(rows)} 行）")

    # --- 2. 场景校验 -----------------------------------------------------
    _hr("步骤 2/7：场景校验")
    report = validate_scene(lamp, camera, uav, dart, sim, station=station)
    if not report.ok:
        # 配置类硬错误（例如站位超出机身安全边界）先暴露，避免继续做无意义的仿真。
        print(report.format())
        report.raise_if_errors()
    probe_p0 = initial_uav_position(INITIAL_NEAR_LAMP, dart, lamp, sim, uav, station=station)
    probe = run_dynamic(
        make_policy("hover", station=station), dart, uav, lamp, sim, probe_p0, P_star=probe_p0,
        warn=False,
    )
    # 与主场景一致地截断到“接触前”：否则 t > dart.t_end 的帧相机会停在灯心，
    # 几何前提（无人机位于相机与灯平面之间）不成立，会刷出一串无意义的 uav.between 告警。
    probe = truncate_run_before_contact(probe, stop_distance_m=DART_STOP_DISTANCE_M)
    report.extend(validate_run(probe, uav, sim, lamp).issues)
    print(report.format())
    report.raise_if_errors()
    print("  -> 校验通过（无硬性错误）")
    del probe

    # --- 4. 静态热图 -----------------------------------------------------
    _hr("步骤 3/7：静态遮挡率热图")
    cam_mid = dart_camera_trajectory(sim.t_end / 2.0, dart)

    # 4a) 平行于灯平面、位于灯前方 standoff 处的截面（最直接的一张：站位该选在哪）
    ys_p, zs_p, heat_plane = static_grid_scan_lamp_parallel(
        cam_mid, uav.size_m, lamp, standoff_m=station.standoff_m,
        y_range=(-0.5, 0.5, 41), z_range=(-0.5, 0.5, 41),
    )
    plot_heatmap(
        heat_plane, ys_p, zs_p, "y [m]（横向）", "z [m]（竖直）",
        f"平行灯平面截面遮挡率 (x = -{station.standoff_m:g} m，相机 C = {np.round(cam_mid, 2).tolist()})",
        sim.R_th, OUT_DIR / "heatmap_lamp_plane.png",
    )
    iv, ju = np.unravel_index(int(np.argmax(heat_plane)), heat_plane.shape)
    print(f"  out/heatmap_lamp_plane.png : max R_occ = {heat_plane.max():.4f} @ "
          f"(y = {ys_p[ju]:.3f} m, z = {zs_p[iv]:.3f} m)")

    # 4b) 侧视切片（固定 y，接近弹道所在横向位置）
    xs, zs, heat_xz = static_grid_scan_xz(
        cam_mid, uav.size_m, lamp, y=-0.2, x_range=(-4.0, 0.2, 61), z_range=(-1.0, 1.0, 41)
    )
    p_xz = plot_heatmap(
        heat_xz, xs, zs, "x [m]", "z [m]",
        f"XZ 切片遮挡率热图 (y = -0.2 m, 相机 C = {np.round(cam_mid, 2).tolist()})",
        sim.R_th, OUT_DIR / "heatmap_xz.png",
    )
    iz, ix = np.unravel_index(int(np.argmax(heat_xz)), heat_xz.shape)
    print(f"  out/heatmap_xz.png    : max R_occ = {heat_xz.max():.4f} @ "
          f"(x = {xs[ix]:.3f} m, z = {zs[iz]:.3f} m)")

    # 4c) 水平切片（固定 z = 灯心高度）
    xs2, ys2, heat_xy = static_grid_scan_xy(
        cam_mid, uav.size_m, lamp, z=0.0, x_range=(-4.0, 0.2, 61), y_range=(-1.5, 1.5, 41)
    )
    p_xy = plot_heatmap(
        heat_xy, xs2, ys2, "x [m]", "y [m]",
        f"XY 切片遮挡率热图 (z = 0 m, 相机 C = {np.round(cam_mid, 2).tolist()})",
        sim.R_th, OUT_DIR / "heatmap_xy.png",
    )
    iy2, ix2 = np.unravel_index(int(np.argmax(heat_xy)), heat_xy.shape)
    print(f"  out/heatmap_xy.png    : max R_occ = {heat_xy.max():.4f} @ "
          f"(x = {xs2[ix2]:.3f} m, y = {ys2[iy2]:.3f} m)")

    # --- 6-8. 动态仿真（悬停站位；可选机动策略） -------------------------
    _hr("步骤 4/7：动态仿真（simple 弹道 + hover 悬停）")
    conditions = (INITIAL_NEAR_LAMP,)
    print(f"  策略 {args.strategy}：无人机在距灯 {station.distance_to_lamp_m:.3f} m 处悬停，不机动")
    runs = run_dynamic_suite(
        lamp, uav, sim, strategy=args.strategy, station=station, conditions=conditions
    )
    for (mode, condition), res in runs.items():
        m = compute_metrics(res["t"], res["R"], sim.R_th)
        print(
            f"  [{args.strategy:>10}] {mode:<16} {condition:<15} "
            f"T_occ = {m['T_occ']:.3f} s | T_cont_max = {m['T_cont_max']:.3f} s | "
            f"first_occlusion_t = {m['first_occlusion_t']} s | "
            f"R_max = {m['R_max']:.3f} | R_mean = {m['R_mean']:.3f}"
        )
        vr = validate_run(res, uav, sim, lamp)
        if not vr.ok:
            print("    !! 校验失败：")
            for i in vr.errors:
                print(f"       {i}")

    # --- 10. 3D 场景 -----------------------------------------------------
    _hr("步骤 5/7：3D 场景图")
    key = next(iter(runs))
    res_ref = runs[key]
    # 取“后段（t >= t_end/2）中最后一个有效遮挡帧”，此时相机已接近灯，
    # 几何关系最紧、最能说明问题；若全程无有效遮挡则退化为取 R_occ 最大的帧。
    late = res_ref["t"] >= sim.t_end / 2.0
    eff = (res_ref["R"] >= sim.R_th) & late
    k_best = int(np.where(eff)[0][-1]) if bool(np.any(eff)) else int(np.argmax(res_ref["R"]))
    scene_info = plot_3d_scene(
        res_ref["P_d"][k_best], res_ref["uav"][k_best], uav.size_m, lamp,
        OUT_DIR / "scene_3d.png",
    )
    print(f"  out/scene_3d.png      : t = {res_ref['t'][k_best]:.2f} s, "
          f"R_occ = {scene_info['R_occ']:.4f}, valid = {scene_info['valid']}")

    # 场景 B：抛物线飞镖（下降段落点为绿灯）+ 贴灯悬停无人机（与绿灯相对静止）
    dart_p, hover_p0, dart_run = build_dart_scenario(lamp, uav, sim, station=station)
    dart_metrics = compute_metrics(dart_run["t"], dart_run["R"], sim.R_th)
    print(
        f"\n  最简抛物线模型：发射 {tuple(round(v, 3) for v in dart_p.launch)} m -> "
        f"落点 {tuple(round(v, 3) for v in dart_p.impact)} m（绿灯中心）"
    )
    print(
        f"  初速 {dart_p.v0_mps:g} m/s，初始仰角 {dart_p.theta0_deg:g}°，"
        f"速度降低率 {dart_p.speed_decay_per_s:g} 1/s（v(t) = v0·e^(-k t)）"
    )
    print(
        f"  等效重力 {dart_p.effective_g_mps2:.3f} m/s²（由抛物线穿过落点反解）；"
        f"弹道弧长 {dart_p.simple_path_len_m():.2f} m，飞行时间 {dart_p.t_end:.3f} s，"
        f"末端速度 {float(np.linalg.norm(dart_p.simple_velocity(dart_p.t_end))):.2f} m/s"
    )
    print(
        f"  悬停点 ({hover_p0[0]:.2f}, {hover_p0[1]:.2f}, {hover_p0[2]:.2f}) m（相对绿灯静止）；"
        f"演示截断于 |C-U| <= {DART_STOP_DISTANCE_M:g} m，"
        f"截止 t = {dart_run['t'][-1]:.2f} s"
    )
    print(
        f"  T_occ = {dart_metrics['T_occ']:.3f} s | T_cont_max = {dart_metrics['T_cont_max']:.3f} s | "
        f"R_min = {float(np.min(dart_run['R'])):.3f} | R_mean = {dart_metrics['R_mean']:.3f} | "
        f"R_max = {dart_metrics['R_max']:.3f}"
    )
    # 相机朝向 = 弹道切线：视场/成像统计 + 第一人称画面
    cam_metrics = camera_frame_metrics(dart_run, dart_p, camera, lamp, uav.size_m, sim.R_th)
    print(
        f"  相机朝向（光轴 = 弹道切线）：灯在视场内帧占比 = {cam_metrics['fov_in_fraction'] * 100:.1f}% | "
        f"最小视场余量 = {cam_metrics['fov_margin_min_deg']:.2f}° | "
        f"灯心相对光轴最大偏角 = {cam_metrics['off_axis_max_deg']:.2f}°"
    )
    print(
        f"  像面遮挡比（无人机轮廓 ∩ 灯盘投影）：min = {cam_metrics['image_R_min']:.3f} | "
        f"mean = {cam_metrics['image_R_mean']:.3f} | 与几何 R_occ 判定一致率 = "
        f"{cam_metrics['image_agreement'] * 100:.1f}%"
    )
    k_last = int(dart_run["t"].size - 1)
    dart_scene = plot_3d_scene(
        dart_run["P_d"][k_last], dart_run["uav"][k_last], uav.size_m, lamp,
        OUT_DIR / "scene_dart_hover.png",
        title="抛物线飞镖 + 贴灯悬停无人机（下降段落点为绿灯）",
        pose=camera_pose(float(dart_run["t"][k_last]), dart_p),
        camera_cfg=camera,
    )
    print(
        f"  out/scene_dart_hover.png: t = {dart_run['t'][k_last]:.2f} s, "
        f"R_occ = {dart_scene['R_occ']:.4f}"
    )

    # --- 12. 视频 + 收尾 --------------------------------------------------
    if not args.no_video:
        _hr("步骤 6/7：双视频输出")
        try:
            path_a, path_b = render_dual_animations(
                lamp, uav, sim, station, camera,
                stop_distance_m=DART_STOP_DISTANCE_M,
                fps=args.fps, dpi=args.video_dpi,
            )
            for pp in (path_a, path_b):
                print(f"  {pp.relative_to(BASE_DIR)!s:<32} {pp.stat().st_size / 1024.0:>9.1f} KB")
        except RuntimeError as exc:
            print(f"  视频渲染失败（已跳过）：{exc}")

    _hr("步骤 7/7：完成")
    outputs = sorted(OUT_DIR.glob("*"))
    for path in outputs:
        size_kb = path.stat().st_size / 1024.0
        print(f"  {path.relative_to(BASE_DIR)!s:<32} {size_kb:>9.1f} KB")
    elapsed = time.perf_counter() - t_start
    print(f"\n  全部完成，用时 {elapsed:.2f} s。")
    print("  说明：本结果为几何可行性研究，不代表已获裁判确认的合法战术；")
    print("        R_th 为分析阈值，不是官方视觉失效阈值。")
    return 0


if __name__ == "__main__":
    # 保证在 Windows 控制台（可能为 GBK 代码页）下打印中文不报错。
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:  # pragma: no cover
        pass
    with warnings.catch_warnings():
        # 首次运行会把遮挡模块的退化告警按类型去重打印，这里保持默认行为。
        raise SystemExit(main())
