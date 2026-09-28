"""参数总表：把当前效果涉及的**全部可调参数**分门别类列出。

每个条目都给出三件事，便于直接改：

1. ``name``          ：代码里的参数名（``类名.字段名``，可直接搜索定位）
2. ``value``         ：当前取值（含单位）
3. ``how_to_change`` ：改动方式（CLI 开关优先，其次代码位置）

运行 ``python main.py`` 时会打印该表并写出 ``out/parameters.md`` 与
``out/parameters.csv``；也可以单独调用::

    from params import build_catalog, dart_kinematics, write_reports
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple, Union
import unicodedata

import numpy as np
import pandas as pd

from config import (
    A_MAX_OPTION,
    CAMERA_UP_REF,
    DEFAULT_HOVER_STATION,
    ENGINEERING_ASSUMPTION,
    HOVER_STATION_OFFSET_V_M,
    HOVER_STATION_STANDOFF_M,
    LAUNCH_POINT_MECH_MM,
    LAUNCH_REL_LAMP_M,
    LAMP_CENTER_MECH_MM,
    MODELING_ASSUMPTION,
    OFFICIAL_2026_BASELINE,
    OFFICIAL_2027,
    UNKNOWN,
    USER_SPEC,
    CameraConfig,
    DartConfig,
    HoverStationConfig,
    LampConfig,
    SimConfig,
    UAVConfig,
)
from motion import dart_camera_trajectory, dart_camera_velocity
from occlusion import LAMP_DISC_QUAD_SEGS
from visualize import DPI as VIS_DPI

#: 来源标签的紧凑写法（控制台用；文件里仍写全称）。
SHORT_SOURCE: Dict[str, str] = {
    OFFICIAL_2027: "O27",
    OFFICIAL_2026_BASELINE: "O26",
    ENGINEERING_ASSUMPTION: "ENG",
    MODELING_ASSUMPTION: "MOD",
    USER_SPEC: "USR",
    UNKNOWN: "UNK",
}


@dataclass(frozen=True)
class ParamRow:
    """一条参数记录。"""

    category: str
    name: str
    value: str
    source: str
    how_to_change: str
    note: str = ""


def _v3(p: Sequence[float]) -> str:
    return f"({float(p[0]):.3f}, {float(p[1]):.3f}, {float(p[2]):.3f})"




def short_source(tag: str) -> str:
    return SHORT_SOURCE.get(tag, tag)


def display_width(text: str) -> int:
    """按终端显示宽度计算（CJK 全角字符算 2 列）。"""
    return sum(2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1 for ch in text)


def pad(text: str, width: int, truncate: bool = False) -> str:
    """按显示宽度左对齐补空格；``truncate=True`` 时超宽截断并加省略号。"""
    if display_width(text) > width:
        if not truncate:
            return text
        out = ""
        for ch in text:
            if display_width(out + ch) > width - 1:
                break
            out += ch
        text = out + "…"
    return text + " " * max(0, width - display_width(text))


# ---------------------------------------------------------------------------
# 静态参数
# ---------------------------------------------------------------------------


def build_catalog(
    lamp: LampConfig,
    camera: CameraConfig,
    uav: UAVConfig,
    sim: SimConfig,
    station: HoverStationConfig = DEFAULT_HOVER_STATION,
    dart_modes: Sequence[str] = ("simple",),
    include_kinematics: bool = True,
) -> List[ParamRow]:
    """返回当前效果的全部参数（含每条弹道的运动学实测量）。"""
    rows: List[ParamRow] = []

    # ---- 1. 场景与坐标系 ------------------------------------------------
    rows += [
        ParamRow("1. 场景 / 坐标系", "LampConfig.center", _v3(lamp.center), lamp.source_pose,
                 "config.LampConfig.center", "灯心即局部坐标原点"),
        ParamRow("1. 场景 / 坐标系", "LampConfig.normal", _v3(lamp.normal), lamp.source_pose,
                 "config.LampConfig.normal", "灯盘法线，指向迎面而来的飞镖（-X）"),
        ParamRow("1. 场景 / 坐标系", "LAMP_CENTER_MECH_MM", str(LAMP_CENTER_MECH_MM) + " mm",
                 USER_SPEC, "config.LAMP_CENTER_MECH_MM", "机械装配坐标系下的绿灯中心"),
        ParamRow("1. 场景 / 坐标系", "LAUNCH_POINT_MECH_MM", str(LAUNCH_POINT_MECH_MM) + " mm",
                 USER_SPEC, "config.LAUNCH_POINT_MECH_MM", "机械装配坐标系下的发射点"),
        ParamRow("1. 场景 / 坐标系", "LAUNCH_REL_LAMP_M", _v3(LAUNCH_REL_LAMP_M), USER_SPEC,
                 "config.LAUNCH_REL_LAMP_M",
                 "发射点在灯局部坐标系下的位置（= 机械数据相减）"),
        ParamRow("1. 场景 / 坐标系", "DEFAULT_UAV_BOUNDS", str(dict(sim.uav_bounds)), sim.source_bounds,
                 "config.DEFAULT_UAV_BOUNDS 或 SimConfig.uav_bounds", "机身允许占据的轴对齐包围盒 [m]"),
    ]

    # ---- 2. 绿灯 --------------------------------------------------------
    rows += [
        ParamRow("2. 绿灯（灯盘）", "LampConfig.diameter_mm", f"{lamp.diameter_mm:g} mm",
                 lamp.source_diameter, "config.LampConfig.diameter_mm / config.LAMP_DIAMETER_MM",
                 "发光部分直径"),
        ParamRow("2. 绿灯（灯盘）", "LampConfig.radius_m（派生）", f"{lamp.radius_m * 1000:.4f} mm",
                 lamp.source_diameter, "由 diameter_mm / 2000 自动得到", "= diameter_mm / 2000"),
        ParamRow("2. 绿灯（灯盘）", "LampConfig.area_m2（派生）", f"{lamp.area_m2:.6e} m^2",
                 lamp.source_diameter, "由半径自动得到", "= pi r^2，R_occ 的分母"),
        ParamRow("2. 绿灯（灯盘）", "occlusion.LAMP_DISC_QUAD_SEGS", f"{LAMP_DISC_QUAD_SEGS}",
                 ENGINEERING_ASSUMPTION, "occlusion.LAMP_DISC_QUAD_SEGS",
                 "灯圆盘多边形逼近精度（每象限段数）"),
    ]

    # ---- 3. 相机内参 ----------------------------------------------------
    rows += [
        ParamRow("3. 相机内参", "CameraConfig.resolution",
                 f"{camera.resolution[0]}x{camera.resolution[1]}", camera.source_resolution,
                 "config.CameraConfig.resolution", "图像分辨率"),
        ParamRow("3. 相机内参", "CameraConfig.fov_h_deg", f"{camera.fov_h_deg:g} deg",
                 camera.source_fov, "config.CameraConfig.fov_h_deg", "水平 FOV"),
        ParamRow("3. 相机内参", "CameraConfig.fov_v_deg（派生）", f"{camera.fov_v_deg:.3f} deg",
                 camera.source_fov, "由 fov_h_deg 与画幅比得到", "仅用于可选的图像平面可视化"),
        ParamRow("3. 相机内参", "CameraConfig.focal_length_mm", f"{camera.focal_length_mm:g} mm",
                 camera.source_focal, "config.CameraConfig.focal_length_mm",
                 "仅实物记录；靶面尺寸 UNKNOWN，不参与 FOV 推导"),
        ParamRow("3. 相机内参", "CameraConfig.distortion_enabled", str(camera.distortion_enabled),
                 ENGINEERING_ASSUMPTION, "config.CameraConfig.distortion_enabled",
                 "MVP 不建模畸变"),
        ParamRow("3. 相机内参", "config.CAMERA_UP_REF", _v3(CAMERA_UP_REF), ENGINEERING_ASSUMPTION,
                 "config.CAMERA_UP_REF",
                 "光轴取弹道切线 dC/dt，本向量只用于确定滚转（相机的“上方向”）"),
        ParamRow("3. 相机内参", "CameraConfig.focal_px（派生）", f"{camera.focal_px:.2f} px",
                 ENGINEERING_ASSUMPTION, "由 fov_h_deg 与分辨率自动得到",
                 "= (W/2)/tan(HFOV/2)，第一人称成像用"),
    ]

    # ---- 4. 飞镖弹道（参数） --------------------------------------------
    for mode in dart_modes:
        dart = DartConfig(mode=mode)
        rows += _dart_parameter_rows(dart)
        if include_kinematics:
            rows += dart_kinematics(dart, lamp)

    # ---- 5. 无人机 ------------------------------------------------------
    size_mm = max(uav.size_m) * 1000.0
    rows += [
        ParamRow("5. 无人机", "UAVConfig.size_m", _v3(uav.size_m), uav.source_size,
                 "config.UAVConfig.size_m / config.UAV_SIZE_MM", "机体三轴尺寸 [m]"),
        ParamRow("5. 无人机", "UAVConfig.corner_radius_m", f"{uav.corner_radius_m:g} m",
                 USER_SPEC, "config.UAVConfig.corner_radius_m", "MVP 为立方体，圆角=0"),
        ParamRow("5. 无人机", "UAVConfig.max_speed", f"{uav.max_speed:g} m/s", uav.source_speed,
                 "config.UAVConfig.max_speed / CLI --strategy 相关实验列 v_max_mps",
                 "仅机动策略会用到；悬停场景不涉及"),
        ParamRow("5. 无人机", "UAVConfig.max_accel", f"{uav.max_accel:g} m/s^2", uav.source_accel,
                 "config.UAVConfig.max_accel", "仅机动策略会用到"),
        ParamRow("5. 无人机", "UAVConfig.mass_kg", f"{uav.mass_kg:.4f} kg", uav.source_mass,
                 "config.UAVConfig.mass_kg",
                 "RM2027 公式 y = -0.001225x + 0.3225，x = 最大伸展尺寸 [mm]"),
        ParamRow("5. 无人机", "UAVConfig.weight_from_size_mm（公式）",
                 f"y = -0.001225x + 0.3225（x = {size_mm:.1f} mm）", OFFICIAL_2027,
                 "config.UAVConfig.weight_from_size_mm", "适用范围 y ∈ [0.15, 0.249] kg"),
    ]

    # ---- 6. 悬停站位 ----------------------------------------------------
    pos = station.position(lamp)
    rows += [
        ParamRow("6. 无人机悬停站位（当前主场景）", "HoverStationConfig.standoff_m",
                 f"{station.standoff_m:g} m", station.source,
                 "CLI --standoff / config.HoverStationConfig.standoff_m",
                 "沿灯法线到灯平面的距离"),
        ParamRow("6. 无人机悬停站位（当前主场景）", "HoverStationConfig.offset_u_m",
                 f"{station.offset_u_m:+.3f} m", station.source,
                 "config.HoverStationConfig.offset_u_m",
                 "灯平面内 u（默认 = 世界 x）方向偏置"),
        ParamRow("6. 无人机悬停站位（当前主场景）", "HoverStationConfig.offset_v_m",
                 f"{station.offset_v_m:+.3f} m", station.source,
                 "CLI --offset-v / config.HoverStationConfig.offset_v_m",
                 "灯平面内 v（默认 = 世界 z，即高度）方向偏置"),
        ParamRow("6. 无人机悬停站位（当前主场景）", "HoverStationConfig.distance_to_lamp_m（派生）",
                 f"{station.distance_to_lamp_m:.3f} m", station.source,
                 "由 standoff / offset 自动得到", "到灯心的直线距离 = sqrt(d^2 + u^2 + v^2)"),
        ParamRow("6. 无人机悬停站位（当前主场景）", "站位世界坐标（派生）", _v3(pos), station.source,
                 "HoverStationConfig.position(lamp)", f"站位 = ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) m"),
        ParamRow("6. 无人机悬停站位（当前主场景）", "SimConfig.body_safe_bounds（派生）",
                 str(sim.body_safe_bounds(uav.size_m)), MODELING_ASSUMPTION,
                 "由 uav_bounds 与 size_m 自动得到",
                 "center_min = body_min + half_size，center_max = body_max - half_size"),
    ]

    # ---- 7. 仿真与判定 --------------------------------------------------
    rows += [
        ParamRow("7. 仿真与判定", "SimConfig.dt", f"{sim.dt:g} s", sim.source_dt,
                 "config.SimConfig.dt / config.SIM_DT_S", "仿真步长"),
        ParamRow("7. 仿真与判定", "SimConfig.t_end", f"{sim.t_end:g} s", sim.source_t_end,
                 "config.SimConfig.t_end / config.SIM_T_END_S", "仿真时长（不含抛物线截断）"),
        ParamRow("7. 仿真与判定", "main.DART_STOP_DISTANCE_M", "0.20 m", ENGINEERING_ASSUMPTION,
                 "main.DART_STOP_DISTANCE_M",
                 "|C-U| 小于该值时截断演示（避免撞机/相机与灯心重合）；等效覆盖约 98% 飞行时间"),
        ParamRow("7. 仿真与判定", "experiment.INITIAL_NEAR_LAMP（当前工况）",
                 "initial_hover_near_lamp", ENGINEERING_ASSUMPTION,
                 "config.INITIAL_NEAR_LAMP", "无人机初始即在悬停站位上"),
    ]

    # ---- 9. 实验计划 ----------------------------------------------------
    # ---- 10. 可视化 -----------------------------------------------------
    rows += [
        ParamRow("10. 可视化 / 输出", "visualize.DPI", f"{VIS_DPI}", ENGINEERING_ASSUMPTION,
                 "visualize.DPI", "静态图 dpi"),
        ParamRow("10. 可视化 / 输出", "CLI --fps", "25", ENGINEERING_ASSUMPTION, "CLI --fps",
                 "动画帧率"),
        ParamRow("10. 可视化 / 输出", "CLI --video-dpi", "80", ENGINEERING_ASSUMPTION,
                 "CLI --video-dpi", "动画渲染 dpi（560x480；150 -> 1050x900）"),
        ParamRow("10. 可视化 / 输出", "main.OUT_DIR", "rm_uav_occlusion/out", ENGINEERING_ASSUMPTION,
                 "main.OUT_DIR", "全部输出目录"),
    ]
    return rows


def _dart_parameter_rows(dart: DartConfig) -> List[ParamRow]:
    """某个飞镖模式的**参数**条目。"""
    cat = f"4. 飞镖弹道参数（{dart.mode}）"
    src = dart.source_parabola if dart.is_parabolic else dart.source
    if dart.mode == "simple":
        return [
            ParamRow(cat, "DartConfig.launch", _v3(dart.launch), src,
                     "config.DartConfig.launch / config.LAUNCH_REL_LAMP_M",
                     "初始发射位置（机械数据换算）"),
            ParamRow(cat, "DartConfig.impact", _v3(dart.impact), src,
                     "config.DartConfig.impact", "末端命中位置（绿灯中心）"),
            ParamRow(cat, "DartConfig.v0_mps", f"{dart.v0_mps:g} m/s", src,
                     "config.SIMPLE_V0_MPS / config.DartConfig.v0_mps", "初始速度"),
            ParamRow(cat, "DartConfig.theta0_deg", f"{dart.theta0_deg:g} deg", src,
                     "config.SIMPLE_THETA0_DEG / config.DartConfig.theta0_deg", "初始仰角"),
            ParamRow(cat, "DartConfig.speed_decay_per_s", f"{dart.speed_decay_per_s:g} 1/s", src,
                     "config.SIMPLE_SPEED_DECAY_PER_S / config.DartConfig.speed_decay_per_s",
                     "速度降低率，v(t) = v0·e^(−k t)"),
            ParamRow(cat, "DartConfig.effective_g_mps2（派生）",
                     f"{dart.effective_g_mps2:.3f} m/s^2", src,
                     "由“抛物线必须穿过落点”反解", "z(x) = z0 + x·tanθ0 − g_eff·x²/(2v0²cos²θ0)"),
            ParamRow(cat, "DartConfig.simple_path_len_m（派生）",
                     f"{dart.simple_path_len_m():.3f} m", src, "弧长积分", "弹道弧长"),
            ParamRow(cat, "DartConfig.t_end（派生）", f"{dart.t_end:.3f} s", src,
                     "按减速律走完全程的时间", "仿真窗口与视频时长都取该值"),
        ]
    a, b = dart.parabolic_coeffs()
    return [
        ParamRow(cat, "DartConfig.launch", _v3(dart.launch), src, "config.DartConfig.launch",
                 "抛物线发射点"),
        ParamRow(cat, "DartConfig.impact", _v3(dart.impact), src, "config.DartConfig.impact",
                 "落点（默认 = 绿灯中心）"),
        ParamRow(cat, "DartConfig.t_flight", f"{dart.t_flight:g} s", src,
                 "config.DartConfig.t_flight", "总飞行时间"),
        ParamRow(cat, "DartConfig.apex_rise_m", f"{dart.apex_rise_m:g} m", src,
                 "config.DartConfig.apex_rise_m",
                 "顶点相对发射点的抬升高度"),
        ParamRow(cat, "DartConfig.apex_z_m（派生）", f"{dart.apex_z_m:.3f} m", src,
                 "由 launch.z + apex_rise_m 自动得到", "顶点相对灯平面的绝对高度"),
        ParamRow(cat, "DartConfig.g_effective_mps2", f"{dart.g_effective_mps2:g} m/s^2", src,
                 "config.DartConfig.g_effective_mps2",
                 "等效重力；=0 时用顶点高度反解抛物线"),
        ParamRow(cat, "DartConfig.constant_speed", str(bool(dart.constant_speed)), src,
                 "config.DartConfig.constant_speed",
                 "True = 沿弹道**恒定速率**飞行（按弧长反解位置，速度模长全程不变）；"
                 "False = 水平匀速 + 竖直抛体"),
        ParamRow(cat, "DartConfig.path_length_m（派生）", f"{dart.path_length_m:.4f} m", src,
                 "由抛物线弧长积分得到（config.ARC_LENGTH_SAMPLES 个采样点）",
                 "发射点到落点的弹道弧长"),
        ParamRow(cat, "DartConfig.speed_mps（派生）", f"{dart.speed_mps:.4f} m/s", src,
                 "= 弧长 / t_flight", "constant_speed=True 时的恒定速率（也是相机前馈参考）"),
    ]


# ---------------------------------------------------------------------------
# 运动学实测量（相机速度 / 距离 / 高度）
# ---------------------------------------------------------------------------


def dart_kinematics(
    dart: DartConfig,
    lamp: LampConfig,
    n_samples: int = 601,
) -> List[ParamRow]:
    """采样计算该弹道下**相机相对绿灯**的距离、高度与速度（即“实际效果”里的值）。"""
    t = np.linspace(0.0, dart.t_end, int(n_samples))
    C = np.asarray([dart_camera_trajectory(float(ti), dart) for ti in t], dtype=float)
    V = np.asarray([dart_camera_velocity(float(ti), dart) for ti in t], dtype=float)
    center = lamp.center_array
    dist = np.linalg.norm(C - center, axis=1)
    height = C[:, 2] - center[2]
    speed = np.linalg.norm(V, axis=1)
    cat = f"4b. 飞镖运动学实测（{dart.mode}）"
    src = dart.source_parabola if dart.is_parabolic else dart.source

    rows = [
        ParamRow(cat, "飞行时长 T = DartConfig.t_end", f"{dart.t_end:.3f} s", src, "—", ""),
        ParamRow(cat, "起点坐标 C(0)", _v3(C[0]), src, "由 DartConfig 起点给出", ""),
        ParamRow(cat, "终点坐标 C(T)", _v3(C[-1]), src, "由 DartConfig 终点给出", ""),
        ParamRow(cat, "相机起点距离 |C(0)-L|", f"{dist[0]:.3f} m", src, "—", "到灯心的直线距离"),
        ParamRow(
            cat, "相机终点距离 |C(T)-L|", f"{dist[-1]:.3f} m", src, "—",
            "抛物线落点即灯心（0 m）；实际演示在 |C-U| <= 0.20 m 处截断",
        ),
        ParamRow(cat, "相机最近距离 min|C-L|", f"{dist.min():.3f} m @ t={t[int(np.argmin(dist))]:.2f} s",
                 src, "—", ""),
        ParamRow(cat, "相机起点高度 z_C(0)-z_L", f"{height[0]:+.3f} m", src, "—", "相对灯心高度"),
        ParamRow(cat, "相机终点高度 z_C(T)-z_L", f"{height[-1]:+.3f} m", src, "—", ""),
        ParamRow(cat, "相机最高点 max z", f"{height.max():+.3f} m @ t={t[int(np.argmax(height))]:.2f} s",
                 src, "—", ""),
        ParamRow(cat, "相机速度 |v(0)|", f"{speed[0]:.3f} m/s", src, "—", ""),
        ParamRow(cat, "相机速度 |v(T)|", f"{speed[-1]:.3f} m/s", src, "—", ""),
        ParamRow(cat, "相机峰值速度 max|v|", f"{speed.max():.3f} m/s", src, "—", ""),
        ParamRow(cat, "水平速度 v_y（沿灯方向）", f"{V[0, 1]:.3f} m/s",
                 src, "—", "匀速接近段速度"),
        ParamRow(cat, "竖直速度 v_z(0)", f"{V[0, 2]:+.3f} m/s", src, "—", ""),
        ParamRow(cat, "竖直速度 v_z(T)", f"{V[-1, 2]:+.3f} m/s", src, "—",
                 "负值表示下降段（落向绿灯）"),
    ]
    if dart.is_parabolic:
        a, b = dart.parabolic_coeffs()
        rows += [
            ParamRow(cat, "顶点时刻 t_apex（派生）", f"{dart.t_apex:.3f} s", src,
                     "由 launch/apex_rise_m/t_flight 反解", ""),
            ParamRow(cat, "顶点高度（派生）", f"{dart.apex_height_m:.3f} m", src, "—", ""),
            ParamRow(cat, "抛物线系数 (a, b)（派生）", f"a = {a:.4f}, b = {b:.4f}", src,
                     "z(t) = z0 + a t - b t^2", ""),
            ParamRow(cat, "落点速度 |v(T)|（派生）", f"{dart.impact_speed_mps:.3f} m/s", src, "—",
                     "下降段落到绿灯中心时的速度"),
        ]
    return rows


# ---------------------------------------------------------------------------
# 输出
# ---------------------------------------------------------------------------


def to_frame(rows: Sequence[ParamRow]) -> pd.DataFrame:
    """转为 DataFrame（列名用 ASCII，便于程序化读取/二次处理）。"""
    return pd.DataFrame(
        [
            {
                "category": r.category,
                "name": r.name,
                "value": r.value,
                "source": r.source,
                "how_to_change": r.how_to_change,
                "note": r.note,
            }
            for r in rows
        ]
    )


def print_catalog(rows: Sequence[ParamRow], show_full_source: bool = False) -> None:
    """按类别打印参数表（控制台友好：来源用短标签、长取值截断，完整值见 CSV）。"""
    if len(rows) == 0:
        return
    name_w = min(46, max(display_width(r.name) for r in rows) + 1)
    value_w = min(46, max(display_width(r.value) for r in rows) + 1)
    header = pad("参数名", name_w) + pad("当前取值", value_w) + pad("来源", 6) + "改动方式"
    current = None
    for r in rows:
        if r.category != current:
            current = r.category
            print(f"\n  【{current}】")
            print("    " + header)
        src = r.source if show_full_source else short_source(r.source)
        print(
            "    "
            + pad(r.name, name_w, truncate=True)
            + pad(r.value, value_w, truncate=True)
            + pad(src, 6)
            + r.how_to_change
        )


def to_markdown(rows: Sequence[ParamRow], title: str = "参数总表") -> str:
    lines: List[str] = [f"# {title}", ""]
    lines.append("> 由 `python main.py` 自动生成；`参数名` 即代码中的字段，可直接搜索定位。")
    lines.append("")
    categories: List[str] = []
    for r in rows:
        if r.category not in categories:
            categories.append(r.category)
    for cat in categories:
        lines.append(f"## {cat}")
        lines.append("")
        lines.append("| 参数名 | 当前取值 | 来源 | 改动方式 | 备注 |")
        lines.append("|---|---|---|---|---|")
        for r in rows:
            if r.category != cat:
                continue
            note = r.note.replace("|", "/")
            lines.append(f"| `{r.name}` | {r.value} | `{r.source}` | {r.how_to_change} | {note} |")
        lines.append("")
    return "\n".join(lines)


def write_reports(
    rows: Sequence[ParamRow],
    out_dir: Union[str, Path],
    stem: str = "parameters",
) -> Tuple[Path, Path]:
    """写出 ``parameters.md`` 与 ``parameters.csv``。"""
    directory = Path(out_dir)
    directory.mkdir(parents=True, exist_ok=True)
    md_path = directory / f"{stem}.md"
    csv_path = directory / f"{stem}.csv"
    md_path.write_text(to_markdown(rows), encoding="utf-8")
    to_frame(rows).to_csv(csv_path, index=False, encoding="utf-8-sig")
    return md_path, csv_path
