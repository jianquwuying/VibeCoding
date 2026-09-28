"""全局配置与参数来源标签。

本模块集中定义所有可配置参数（几何尺寸、速度上限、仿真步长、边界等），
其它模块不得硬编码物理量。每个参数都通过 ``source_*`` 字段标注来源：

======================  ==================================================
来源标签                 含义
======================  ==================================================
OFFICIAL_2027           《RoboMaster 2027 高校系列赛规则变更前瞻手册》(2026-09-09) 明确给出
OFFICIAL_2026_BASELINE  2026 正式规则给出，本项目作为 baseline 使用
ENGINEERING_ASSUMPTION  工程假设（速度、加速度、步长、分析阈值等）
MODELING_ASSUMPTION     几何 / 建模假设（灯姿态、局部坐标系等）
USER_SPEC               用户设备设定（分辨率、焦距、机身尺寸等）
UNKNOWN                 未知，待正式规则或实测确认（例如传感器靶面尺寸）
======================  ==================================================

本项目定位为**几何可行性研究**：只回答“无人机是否在几何上遮挡了绿色引导灯”，
不代表已获裁判确认的合法战术，也不模拟飞镖空气动力学。

坐标系（与机械组数据一致）
--------------------------
以**绿色引导灯几何中心**为原点：

* ``X``：从发射点指向基地，沿飞行距离方向（弹道沿 ``+X`` 前进）
* ``Y``：场地横向
* ``Z``：竖直向上

机械装配数据 [mm]：绿灯中心 ``(25037.05, 2868.18, 1139.04)``、发射点
``(0, 0, 570.00)``；两者相减得到发射点在灯局部坐标系下的位置
``(-25.03705, -2.86818, -0.56904) m``（距灯心 25.20 m、横向 -2.87 m、
低于灯心 0.569 m——**不是离地高度**）。灯球所在平面为 ``X = 0``，法线
``(-1, 0, 0)``，朝向迎面而来的飞镖。
"""

from __future__ import annotations

import math
import warnings
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# 类型别名与来源标签
# ---------------------------------------------------------------------------

Vector3 = Tuple[float, float, float]
Interval = Tuple[float, float]
Bounds = Mapping[str, Interval]

OFFICIAL_2027: str = "OFFICIAL_2027"
OFFICIAL_2026_BASELINE: str = "OFFICIAL_2026_BASELINE"
ENGINEERING_ASSUMPTION: str = "ENGINEERING_ASSUMPTION"
MODELING_ASSUMPTION: str = "MODELING_ASSUMPTION"
USER_SPEC: str = "USER_SPEC"
UNKNOWN: str = "UNKNOWN"

SOURCE_LEGEND: Dict[str, str] = {
    OFFICIAL_2027: "RoboMaster 2027 前瞻手册明确给出",
    OFFICIAL_2026_BASELINE: "2026 正式规则给出（baseline）",
    ENGINEERING_ASSUMPTION: "工程假设",
    MODELING_ASSUMPTION: "几何 / 建模假设",
    USER_SPEC: "用户设备设定",
    UNKNOWN: "未知，待正式规则或实测确认",
}

# ---------------------------------------------------------------------------
# 顶层常量（默认场景取值）
# ---------------------------------------------------------------------------

#: 机械装配坐标系下的原始数据 [mm]（用户提供）。
# ---- 调参：机械装配原始数据（唯一录入点）--------------------------------------
# 单位 mm，来自机械组总装图。下方所有坐标（含发射点相对灯心的位置）都由这两个数相减推导。
#   绿灯中心：机械装配系下的灯几何中心
#   发射点  ：机械装配系下的飞镖出膛点
# 改动影响：同时平移灯局部坐标系原点与发射点 → 影响站位坐标、弹道、遮挡几何。
# 修改建议：机械组更新数据时只改这两行，其它文件不必动；不要把这里写成"相对灯心"的坐标。
LAMP_CENTER_MECH_MM: Vector3 = (25037.05, 2868.18, 1139.04)
LAUNCH_POINT_MECH_MM: Vector3 = (0.0, 0.0, 570.00)


def _mech_to_lamp_local(point_mech_mm: Vector3) -> Vector3:
    """机械装配坐标 [mm] -> 以绿灯中心为原点的仿真坐标 [m]。"""
    return tuple(
        (float(point_mech_mm[i]) - float(LAMP_CENTER_MECH_MM[i])) / 1000.0 for i in range(3)
    )


#: 发射点在灯局部坐标系下的位置 [m]：(-25.03705, -2.86818, -0.56904)。
LAUNCH_REL_LAMP_M: Vector3 = _mech_to_lamp_local(LAUNCH_POINT_MECH_MM)

#: 无人机：**细长长方体**机体（长 × 宽 × 高），长边沿弹道方向（+X）。
#: 迎面（+X 方向）投影截面 100 × 80 mm > 灯球 55 mm，仍能形成完整遮挡。
# ---- 调参：无人机与相机（默认单值）--------------------------------------------
# 机体三轴尺寸 [mm]（用户设定；v3 起三轴独立：长=X、宽=Y、高=Z）。直接决定阴影大小与机身安全边界：
#   机身越大 → 阴影越大 → 越容易全遮灯（R_occ 越大、可用站位区间越宽）。
#   机身安全边界 = DEFAULT_UAV_BOUNDS ∓ 半尺寸，故改尺寸会同时改变可用站位范围。
#   v3 默认 (100, 100, 80)：高度 80 mm 时最紧时刻（t≈0.75 s）灯球会露出 ≈2.5% 面积，
#   即"默认不再完全遮挡"（R_min ≈ 0.974843、minMargin ≈ −3.36 mm）。
UAV_SIZE_MM: Vector3 = (100.0, 100.0, 80.0)

# 灯发光直径 [mm]（2026 规则 baseline）。R_occ 的分母与"影子要盖多大"都由它决定：
#   直径越小 → 越容易被遮住（R_occ 越大）；55 mm → 半径 27.5 mm、面积 2.376e-3 m²。
LAMP_DIAMETER_MM: float = 55.0
# 最大速度 [m/s]：只影响"从起始位到悬停站位"的到位过程（悬停后不再移动），不影响稳态遮挡。
UAV_MAX_SPEED_MPS: float = 5.0
# 最大加速度 [m/s²]：同上；越小到位越慢，前 1~2 s 的轨迹越平缓。
UAV_MAX_ACCEL_MPS2: float = 2.0
# 相机分辨率 (W, H)：只用于视场判定与 FPV 成像像素大小，**不参与 R_occ 计算**。
CAMERA_RESOLUTION: Tuple[int, int] = (640, 480)
# 水平 FOV [deg]：工程假设（缺靶面尺寸，不允许用焦距反推 FOV）。
#   调大 → 更容易把灯框进画面（视场余量变大）；调小 → 末端可能出画。
CAMERA_HFOV_DEG: float = 70.0
# 仿真步长 [s]：0.02 s = 50 Hz。决定曲线采样密度，也是 T_occ = sum(eff)*dt 里的 dt。
SIM_DT_S: float = 0.01
# 仿真时长上限 [s]：主场景会被"弹道飞行时间（≈1.92 s）"与"接触前截断"覆盖，故这里只是上限。
SIM_T_END_S: float = 5.0

A_MAX_OPTION: float = 2.0

# ---------------------------------------------------------------------------
# M2 模型常量（v2）
# ---------------------------------------------------------------------------

#: 重力加速度 [m/s^2]（M2 六步积分用，替代 v1 的"等效重力"反解）。
GRAVITY_MPS2: float = 9.8
#: 追踪段最大转弯率 [deg/s]（尾翼转向上限）。
MAX_TURN_RATE_DPS: float = 60.0
#: 命中判定距离 [m]：线段（P_i -> P_{i+1}）到灯心的最近距离阈值。
HIT_DISTANCE_M: float = 0.05
#: 灯 Y 方向偏移的物理限位 [mm]。
LAMP_Y_RANGE_MM: Tuple[float, float] = (-240.0, 240.0)
#: 灯横向平移的开始 / 到位时刻 [s]。
LAMP_MOVE_START_S: float = 1.2
LAMP_MOVE_END_S: float = 1.8
#: 灯球半径 [m]（v2：灯由平面圆盘升级为球体，半径沿用 55 mm 直径）。
LAMP_SPHERE_RADIUS_M: float = LAMP_DIAMETER_MM / 2000.0

#: 无人机可行域（机身包围盒）：覆盖“发射点 -> 灯”的整条接近走廊。
# ---- 调参：飞行边界（机身包围盒，单位 m）--------------------------------------
# 机身**整体**必须落在该盒内；机身中心可用范围 = 该盒 ∓ 半尺寸（SimConfig.body_safe_bounds）。
# 当前覆盖"发射点 → 灯"整条走廊：x∈[-26,0.5]（25 m 弹道）、y∈[-6,6]、z∈[-2,3]。
# 收窄 x 会让无人机无法在远处就位；把 z 下界抬高（例如 0.06）会强制悬停高度、
# 破坏"机身中心与灯心等高"的最优站位（实测 R_occ 会从 0.970 掉到 0.453）。
DEFAULT_UAV_BOUNDS: Dict[str, Interval] = {
    "x": (-26.0, 0.5),
    "y": (-6.0, 6.0),
    "z": (-2.0, 3.0),
}


#: 预测策略的预测时域与采样数（ENGINEERING_ASSUMPTION）。

#: 视线跟踪比例：目标点位于相机 -> 灯连线上 ratio 处（ENGINEERING_ASSUMPTION）。

#: 追踪控制器参数（ENGINEERING_ASSUMPTION）：加速度受限的 PD 律
#: ``a_cmd = Kp * e + Kd * (v_ff - v)``，其中 ``Kp = omega^2``、``Kd = 2 * zeta * omega``。
#: 期望加速度直接受 Amax 限制，因此 ``|dv| <= Amax * dt`` 由构造保证。

#: 飞镖相机轨迹模式（v1 的 ``parabolic`` 已删除，M2 只保留数值积分模式）。
DART_MODE_SIMPLE: str = "simple"
DART_MODES: Tuple[str, ...] = (
    DART_MODE_SIMPLE,
)

#: 最简抛物线模型参数（当前主场景）：只由 5 个量决定一条弹道
#: 发射点、落点、初速 v0、初始仰角 theta0、速度降低率 k。
# ---- 调参（★最常改）：最简弹道 3 个输入量 -------------------------------------
# 弹道 = 抛物线几何（由 发射点/落点/v0/仰角 决定）+ 沿弧长的减速律（由 k 决定时间）。
#   v0 [m/s]：出膛速度。改大 → 同距离处更高更快（等效重力反解会随之变化）。
#   theta0 [deg]：初始仰角。是"打高/打低"的主要旋钮；当前 26° 是让抛物线正好穿过灯心的解。
#   k [1/s]：速度降低率，v(t)=v0·e^(−kt)。改大 → 后段更慢、飞行时间更长、末端速度更低。
# 三个量一起决定：飞行时间 t_end、末端速度（进而在标题里体现）、以及弹道的弯折程度。
SIMPLE_V0_MPS: float = 20.0
SIMPLE_THETA0_DEG: float = 30.0
SIMPLE_SPEED_DECAY_PER_S: float = 0.55

# ---------------------------------------------------------------------------
# 2026 有控飞镖：报告给出的“工程级质点飞行模型”参数（ENGINEERING_ASSUMPTION / 报告数据）
#: 相机光轴基准“上方向”：光轴取弹道切线，用该向量构造相机姿态矩阵。
# ---- 调参：相机"上方向"参考向量 ------------------------------------------------
# 光轴固定取弹道切线 dC/dt；本向量只用于确定滚转（画面绕光轴的旋转），不影响 R_occ。
# 一般保持世界 +Z；若光轴接近竖直，代码会自动换备用参考轴。
CAMERA_UP_REF: Vector3 = (0.0, 0.0, 1.0)

#: 抛物线弧长表的采样点数（越大越精确；表按 DartConfig 缓存）。

#: 默认悬停站位：到灯平面距离 0.15 m、灯平面内横向偏置 (u=0, v=0.06) m。
#: （机身安全下界为 z = 0.06 m，故 v 偏置取该值；站位定义见 ``HoverStationConfig``。）
#: 追踪控制器参数（加速度限并的 PD 律）。

#: 默认悬停站位：到灯平面距离 0.15 m、灯平面内横向偏置 (u=0, v=0) m。
#: v 是灯平面内的竖直偏置，v = 0 表示机身中心与灯心等高；
# ---- 调参（★最常改）：悬停站位 ------------------------------------------------
# 站位 = 灯心 + standoff·n̂ + offset_u·û + offset_v·v̂（n̂ 指向相机侧，v̂ 是灯平面内竖直方向）。
#   standoff [m]：机身中心到**灯平面**的距离（0.15 m）。是目前最敏感的参数：
#       实测可行区间 standoff ∈ [0.08, 0.16] m（R_min ≥ 0.7）；<0.08 机身前缘触到灯面，
#       >0.16 末端阴影中心偏移发散、灯球被甩出阴影。
#   offset_u [m]：灯平面内横向偏置（默认 0，机身正对灯心）。
#   offset_v [m]：灯平面内竖直偏置。**默认 0 = 机身中心与灯心等高**（这是最优解）：
#       实测 v=0.06 会把阴影中心抬高 6 cm，R_occ 从 0.970 掉到 0.453（失效）。
HOVER_STATION_STANDOFF_M: float = 0.15
HOVER_STATION_OFFSET_V_M: float = 0.00

#: 策略名。

#: 初始工况名。
INITIAL_NEAR_LAMP: str = "initial_hover_near_lamp"

#: ``initial_off_LOS`` 相对视线点的偏移量（ENGINEERING_ASSUMPTION）。


# ---------------------------------------------------------------------------
# 灯
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class LampConfig:
    """绿色引导灯模型：**球体**（半径 27.5 mm），沿 Y 在 ±240 mm 内平移。

    灯球中心始终位于 ``X = 0`` 平面内（``center`` 的 ``y`` 分量即灯位）；
    ``X = 0`` 只是局部坐标取值，与地面高度无关。``normal`` 仅作平面/网格占位——
    球体无朝向，遮挡计算一律用 ``los_basis`` 的逐帧视线法线。
    """

    center: Vector3 = (0.0, 0.0, 0.0)
    normal: Vector3 = (-1.0, 0.0, 0.0)
    diameter_mm: float = LAMP_DIAMETER_MM
    source_diameter: str = OFFICIAL_2026_BASELINE
    source_pose: str = MODELING_ASSUMPTION

    def __post_init__(self) -> None:
        if not math.isfinite(self.diameter_mm) or self.diameter_mm <= 0.0:
            raise ValueError(f"灯直径必须 > 0，当前为 {self.diameter_mm!r} mm")
        if float(np.linalg.norm(np.asarray(self.normal, dtype=float))) <= 1e-12:
            raise ValueError("灯法线不能是零向量")

    # -- 派生量 ---------------------------------------------------------

    @property
    def radius_m(self) -> float:
        """灯发光部分半径 [m]。"""
        return self.diameter_mm / 2000.0

    @property
    def area_m2(self) -> float:
        """灯发光部分面积 [m^2] = pi * r^2。"""
        return math.pi * self.radius_m ** 2

    @property
    def center_array(self) -> np.ndarray:
        return np.asarray(self.center, dtype=float)

    @property
    def normal_array(self) -> np.ndarray:
        n = np.asarray(self.normal, dtype=float)
        return n / float(np.linalg.norm(n))

    def source_rows(self) -> List[Tuple[str, str, str]]:
        """返回 ``(参数, 取值, 来源标签)`` 行，便于打印。"""
        return [
            ("lamp.diameter_mm", f"{self.diameter_mm:g}", self.source_diameter),
            ("lamp.center/normal", f"{self.center} / {self.normal}", self.source_pose),
        ]


@dataclass(frozen=True)
class HoverStationConfig:
    """无人机**悬停站位**：相对绿灯的特定位置（本项目当前唯一启用的机动方式）。

    站位以灯的局部坐标系描述::

        position = L + standoff_m * n_hat + offset_u_m * u_hat + offset_v_m * v_hat

    其中 ``n_hat`` 为灯法线（默认 ``(-1,0,0)``，指向相机一侧）、``(u_hat, v_hat)``
    为灯平面内正交基（默认 ``(1,0,0)`` 与 ``(0,0,1)``）。

    * ``standoff_m``：沿灯法线到**灯平面**的距离；
    * ``offset_u_m`` / ``offset_v_m``：灯平面内的横向偏置（决定阴影中心相对灯心的偏移）；
    * ``distance_to_lamp_m``：到灯心的**直线距离** ``sqrt(standoff^2 + u^2 + v^2)``。

    站位是遮挡能否成立的主要设计量：机身越靠近灯平面，阴影中心越接近机身自身的
    横向位置，对相机运动越不敏感（详见 README 第 9/10 节）。
    """

    standoff_m: float = HOVER_STATION_STANDOFF_M
    offset_u_m: float = 0.0
    offset_v_m: float = HOVER_STATION_OFFSET_V_M
    source: str = ENGINEERING_ASSUMPTION

    def __post_init__(self) -> None:
        if not (self.standoff_m > 0.0):
            raise ValueError(f"站位到灯平面的距离必须为正，当前 {self.standoff_m!r} m")

    # -- 派生量 ---------------------------------------------------------

    @property
    def distance_to_lamp_m(self) -> float:
        """站位到灯心的直线距离 [m]。"""
        return math.sqrt(self.standoff_m ** 2 + self.offset_u_m ** 2 + self.offset_v_m ** 2)

    def position(self, lamp: LampConfig) -> np.ndarray:
        """站位在世界坐标下的位置 [m]。"""
        from geometry import plane_basis  # 延迟导入，避免模块循环依赖

        u_hat, v_hat = plane_basis(lamp.normal_array)
        return (
            lamp.center_array
            + float(self.standoff_m) * lamp.normal_array
            + float(self.offset_u_m) * u_hat
            + float(self.offset_v_m) * v_hat
        )

    @classmethod
    def from_distance(
        cls,
        distance_m: float,
        offset_u_m: float = 0.0,
        offset_v_m: float = HOVER_STATION_OFFSET_V_M,
        source: str = ENGINEERING_ASSUMPTION,
    ) -> "HoverStationConfig":
        """按“到灯心的直线距离”构造站位（法向距离由勾股关系反解）。"""
        lateral2 = float(offset_u_m) ** 2 + float(offset_v_m) ** 2
        if float(distance_m) ** 2 <= lateral2:
            raise ValueError(
                f"距灯心 {distance_m} m 必须大于横向偏置范数 {math.sqrt(lateral2):.4f} m"
            )
        return cls(
            standoff_m=math.sqrt(float(distance_m) ** 2 - lateral2),
            offset_u_m=float(offset_u_m),
            offset_v_m=float(offset_v_m),
            source=source,
        )

    def source_rows(self) -> List[Tuple[str, str, str]]:
        return [
            (
                "hover.station",
                f"法向 {self.standoff_m:.3f} m, 横向 (u={self.offset_u_m:+.3f}, "
                f"v={self.offset_v_m:+.3f}) m, 距灯心 {self.distance_to_lamp_m:.3f} m",
                self.source,
            ),
        ]


#: 默认悬停站位（ENGINEERING_ASSUMPTION）：距灯平面 0.15 m、v 偏置 0.06 m
#: （= 机身安全下界，v 偏置即机身中心高度）。
DEFAULT_HOVER_STATION: HoverStationConfig = HoverStationConfig()


# ---------------------------------------------------------------------------
# 相机（不含固定位置：相机位置由 DartConfig 的 C(t) 给出）
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CameraConfig:
    """相机 / 飞镖相机内参模型。

    注意：本模块**不定义相机位置**，相机位置由 :class:`DartConfig` 通过 ``C(t)`` 给出。

    焦距 ``focal_length_mm`` 仅作为实物记录；由于缺少传感器靶面尺寸（``UNKNOWN``），
    不允许用焦距反推 FOV，FOV 一律取 ``fov_h_deg``。
    """

    resolution: Tuple[int, int] = CAMERA_RESOLUTION
    fov_h_deg: float = CAMERA_HFOV_DEG
    focal_length_mm: float = 3.6
    distortion_enabled: bool = False
    source_resolution: str = USER_SPEC
    source_fov: str = ENGINEERING_ASSUMPTION
    source_focal: str = USER_SPEC

    def __post_init__(self) -> None:
        if len(self.resolution) != 2 or min(self.resolution) <= 0:
            raise ValueError(f"分辨率非法：{self.resolution!r}")
        if not (0.0 < self.fov_h_deg < 180.0):
            raise ValueError(f"水平 FOV 非法：{self.fov_h_deg!r} 度")
        if self.focal_length_mm <= 0.0:
            raise ValueError(f"焦距非法：{self.focal_length_mm!r} mm")

    @property
    def fov_h_rad(self) -> float:
        return math.radians(self.fov_h_deg)

    @property
    def aspect(self) -> float:
        return self.resolution[0] / self.resolution[1]

    @property
    def fov_v_deg(self) -> float:
        """由水平 FOV 与画幅比例推导的垂直 FOV（ENGINEERING_ASSUMPTION）。"""
        return math.degrees(2.0 * math.atan(math.tan(self.fov_h_rad / 2.0) / self.aspect))

    @property
    def focal_px(self) -> float:
        """等效像素焦距 f_x = (W/2) / tan(HFOV/2)，仅用于可选的图像平面可视化。"""
        return (self.resolution[0] / 2.0) / math.tan(self.fov_h_rad / 2.0)

    def source_rows(self) -> List[Tuple[str, str, str]]:
        return [
            ("camera.resolution", f"{self.resolution[0]}x{self.resolution[1]}", self.source_resolution),
            ("camera.fov_h_deg", f"{self.fov_h_deg:g}", self.source_fov),
            ("camera.focal_length_mm", f"{self.focal_length_mm:g} (不参与 FOV 推导；靶面尺寸 UNKNOWN)", self.source_focal),
        ]


# ---------------------------------------------------------------------------
# 无人机
# ---------------------------------------------------------------------------


def _default_uav_mass() -> float:
    return 0.1755

@dataclass(frozen=True)
class UAVConfig:
    """无人机几何与性能模型（MVP：轴对齐**细长长方体**，忽略圆角）。"""

    size_m: Vector3 = (UAV_SIZE_MM[0] / 1000.0, UAV_SIZE_MM[1] / 1000.0, UAV_SIZE_MM[2] / 1000.0)
    corner_radius_m: float = 0.0
    max_speed: float = UAV_MAX_SPEED_MPS
    max_accel: float = UAV_MAX_ACCEL_MPS2
    mass_kg: float = field(default_factory=_default_uav_mass)
    source_size: str = USER_SPEC
    source_mass: str = OFFICIAL_2027
    source_speed: str = ENGINEERING_ASSUMPTION
    source_accel: str = ENGINEERING_ASSUMPTION
    source_mass_model: str = OFFICIAL_2027

    def __post_init__(self) -> None:
        if len(self.size_m) != 3 or min(self.size_m) <= 0.0:
            raise ValueError(f"无人机尺寸非法：{self.size_m!r} m")
        if self.corner_radius_m < 0.0:
            raise ValueError("圆角半径不能为负")
        if self.max_speed <= 0.0 or self.max_accel <= 0.0:
            raise ValueError("Vmax / Amax 必须为正")
        if not (0.15 <= self.mass_kg <= 0.249):
            # 质量公式适用范围 y ∈ [0.15, 0.249]，超出范围时给出显式告警。
            warnings.warn(
                f"质量 {self.mass_kg:.4f} kg 超出 RM2027 经验公式适用范围 [0.15, 0.249] kg",
                stacklevel=2,
            )

    # -- 官方质量经验公式 ------------------------------------------------

    @staticmethod
    def weight_from_size_mm(size_mm: float) -> float:
        """RM2027 前瞻手册给出的最大重量经验公式：y = -0.001225x + 0.3225。

        参数
        ----
        size_mm : 最大伸展尺寸 [mm]

        返回
        ----
        最大重量 [kg]；适用范围内 ``size_mm`` 约为 60 ~ 140.8 mm。
        """
        return -0.001225 * float(size_mm) + 0.3225

    @staticmethod
    def mass_valid_size_range_mm() -> Tuple[float, float]:
        """由 y ∈ [0.15, 0.249] kg 反解出的尺寸适用范围 [mm]。"""
        lo = (0.3225 - 0.249) / 0.001225
        hi = (0.3225 - 0.15) / 0.001225
        return (lo, hi)

    @property
    def half_size_m(self) -> np.ndarray:
        return np.asarray(self.size_m, dtype=float) / 2.0

    @property
    def volume_m3(self) -> float:
        return float(np.prod(np.asarray(self.size_m, dtype=float)))

    def source_rows(self) -> List[Tuple[str, str, str]]:
        size_mm = max(self.size_m) * 1000.0
        return [
            ("uav.size_m", f"{self.size_m} m ({size_mm:g} mm)", self.source_size),
            ("uav.mass_kg", f"{self.mass_kg:.4f} kg = -0.001225*{size_mm:g} + 0.3225", self.source_mass),
            ("uav.max_speed", f"{self.max_speed:g} m/s", self.source_speed),
            ("uav.max_accel", f"{self.max_accel:g} m/s^2", self.source_accel),
        ]


# ---------------------------------------------------------------------------
# 飞镖相机轨迹
# ---------------------------------------------------------------------------


#: 追踪控制器参数（加速度限并的 PD 律）。
# ---- 调参：到位控制器（只影响"飞向站位"的过程，不影响稳态遮挡）----------------
# 加速度受限 PD 律：a_cmd = Kp·e + Kd·(v_ff − v)，Kp = ω²、Kd = 2ζω。
#   ω 越大到位越快（但过大可能引起离散振荡）；ζ = 1 为临界阻尼（不过冲）。
#   经验：默认 ω=5、ζ=1、dt=0.02 s 时离散稳定；若改大 dt，请保证 Kd·dt ≤ 1.5。
TRACKING_OMEGA_RAD_S: float = 5.0
TRACKING_ZETA: float = 1.0

@dataclass(frozen=True)
class DartConfig:
    """飞镖相机**初始条件与阻力参数**容器（M2）。

    M2 的轨迹由 ``motion.step_dart_m2`` 逐帧数值积分给出（重力 + 线性阻力 +
    视线追击 + ``ω_max`` 限幅），本类只提供积分所需的输入：
    ``launch``（发射点，灯局部坐标系）、``v0_mps``、``theta0_deg``（初始仰角）、
    ``speed_decay_per_s``（阻力系数 k）。

    v1 的**解析抛物线模型已删除**（``parabolic`` 模式、``parabolic_*`` 方法与
    ``motion.dart_camera_*``）——它与 M2 是两套互相矛盾的弹道模型。
    ``mode`` 及下列 v1 元数据字段（``impact`` / ``t_flight`` / ``apex_rise_m`` /
    ``g_effective_mps2`` / ``constant_speed``）**仅为兼容上游构造签名而保留**，
    M2 不读取它们。
    """

    mode: str = DART_MODE_SIMPLE
    source: str = ENGINEERING_ASSUMPTION
    launch: Vector3 = LAUNCH_REL_LAMP_M
    # ---- 以下 5 个字段是 v1 抛物线元数据，M2 不再使用（保留以兼容上游签名）----
    impact: Vector3 = (0.0, 0.0, 0.0)
    t_flight: float = 3.0
    apex_rise_m: float = 1.0
    g_effective_mps2: float = 0.0
    constant_speed: bool = True
    # ---- M2 积分参数（gen_golden 直接读取）--------------------------------
    v0_mps: float = 20.0
    theta0_deg: float = SIMPLE_THETA0_DEG
    speed_decay_per_s: float = SIMPLE_SPEED_DECAY_PER_S
    source_parabola: str = ENGINEERING_ASSUMPTION

    def __post_init__(self) -> None:
        if self.mode not in DART_MODES:
            raise ValueError(f"未知飞镖模式 {self.mode!r}，可选 {DART_MODES}")

    @property
    def is_simple(self) -> bool:
        return self.mode == DART_MODE_SIMPLE

    @property
    def keypoints(self) -> List[Tuple[float, Vector3]]:
        """按时序排列的 ``(t [s], (x, y, z) [m])`` 关键点。

        M2 的轨迹由数值积分给出、没有解析路径，因此这里只返回发射点作为占位。
        """
        return [(0.0, tuple(float(x) for x in self.launch))]

    @property
    def t_start(self) -> float:
        return self.keypoints[0][0]

    @property
    def t_end(self) -> float:
        """M2 积分时域上限 [s]（命中由线段-球判据决定，与解析飞行时间无关）。"""
        return float(SIM_T_END_S)

    def source_rows(self) -> List[Tuple[str, str, str]]:
        kp = " -> ".join(f"C({t:g})={pos}" for t, pos in self.keypoints)
        return [("dart.mode/keypoints", f"{self.mode}: {kp}", self.source)]


# ---------------------------------------------------------------------------
# 仿真配置
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SimConfig:
    """仿真与实验配置。

    ``uav_bounds`` 是**机身**允许占据的轴对齐包围盒，单位为米；机身安全边界
    （body-safe bounds，即机身中心允许范围）由 :meth:`body_safe_bounds` 计算：

        center_min = body_min + half_size
        center_max = body_max - half_size
    """

    R_th: float = 0.7
    dt: float = SIM_DT_S
    t_end: float = SIM_T_END_S
    uav_bounds: Bounds = field(default_factory=lambda: dict(DEFAULT_UAV_BOUNDS))
    #: M2：灯的 Y 方向目标偏移 [m]（0 表示灯静止在原点）。
    lamp_target_y_m: float = 0.0
    source_R_th: str = ENGINEERING_ASSUMPTION
    source_dt: str = ENGINEERING_ASSUMPTION
    source_t_end: str = ENGINEERING_ASSUMPTION
    source_bounds: str = MODELING_ASSUMPTION

    def __post_init__(self) -> None:
        if not (0.0 <= self.R_th <= 1.0):
            raise ValueError(f"R_th 必须位于 [0, 1]，当前为 {self.R_th!r}")
        if self.dt <= 0.0:
            raise ValueError("dt 必须为正")
        if self.t_end <= 0.0:
            raise ValueError("t_end 必须为正")
        for axis in ("x", "y", "z"):
            if axis not in self.uav_bounds:
                raise ValueError(f"uav_bounds 缺少轴 {axis!r}")
            lo, hi = self.uav_bounds[axis]
            if not (lo < hi):
                raise ValueError(f"uav_bounds[{axis!r}] 非法：({lo}, {hi})")

    # -- 派生量 ---------------------------------------------------------

    @property
    def n_steps(self) -> int:
        return int(round(self.t_end / self.dt))

    @property
    def time_array(self) -> np.ndarray:
        """均匀时间序列，保证末端恰好等于 ``t_end``。"""
        n = self.n_steps
        return np.linspace(0.0, n * self.dt, n + 1)

    def body_safe_bounds(self, uav_size_m: Sequence[float]) -> Dict[str, Interval]:
        """由机身包围盒与机身尺寸计算机身中心允许范围。"""
        half = np.asarray(uav_size_m, dtype=float) / 2.0
        if half.shape != (3,):
            raise ValueError(f"uav_size_m 必须是 3 维，当前 {uav_size_m!r}")
        out: Dict[str, Interval] = {}
        for i, axis in enumerate(("x", "y", "z")):
            lo, hi = self.uav_bounds[axis]
            c_lo = float(lo + half[i])
            c_hi = float(hi - half[i])
            if c_lo > c_hi:
                mid = 0.5 * (float(lo) + float(hi))
                warnings.warn(
                    f"轴 {axis} 上机身尺寸 {2 * half[i]:.4f} m 超过可行范围 ({lo}, {hi})，"
                    f"机身安全边界退化为中点 {mid:.4f} m",
                    stacklevel=2,
                )
                c_lo = c_hi = mid
            out[axis] = (c_lo, c_hi)
        return out

    def bounds_array(self) -> np.ndarray:
        """返回形状 (3, 2) 的机身包围盒数组。"""
        return np.asarray([self.uav_bounds[a] for a in ("x", "y", "z")], dtype=float)

    def source_rows(self) -> List[Tuple[str, str, str]]:
        return [
            ("sim.R_th", f"{self.R_th:g}", self.source_R_th),
            ("sim.dt", f"{self.dt:g} s", self.source_dt),
            ("sim.t_end", f"{self.t_end:g} s", self.source_t_end),
            ("sim.uav_bounds", str(dict(self.uav_bounds)), self.source_bounds),
        ]


# ---------------------------------------------------------------------------
# 默认场景工厂
# ---------------------------------------------------------------------------


def default_lamp() -> LampConfig:
    return LampConfig()


def default_camera() -> CameraConfig:
    return CameraConfig()


def default_uav(max_speed: float = UAV_MAX_SPEED_MPS, max_accel: float = UAV_MAX_ACCEL_MPS2) -> UAVConfig:
    size_m = (UAV_SIZE_MM[0] / 1000.0, UAV_SIZE_MM[1] / 1000.0, UAV_SIZE_MM[2] / 1000.0)
    return UAVConfig(
        size_m=size_m,
        corner_radius_m=0.0,
        max_speed=max_speed,
        max_accel=max_accel,
        mass_kg=0.1755,
    )


def default_dart(mode: str = DART_MODE_SIMPLE) -> DartConfig:
    return DartConfig(mode=mode)


def default_sim(R_th: float = 0.7) -> SimConfig:
    return SimConfig(R_th=R_th, dt=SIM_DT_S, t_end=SIM_T_END_S, uav_bounds=dict(DEFAULT_UAV_BOUNDS))


def scene_source_table(
    lamp: LampConfig,
    camera: CameraConfig,
    uav: UAVConfig,
    dart: DartConfig,
    sim: SimConfig,
) -> List[Tuple[str, str, str]]:
    """汇总当前场景全部参数的来源标签。"""
    rows: List[Tuple[str, str, str]] = []
    for cfg in (lamp, camera, uav, dart, sim):
        rows.extend(cfg.source_rows())
    return rows
