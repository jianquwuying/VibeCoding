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
低于灯心 0.569 m——**不是离地高度**）。灯盘所在平面为 ``X = 0``，法线
``(-1, 0, 0)``，朝向迎面而来的飞镖。
"""

from __future__ import annotations

import math
import warnings
from functools import lru_cache
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
#: 迎面（+X 方向）投影截面 100 × 80 mm > 灯盘 55 mm，仍能形成完整遮挡。
# ---- 调参：无人机与相机（默认单值）--------------------------------------------
# 机体三轴尺寸 [mm]（用户设定）。直接决定阴影大小与机身安全边界：
#   机身越大 → 阴影越大 → 越容易全遮灯（R_occ 越大、可用站位区间越宽）。
#   机身安全边界 = DEFAULT_UAV_BOUNDS ∓ 半尺寸，故改尺寸会同时改变可用站位范围。
UAV_SIZE_MM: Vector3 = (120.0, 120.0, 120.0)

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
SIM_DT_S: float = 0.02
# 仿真时长上限 [s]：主场景会被"弹道飞行时间（≈1.92 s）"与"接触前截断"覆盖，故这里只是上限。
SIM_T_END_S: float = 3.0

A_MAX_OPTION: float = 2.0

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

#: 飞镖相机轨迹模式。
DART_MODE_PARABOLIC: str = "parabolic"
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
SIMPLE_V0_MPS: float = 22.0
SIMPLE_THETA0_DEG: float = 26.0
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
#       >0.16 末端阴影中心偏移发散、灯盘被甩出阴影。
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
    """绿色引导灯模型：垂直圆盘薄片。

    灯盘位于 ``X = 0`` 平面、法线 ``(-1, 0, 0)``（朝向迎面而来的飞镖）；
    ``X = 0`` 只是局部坐标取值，与地面高度无关。
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

#: 默认悬停站位：到灯平面距离 0.15 m、灯平面内横向偏置 (u=0, v=0) m。
#: v 是灯平面内的竖直偏置，v = 0 表示机身中心与灯心等高；
#: 实测 v=0.06 时阴影中心抬高 6 cm，R_occ 从 0.970 掉到 0.453（失效），故取 0。
PARABOLIC_LAUNCH_M: Vector3 = LAUNCH_REL_LAMP_M
PARABOLIC_IMPACT_M: Vector3 = (0.0, 0.0, 0.0)
PARABOLIC_FLIGHT_TIME_S: float = 3.0
PARABOLIC_APEX_RISE_M: float = 1.0
ARC_LENGTH_SAMPLES: int = 2001

def _arc_length_table(
    dart: "DartConfig",
    n_samples: int = ARC_LENGTH_SAMPLES,
) -> Tuple[np.ndarray, np.ndarray]:
    """抛物线弧长表 ``(u_grid, s_grid)``（按 ``DartConfig`` 缓存）。

    ``u`` 为路径参数（0 = 发射点、1 = 落点），``s`` 为从发射点沿曲线累计的弧长 [m]。
    用于“沿弹道恒定速率飞行”的时域映射 ``s(u) = v * t``，其中
    ``v = 总弧长 / t_flight``。
    """
    return _arc_length_table_cached(dart, int(n_samples))


@lru_cache(maxsize=64)
def _arc_length_table_cached(
    dart: "DartConfig",
    n_samples: int = ARC_LENGTH_SAMPLES,
) -> Tuple[np.ndarray, np.ndarray]:
    u_grid = np.linspace(0.0, 1.0, max(2, int(n_samples)))
    pts = np.asarray([dart.parabolic_path(float(u)) for u in u_grid], dtype=float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s_grid = np.concatenate([[0.0], np.cumsum(seg)])
    return u_grid, s_grid


@lru_cache(maxsize=64)
def _simple_table_cached(dart: "DartConfig", n_samples: int = 601):
    """最简抛物线模型预计算表。

    返回 ``(xi_grid, s_grid, s_total, g_eff, t_end)``：

    * ``xi``：沿水平方向的推进距离 [m]（0 -> 落点水平距离 R）
    * ``s``：对应弧长 [m]
    * ``g_eff``：由“射线穿过落点”反解的等效重力 [m/s^2]
    * ``t_end``：按 ``v(t) = v0 exp(-k t)`` 走完全程的时间 [s]
    """
    p0 = np.asarray(dart.launch, dtype=float)
    p1 = np.asarray(dart.impact, dtype=float)
    v0 = float(dart.v0_mps)
    theta = math.radians(float(dart.theta0_deg))
    range_m = float(math.hypot(p1[0] - p0[0], p1[1] - p0[1]))
    dz = float(p1[2] - p0[2])
    denom = max(range_m, 1e-9)
    g_eff = 2.0 * v0 * v0 * math.cos(theta) ** 2 * (range_m * math.tan(theta) - dz) / denom**2

    xi_grid = np.linspace(0.0, range_m, max(2, int(n_samples)))
    pts = np.asarray([dart._simple_point_at(float(xi), g_eff) for xi in xi_grid], dtype=float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s_grid = np.concatenate([[0.0], np.cumsum(seg)])
    s_total = float(s_grid[-1])

    k = max(0.0, float(dart.speed_decay_per_s))
    if k <= 1e-9:
        t_end = s_total / max(v0, 1e-9)
    else:
        ratio = 1.0 - k * s_total / max(v0, 1e-9)
        t_end = -math.log(max(ratio, 1e-6)) / k if ratio > 1e-6 else float(dart.t_flight)
    return xi_grid, s_grid, s_total, g_eff, t_end


@dataclass(frozen=True)
class DartConfig:
    """飞镖 / 相机位置轨迹 ``C(t)``，只建模板位置，不建模空气动力学。

    三种模式
    --------
    ``parabolic``        ：从 ``launch`` 抛出，先相对发射点上升 ``apex_rise_m``，
                           再于下降段落到 ``impact``（默认即绿灯中心 ``(0,0,0)``）。

    抛物线位于 y-z 平面：水平方向匀速，竖直方向为抛体（等效重力可由顶点高度反解，
    也可直接用 ``g_effective_mps2`` 指定），即 ``z(t) = z0 + a t - b t^2``。

    （注：本类内部仍用 ``z`` 表示竖直方向，即仿真坐标系的 ``Z`` 轴。）
    """

    mode: str = DART_MODE_SIMPLE
    source: str = ENGINEERING_ASSUMPTION
    launch: Vector3 = PARABOLIC_LAUNCH_M
    impact: Vector3 = PARABOLIC_IMPACT_M
    t_flight: float = PARABOLIC_FLIGHT_TIME_S
    apex_rise_m: float = PARABOLIC_APEX_RISE_M
    g_effective_mps2: float = 0.0
    constant_speed: bool = True
    # -- 2026 有控飞镖质点模型参数（报告数据） --------------------------
    v0_mps: float = 22.0
    # -- 最简抛物线模型（当前主场景） ------------------------------------
    theta0_deg: float = SIMPLE_THETA0_DEG
    speed_decay_per_s: float = SIMPLE_SPEED_DECAY_PER_S
    source_parabola: str = ENGINEERING_ASSUMPTION

    def __post_init__(self) -> None:
        if self.mode not in DART_MODES:
            raise ValueError(f"未知飞镖模式 {self.mode!r}，可选 {DART_MODES}")
        if self.mode == DART_MODE_PARABOLIC:
            if self.t_flight <= 0.0:
                raise ValueError("t_flight 必须为正")
            z0, z1 = float(self.launch[2]), float(self.impact[2])
            if self.g_effective_mps2 > 0.0:
                if self.g_effective_mps2 <= 0.0:
                    raise ValueError("g_effective_mps2 必须为正")
            else:
                if self.apex_rise_m <= 0.0:
                    raise ValueError(f"顶点相对发射点的抬升 apex_rise_m 必须为正，当前 {self.apex_rise_m}")
                if z1 - z0 > self.apex_rise_m + 1e-12:
                    raise ValueError("落点高度不能高于抛物线顶点")
            if not (0.0 < self.t_apex < self.t_flight):
                raise ValueError(
                    f"顶点时刻 {self.t_apex:.4f} s 必须位于 (0, {self.t_flight:g}) s 内，否则没有下降段"
                )

    # -- 抛物线解析式 -----------------------------------------------------

    @property
    def is_parabolic(self) -> bool:
        return self.mode == DART_MODE_PARABOLIC

    def parabolic_coeffs(self) -> Tuple[float, float]:
        """返回竖直方向系数 ``(a, b)``，满足 ``z(t) = z0 + a t - b t^2``。"""
        if not self.is_parabolic:
            raise ValueError("仅 parabolic 模式有抛物线解析式")
        z0 = float(self.launch[2])
        z1 = float(self.impact[2])
        T = float(self.t_flight)
        g = float(self.g_effective_mps2)
        if g > 0.0:
            # z(t) = z0 + v0 t - g t^2 / 2 且 z(T) = z1  =>  v0 = (z1 - z0 + g T^2 / 2) / T
            return (z1 - z0 + 0.5 * g * T * T) / T, 0.5 * g
        dz = self.apex_z_m - z0
        # 顶点条件 a^2 = 4 b dz；落点条件 z0 + aT - bT^2 = z1
        a = (2.0 * dz / T) * (1.0 + math.sqrt(max(0.0, 1.0 - (z1 - z0) / dz)))
        b = a * a / (4.0 * dz)
        return a, b

    @property
    def t_apex(self) -> float:
        """到达抛物线顶点（最高点）的时刻 [s]。

        ``constant_speed=True`` 时顶点位于路径参数 ``u_apex = a / (2 b T)`` 处，
        到达时刻 = ``L(u_apex) / v``（``L`` 为弧长、``v`` 为恒定速率）。
        """
        u_apex = self.path_param_apex
        if self.constant_speed:
            speed = self.speed_mps
            return self.arc_length_to(u_apex) / speed if speed > 0.0 else 0.0
        a, b = self.parabolic_coeffs()
        return a / (2.0 * b) if b > 0.0 else 0.0

    @property
    def path_param_apex(self) -> float:
        """顶点对应的**路径参数** ``u_apex ∈ [0, 1]``（与时间无关）。"""
        a, b = self.parabolic_coeffs()
        T = float(self.t_flight)
        return float(np.clip(a / (2.0 * b * T), 0.0, 1.0)) if b > 0.0 else 0.0

    @property
    def apex_z_m(self) -> float:
        """顶点绝对高度（相对灯平面）[m] = 发射点高度 + ``apex_rise_m``。"""
        return float(self.launch[2]) + float(self.apex_rise_m)

    @property
    def apex_height_m(self) -> float:
        """顶点的实际 z 值 [m]。"""
        return float(self.parabolic_path(self.path_param_apex)[2])

    # -- 路径 / 弧长参数化 -------------------------------------------------

    def parabolic_path(self, u: float) -> np.ndarray:
        """抛物线**路径**：``u ∈ [0, 1]`` 为路径参数（``u=0`` 发射、``u=1`` 落点）。

        形状与时间参数化完全一致（同一条抛物线），供弧长/恒定速率参数化使用。
        """
        a, b = self.parabolic_coeffs()
        T = float(self.t_flight)
        p0 = np.asarray(self.launch, dtype=float)
        p1 = np.asarray(self.impact, dtype=float)
        return np.array(
            [
                p0[0] + (p1[0] - p0[0]) * u,
                p0[1] + (p1[1] - p0[1]) * u,
                p0[2] + (a * T) * u - (b * T * T) * u * u,
            ]
        )

    def arc_length_to(self, u: float) -> float:
        """从发射点沿抛物线到路径参数 ``u`` 的弧长 [m]。"""
        u_grid, s_grid = _arc_length_table(self)
        return float(np.interp(float(np.clip(u, 0.0, 1.0)), u_grid, s_grid))

    def path_param_of_time(self, t: float) -> float:
        """时刻 ``t`` 对应的路径参数 ``u``（恒定速率时按弧长反解）。"""
        if not self.constant_speed:
            return float(np.clip(float(t) / float(self.t_flight), 0.0, 1.0))
        u_grid, s_grid = _arc_length_table(self)
        return float(np.interp(float(np.clip(self.speed_mps * float(t), 0.0, s_grid[-1])), s_grid, u_grid))

    @property
    def path_length_m(self) -> float:
        """抛物线全程弧长 [m]。"""
        _, s_grid = _arc_length_table(self)
        return float(s_grid[-1])

    @property
    def speed_mps(self) -> float:
        """``constant_speed=True`` 时的恒定速率 [m/s] = 弧长 / 飞行时间。"""
        return self.path_length_m / float(self.t_flight)

    def parabolic_position(self, t: float) -> np.ndarray:
        """抛物线位置 ``C(t)``。

        ``constant_speed=True``（默认）：沿抛物线以**恒定速率**飞行，按弧长反解位置；
        ``False``：退化为“水平匀速 + 竖直抛体”的弹道式时间参数化。
        """
        if self.constant_speed:
            return self.parabolic_path(self.path_param_of_time(t))
        a, b = self.parabolic_coeffs()
        p0 = np.asarray(self.launch, dtype=float)
        p1 = np.asarray(self.impact, dtype=float)
        T = float(self.t_flight)
        return np.array(
            [
                p0[0] + (p1[0] - p0[0]) * t / T,
                p0[1] + (p1[1] - p0[1]) * t / T,
                p0[2] + a * t - b * t * t,
            ]
        )

    def parabolic_velocity(self, t: float) -> np.ndarray:
        """抛物线速度 ``dC/dt``。

        ``constant_speed=True``：模长恒为 :attr:`speed_mps`，方向为路径切线
        （相机光轴因此等于弹道切线）；``False``：弹道式解析速度。
        """
        if self.constant_speed:
            u = self.path_param_of_time(t)
            du = 1e-4
            lo, hi = max(0.0, u - du), min(1.0, u + du)
            tangent = self.parabolic_path(hi) - self.parabolic_path(lo)
            norm = float(np.linalg.norm(tangent))
            if norm <= 1e-12:
                return np.zeros(3)
            return self.speed_mps * (tangent / norm)
        a, b = self.parabolic_coeffs()
        p0 = np.asarray(self.launch, dtype=float)
        p1 = np.asarray(self.impact, dtype=float)
        T = float(self.t_flight)
        return np.array(
            [
                (p1[0] - p0[0]) / T,
                (p1[1] - p0[1]) / T,
                a - 2.0 * b * t,
            ]
        )

    @property
    def impact_speed_mps(self) -> float:
        """落点速度大小 [m/s]。"""
        return float(np.linalg.norm(self.parabolic_velocity(self.t_flight - 1e-9)))

    # -- 2026 有控飞镖：六状态质点模型 ------------------------------------

    @property
    def is_simple(self) -> bool:
        return self.mode == DART_MODE_SIMPLE

    def _simple_table(self, n: int = 601):
        """最简抛物线：由 ``v0 / theta0`` 与两端点反解等效重力，再建弧长表。"""
        return _simple_table_cached(self, int(n))

    @property
    def effective_g_mps2(self) -> float:
        """由“穿过落点”反解出的等效重力 [m/s^2]。"""
        return float(self._simple_table()[3])

    @property
    def simple_flight_time(self) -> float:
        """按 ``v(t) = v0 exp(-k t)`` 的减速律走完全程的时间 [s]。"""
        return float(self._simple_table()[4])

    def simple_path_len_m(self) -> float:
        return float(self._simple_table()[2])

    def simple_position(self, t: float) -> np.ndarray:
        xi_grid, s_grid, s_total, _g_eff, t_end = self._simple_table()
        k = max(0.0, float(self.speed_decay_per_s))
        v0 = float(self.v0_mps)
        tt = float(np.clip(float(t), 0.0, t_end))
        s = (v0 / k) * (1.0 - math.exp(-k * tt)) if k > 1e-9 else v0 * tt
        xi = float(np.interp(min(s, s_total), s_grid, xi_grid))
        return self._simple_point_at(xi)

    def simple_velocity(self, t: float) -> np.ndarray:
        xi_grid, s_grid, s_total, _g_eff, t_end = self._simple_table()
        k = max(0.0, float(self.speed_decay_per_s))
        tt = float(np.clip(float(t), 0.0, t_end))
        speed = float(self.v0_mps) * math.exp(-k * tt)
        dt = 1e-4
        p0_ = self.simple_position(max(0.0, tt - dt))
        p1_ = self.simple_position(min(t_end, tt + dt))
        direction = p1_ - p0_
        nrm = float(np.linalg.norm(direction))
        return direction / nrm * speed if nrm > 1e-12 else np.zeros(3)

    def _simple_g_eff(self) -> float:
        """由“抛物线穿过落点”反解的等效重力（不依赖预计算表，避免递归）。"""
        p0 = np.asarray(self.launch, dtype=float)
        p1 = np.asarray(self.impact, dtype=float)
        v0 = float(self.v0_mps)
        theta = math.radians(float(self.theta0_deg))
        range_m = float(math.hypot(p1[0] - p0[0], p1[1] - p0[1]))
        dz = float(p1[2] - p0[2])
        denom = max(range_m, 1e-9)
        return 2.0 * v0 * v0 * math.cos(theta) ** 2 * (range_m * math.tan(theta) - dz) / denom**2

    def _simple_point_at(self, xi: float, g_eff: float | None = None) -> np.ndarray:
        """给定水平前进距离 ``xi`` 返回轨迹点（抛物线解析式）。"""
        p0 = np.asarray(self.launch, dtype=float)
        p1 = np.asarray(self.impact, dtype=float)
        horiz = np.array([p1[0] - p0[0], p1[1] - p0[1], 0.0])
        range_m = float(np.linalg.norm(horiz))
        if range_m <= 1e-9:
            return p0 + (p1 - p0) * (xi / max(float(np.linalg.norm(p1 - p0)), 1e-9))
        h_hat = horiz / range_m
        g_use = self._simple_g_eff() if g_eff is None else float(g_eff)
        theta = math.radians(float(self.theta0_deg))
        v0 = float(self.v0_mps)
        drop = g_use * xi * xi / (2.0 * v0 * v0 * math.cos(theta) ** 2)
        return p0 + h_hat * xi + np.array([0.0, 0.0, xi * math.tan(theta) - drop])

    # -- 关键点（统一接口） ------------------------------------------------

    @property
    def keypoints(self) -> List[Tuple[float, Vector3]]:
        """返回按时序排列的 ``(t [s], (x, y, z) [m])`` 关键点。"""
        if self.mode == DART_MODE_SIMPLE:
            times = np.linspace(0.0, float(self.t_end), 61)
            return [(float(ti), tuple(self.simple_position(float(ti)))) for ti in times]
        # parabolic：给出解析曲线的采样点，便于校验、绘图与离散查看
        times = np.linspace(0.0, float(self.t_flight), 61)
        return [(float(ti), tuple(self.parabolic_position(float(ti)))) for ti in times]

    @property
    def t_start(self) -> float:
        return self.keypoints[0][0]

    @property
    def t_end(self) -> float:
        if self.is_simple:
            return float(self.simple_flight_time)
        return float(self.t_flight) if self.is_parabolic else self.keypoints[-1][0]

    def source_rows(self) -> List[Tuple[str, str, str]]:
        if self.is_parabolic:
            a, b = self.parabolic_coeffs()
            return [
                (
                    "dart.mode/parabola",
                    f"{self.mode}: {self.launch} -> apex z={self.apex_height_m:.3f} m @ "
                    f"{self.t_apex:.3f} s -> impact {self.impact}; T={self.t_flight:g} s; "
                    f"z(t) = z0 + {a:.4f} t - {b:.4f} t^2",
                    self.source_parabola,
                ),
                (
                    "dart.impact",
                    f"落点速度 {self.impact_speed_mps:.3f} m/s（下降段，落点为绿灯中心）",
                    self.source_parabola,
                ),
            ]
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
