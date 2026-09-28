"""策略层：只生成“下一步目标点”，从不直接移动无人机。

当前阶段（唯一启用）：无人机**不做机动**，只在距灯特定位置的站位上悬停。
``TargetPolicy`` 接口保留，后续要引入机动策略时新增一个实现即可。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol, Sequence, runtime_checkable

import numpy as np

from config import DEFAULT_HOVER_STATION, HoverStationConfig, LampConfig
from geometry import as_vector


@runtime_checkable
class TargetPolicy(Protocol):
    """目标点生成接口（机动能力的扩展点）。"""

    name: str

    def target(
        self,
        t: float,
        dart_cfg,
        lamp: LampConfig,
        P_current: Optional[Sequence[float]] = None,
        P_star: Optional[Sequence[float]] = None,
    ) -> np.ndarray:
        """返回 ``t`` 时刻的期望目标点（世界坐标，米）。"""

    def target_velocity(self, t: float, dart_cfg) -> np.ndarray:
        """返回目标点自身速度（前馈项）；静止目标返回零向量。"""

    def describe(self) -> str:
        """返回该策略的可读描述（用于日志）。"""


@dataclass(frozen=True)
class HoverStationPolicy:
    """**悬停在距灯特定位置的站位上**（当前唯一启用的方式）。

    目标是常数点，与时间、相机位置无关；``target_velocity`` 恒为零向量。
    """

    station: HoverStationConfig = DEFAULT_HOVER_STATION
    name: str = "hover"

    def target(self, t, dart_cfg, lamp, P_current=None, P_star=None) -> np.ndarray:
        del t, dart_cfg, P_current, P_star
        return self.station.position(lamp)

    def target_velocity(self, t, dart_cfg) -> np.ndarray:
        del t, dart_cfg
        return np.zeros(3)

    def describe(self) -> str:
        return (
            f"hover @ 距灯心 {self.station.distance_to_lamp_m:.3f} m"
            f"（法向 {self.station.standoff_m:.3f} m）"
        )


@dataclass(frozen=True)
class FixedPointPolicy:
    """悬停在给定点 ``P_star``（不依赖灯位姿）。"""

    name: str = "fixed"

    def target(self, t, dart_cfg, lamp, P_current=None, P_star=None) -> np.ndarray:
        del t, dart_cfg, lamp, P_current
        if P_star is None:
            raise ValueError("FixedPointPolicy 需要提供 P_star")
        return as_vector(P_star).copy()

    def target_velocity(self, t, dart_cfg) -> np.ndarray:
        del t, dart_cfg
        return np.zeros(3)

    def describe(self) -> str:
        return "fixed @ 预定定义点"


#: CLI / 场景层可见的全部策略（当前只有“站住不动”两种写法）。
POLICY_NAMES: tuple = ("hover", "fixed")


def make_policy(
    name: str,
    station: HoverStationConfig = DEFAULT_HOVER_STATION,
) -> TargetPolicy:
    """按名字构造策略对象；``hover`` 为当前主场景。"""
    if name == "hover":
        return HoverStationPolicy(station=station)
    if name == "fixed":
        return FixedPointPolicy()
    raise ValueError(f"未知策略 {name!r}，可选 {POLICY_NAMES}")
