"""由 vendored Python 参考实现（``tools/python_ref/``）生成 M2 黄金基准。

用法::

    python tools/gen_golden.py [--python-dir tools/python_ref] [--out-dir tests/golden]

输出：

* ``tests/golden/<case>.json``  —— 7 组 M2 场景（逐帧位置/速度/遮挡率/锁定/有效帧）
* ``tests/golden/tools_parity.json`` —— 新函数的跨语言输入→输出表（≤1e-12 断言）
* ``tests/golden/margin.json`` —— 各 case 的遮挡余量（灯心到阴影最近边 − 灯球半径）
* ``tests/golden/geometry.json`` —— 纯几何采样（顶点/平面基/视线基/J2 判据）

⚠️ 无效帧（``valid = 0``）的 ``R`` 写成 ``null``：Python ``json.dumps`` 会输出非法的
``NaN`` 字面量，而前端 ``JSON.parse`` 会直接抛错。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

HERE = Path(__file__).resolve().parent
WEB_ROOT = HERE.parent
DEFAULT_PYTHON_DIR = HERE / "python_ref"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="生成 M2 黄金基准")
    parser.add_argument("--python-dir", default=str(DEFAULT_PYTHON_DIR),
                        help="vendored Python 参考实现目录")
    parser.add_argument("--out-dir", default=str(WEB_ROOT / "tests" / "golden"),
                        help="输出目录")
    return parser.parse_args()


def load_reference(python_dir: Path) -> Dict[str, Any]:
    if not (python_dir / "config.py").exists():
        raise SystemExit(f"找不到参考实现：{python_dir}")
    sys.path.insert(0, str(python_dir))
    import camera as camera_mod  # noqa: E402
    import config as config_mod  # noqa: E402
    import experiment as experiment_mod  # noqa: E402
    import geometry as geometry_mod  # noqa: E402
    import motion as motion_mod  # noqa: E402
    import occlusion as occlusion_mod  # noqa: E402

    return {
        "camera": camera_mod,
        "config": config_mod,
        "experiment": experiment_mod,
        "geometry": geometry_mod,
        "motion": motion_mod,
        "occlusion": occlusion_mod,
    }


def _nan_to_none(values) -> List[Any]:
    return [None if not np.isfinite(v) else float(v) for v in np.asarray(values, dtype=float)]


def build_case(
    ref: Dict[str, Any],
    *,
    standoff: float = 0.15,
    offset_u: float = 0.0,
    offset_v: float = 0.0,
    uav_size_mm: Optional[Sequence[float]] = None,
    lamp_diameter_mm: float = 55.0,
    lamp_quad_segs: int = 256,
    hfov_deg: float = 70.0,
    v0: float = 20.0,
    theta0_deg: float = 30.0,
    speed_decay: float = 0.55,
    dt: float = 0.01,
    r_th: float = 0.7,
    lamp_target_y_m: float = 0.0,
) -> Dict[str, Any]:
    cfg = ref["config"]
    lamp = cfg.LampConfig(diameter_mm=float(lamp_diameter_mm))
    camera_cfg = cfg.CameraConfig(fov_h_deg=float(hfov_deg))
    dart = cfg.DartConfig(mode="simple", v0_mps=float(v0), theta0_deg=float(theta0_deg),
                          speed_decay_per_s=float(speed_decay))
    # 三轴独立尺寸（长=X / 宽=Y / 高=Z）；默认取 vendored config 的唯一真值
    size_mm: Tuple[float, float, float] = tuple(  # type: ignore[assignment]
        float(v) for v in (uav_size_mm if uav_size_mm is not None else cfg.UAV_SIZE_MM)
    )
    size_m = (size_mm[0] / 1000.0, size_mm[1] / 1000.0, size_mm[2] / 1000.0)
    uav = cfg.UAVConfig(size_m=size_m)
    sim = cfg.SimConfig(R_th=float(r_th), dt=float(dt), t_end=float(cfg.SIM_T_END_S),
                        uav_bounds=dict(cfg.DEFAULT_UAV_BOUNDS),
                        lamp_target_y_m=float(lamp_target_y_m))
    station = cfg.HoverStationConfig(standoff_m=float(standoff), offset_u_m=float(offset_u),
                                     offset_v_m=float(offset_v))

    run = ref["experiment"].run_m2(
        dart, lamp, camera_cfg, sim, station, size_m,
        uav_max_speed=float(cfg.UAV_MAX_SPEED_MPS),
        uav_max_accel=float(cfg.UAV_MAX_ACCEL_MPS2),
        omega_max_dps=float(cfg.MAX_TURN_RATE_DPS),
        g=float(cfg.GRAVITY_MPS2),
        dt=float(dt),
        lamp_target_y_m=float(lamp_target_y_m),
        lamp_quad_segs=int(lamp_quad_segs),
    )

    poly = ref["occlusion"].lamp_disc_polygon(lamp, quad_segs=int(lamp_quad_segs))
    return {
        "params": {
            "standoff_m": float(standoff),
            "offset_u_m": float(offset_u),
            "offset_v_m": float(offset_v),
            "uav_size_mm": [float(size_mm[0]), float(size_mm[1]), float(size_mm[2])],
            "lamp_diameter_mm": float(lamp_diameter_mm),
            "lamp_quad_segs": int(lamp_quad_segs),
            "lamp_segments": int(lamp_quad_segs) * 4,
            "hfov_deg": float(hfov_deg),
            "v0_mps": float(v0),
            "theta0_deg": float(theta0_deg),
            "speed_decay_per_s": float(speed_decay),
            "dt": float(dt),
            "R_th": float(r_th),
            "lamp_target_y_m": float(lamp_target_y_m),
        },
        "n": int(run["n"]),
        "tLast": float(run["t"][-1]),
        "lampYTargetM": float(lamp_target_y_m),
        "tLock": float(run["tLock"]),
        "tHit": (None if run["tHit"] is None else float(run["tHit"])),
        "hitLamp": bool(run["hitLamp"]),
        "lampY": _nan_to_none(run["lampY"]),
        "lampPos": _nan_to_none(run["lampPos"]),
        "uavY": _nan_to_none(run["uavY"]),
        "uavVy": _nan_to_none(run["uavVy"]),
        "uavPos": _nan_to_none(run["uavPos"]),
        "Pd": _nan_to_none(run["Pd"]),
        "Vd": _nan_to_none(run["Vd"]),
        "R": _nan_to_none(run["R"]),
        "imageR": _nan_to_none(run["imageR"]),
        "fovMarginDeg": _nan_to_none(run["fovMarginDeg"]),
        "locked": [int(x) for x in run["locked"]],
        "valid": [int(x) for x in run["valid"]],
        "lamp": {
            "radiusM": float(lamp.radius_m),
            "areaM2": float(lamp.area_m2),
            "polygonAreaM2": float(poly.area),
        },
        "camera": {
            "width": int(camera_cfg.resolution[0]),
            "height": int(camera_cfg.resolution[1]),
            "fovHDeg": float(camera_cfg.fov_h_deg),
            "focalPx": float(camera_cfg.focal_px),
        },
        "marginMinM": float(run["marginMin"]),
        "marginMinT": float(run["marginMinT"]),
    }


def tools_parity_case(ref: Dict[str, Any]) -> Dict[str, Any]:
    """跨语言函数表：Python 现算，TS 端逐条断言（≤1e-12）。"""
    motion = ref["motion"]
    config_mod = ref["config"]
    camera_cfg = config_mod.CameraConfig()
    dt, k, g, omega = 0.01, 0.55, 9.8, math.radians(60.0)
    camera_json = {
        "width": int(camera_cfg.resolution[0]),
        "height": int(camera_cfg.resolution[1]),
        "fovHDeg": float(camera_cfg.fov_h_deg),
    }

    lamp_y_rows = []
    lamp_vy_rows = []
    for yt in (0.24, -0.24):
        for t in (0.0, 1.19, 1.2, 1.5, 1.79, 1.8, 2.0):
            lamp_y_rows.append({"t": t, "yTarget": yt,
                                "expected": float(motion.lamp_y_at(t, yt))})
            lamp_vy_rows.append({"t": t, "yTarget": yt,
                                 "expected": float(motion.lamp_vy_at(t, yt))})

    # stepDartM2：覆盖"弹道段（灯不在视场内）/追踪段（灯在视场内）/速度退化"三类状态
    launch = np.asarray(config_mod.LAUNCH_REL_LAMP_M, dtype=float)
    h0 = np.array([0.0 - launch[0], 0.0 - launch[1], 0.0])
    h0 = h0 / float(np.linalg.norm(h0))
    th = math.radians(30.0)
    v0_vec = 20.0 * (math.cos(th) * h0 + math.sin(th) * np.array([0.0, 0.0, 1.0]))
    step_rows = []
    for P, V, L in (
        (launch, v0_vec, np.array([0.0, 0.0, 0.0])),
        (np.array([-5.0, -0.5, 2.0]), np.array([8.0, 0.2, 0.5]), np.array([0.0, 0.0, 0.0])),
        (np.array([-1.0, -0.1, 1.5]), np.array([6.0, 0.1, -0.2]), np.array([0.0, 0.24, 0.0])),
        (np.array([-0.5, 0.0, 1.0]), np.array([1e-13, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])),
    ):
        p_next, v_next, locked = motion.step_dart_m2(
            P, V, L, dt, k, g, omega, camera_cfg, config_mod.CAMERA_UP_REF
        )
        step_rows.append({
            "P": [float(x) for x in P], "V": [float(x) for x in V],
            "L": [float(x) for x in L], "dt": dt, "k": k, "g": g,
            "omegaMaxRad": omega,
            "expected": {"P_next": [float(x) for x in p_next],
                         "V_next": [float(x) for x in v_next],
                         "locked": bool(locked)},
        })

    seg_rows = []
    for A, B, Q in (
        ([0, 0, 0], [1, 0, 0], [0.5, 0.1, 0]),
        ([0, 0, 0], [1, 0, 0], [2, 0, 0]),
        ([0, 0, 0], [0, 0, 0], [1, 0, 0]),
        ([-0.5, 0.2, 1.0], [0.3, -0.4, 2.0], [0.1, 0.0, 1.5]),
    ):
        seg_rows.append({"A": A, "B": B, "Q": Q,
                         "expected": float(motion.segment_to_point_distance(A, B, Q))})

    uav_rows = []
    for y0, v0, y1, v_max, a_max, ddt in (
        (0.0, 0.0, 0.24, 5.0, 2.0, 0.01),
        (0.12, 0.30, 0.24, 5.0, 2.0, 0.01),
        (0.0, 0.0, 100.0, 5.0, 2.0, 0.01),
        (0.0, -1.0, 0.24, 5.0, 2.0, 0.01),
        (0.24, -0.6, 0.24, 5.0, 2.0, 0.01),
        (0.0, 0.0, -0.24, 5.0, 2.0, 0.01),
    ):
        y_next, v_next = motion.uav_step(y0, v0, y1, v_max, a_max, ddt)
        uav_rows.append({"y0": y0, "v0": v0, "y1": y1, "vMax": v_max, "aMax": a_max,
                         "dt": ddt, "expected": {"y": float(y_next), "v": float(v_next)}})

    return {
        "camera": camera_json,
        "lampYAt": lamp_y_rows,
        "lampVyAt": lamp_vy_rows,
        "stepDartM2": step_rows,
        "segmentToPointDistance": seg_rows,
        "uavStep": uav_rows,
    }


def geometry_case(ref: Dict[str, Any]) -> Dict[str, Any]:
    geometry_mod = ref["geometry"]
    motion = ref["motion"]
    config_mod = ref["config"]
    center = [0.1, -0.2, 0.3]
    # v3：采样用三轴互不相同的机体，让顶点顺序/坐标在跨语言 1e-15 断言下也被锁定
    size = [0.15, 0.07, 0.09]
    corners = geometry_mod.uav_corners(center, size)
    u_hat, v_hat = geometry_mod.plane_basis((-1.0, 0.0, 0.0))
    los_rows = []
    for C, L in (
        ([-1.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        ([-5.0, -1.0, 2.0], [0.0, 0.24, 0.0]),
        ([-0.3, 0.1, 0.05], [0.0, -0.24, 0.0]),
    ):
        lu, lv = geometry_mod.los_basis(np.asarray(C, dtype=float), np.asarray(L, dtype=float))
        los_rows.append({"C": C, "L": L,
                         "u": [float(x) for x in lu], "v": [float(x) for x in lv]})
    camera_cfg = config_mod.CameraConfig()
    vis_rows = []
    for P, d, L in (
        ([-25.03705, -2.86818, -0.56904], [0.866, 0.0, 0.5], [0.0, 0.0, 0.0]),
        ([-5.0, -0.6, 1.2], [0.95, 0.1, -0.2], [0.0, 0.0, 0.0]),
        ([-0.5, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
    ):
        dd = np.asarray(d, dtype=float)
        dd = dd / float(np.linalg.norm(dd))
        vis_rows.append({"P": P, "d": [float(x) for x in dd], "L": L,
                         "expected": bool(motion.is_fully_visible(
                             np.asarray(P, dtype=float), dd, np.asarray(L, dtype=float), camera_cfg))})
    return {
        "center": center,
        "size": size,
        "corners": [[float(v) for v in row] for row in corners],
        "normal": [-1.0, 0.0, 0.0],
        "basisU": [float(v) for v in u_hat],
        "basisV": [float(v) for v in v_hat],
        "losBasis": los_rows,
        "isFullyVisible": vis_rows,
    }


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    ref = load_reference(Path(args.python_dir).resolve())

    cases = {
        "default": build_case(ref, lamp_target_y_m=0.0),
        "lamp_y_pos": build_case(ref, lamp_target_y_m=0.24),
        "lamp_y_neg": build_case(ref, lamp_target_y_m=-0.24),
        "lamp_y_pos_v0_22": build_case(ref, lamp_target_y_m=0.24, v0=22.0),
        "standoff_012": build_case(ref, standoff=0.12),
        "standoff_025": build_case(ref, standoff=0.25),
        "lamp_segments_64": build_case(ref, lamp_quad_segs=16),
        # v3：三轴显著不同（长 150 / 窄 70 / 中高 90），覆盖 uav_corners / 机身安全边界 /
        # 站位裁剪里所有"轴混用"路径
        "uav_rectangular": build_case(ref, uav_size_mm=(150.0, 70.0, 90.0)),
    }

    margin: Dict[str, Any] = {}
    for name, payload in cases.items():
        path = out_dir / f"{name}.json"
        path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
        finite = [r for r in payload["R"] if r is not None]
        margin[name] = {
            "n": payload["n"],
            "tHit": payload["tHit"],
            "hitLamp": payload["hitLamp"],
            "tLock": payload["tLock"],
            "R_min": min(finite) if finite else None,
            "R_mean": (sum(finite) / len(finite)) if finite else None,
            "invalidFrames": sum(1 for r in payload["R"] if r is None),
            "marginMinM": payload["marginMinM"],
            "marginMinT": payload["marginMinT"],
        }
        print(f"  -> {name}.json: n={payload['n']:4d} tHit={payload['tHit']} "
              f"hitLamp={payload['hitLamp']} R_min={margin[name]['R_min']} "
              f"margin={payload['marginMinM']*1000:+.2f}mm")

    (out_dir / "tools_parity.json").write_text(
        json.dumps(tools_parity_case(ref), ensure_ascii=False), encoding="utf-8"
    )
    (out_dir / "margin.json").write_text(
        json.dumps(margin, ensure_ascii=False, indent=1), encoding="utf-8"
    )
    (out_dir / "geometry.json").write_text(
        json.dumps(geometry_case(ref), ensure_ascii=False, indent=1), encoding="utf-8"
    )
    print("  -> tools_parity.json / margin.json / geometry.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
