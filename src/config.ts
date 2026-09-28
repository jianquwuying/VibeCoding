/**
 * 全局配置与默认参数。
 *
 * 坐标系：以绿灯几何中心为原点，X 沿飞行方向（弹道沿 +X 前进）、Y 场地横向、Z 竖直向上。
 * 灯为**球体**（半径 27.5 mm），沿 Y 方向在 [-0.24, +0.24] m 内平移。
 *
 * 数值真值来源：`tools/python_ref/`（vendored Python 参考实现）。
 * 改动本文件任何数值后，必须重新生成黄金基准（`npm run golden`），否则 parity 测试会失败。
 */

// ============================================================================
// 可调参数区（所有默认值集中在此，改这里即可）
// ============================================================================
//
// 修改后需要重新生成黄金基准（npm run golden），否则 parity 测试会失败。
// 参数范围（PARAM_RANGES）与本区独立，用于 Tweakpane 的 UI 约束。

// ---- 弹道 / 制导 ---------------------------------------------------------

/** 出膛速度 [m/s]。UI 范围 [15, 30]。影响命中时间与轨迹长度。 */
export const SIMPLE_V0_MPS = 20.0;

/** 初始仰角 [°]。UI 范围 [15, 45]。影响弹道段轨迹与首次锁定时刻。 */
export const SIMPLE_THETA0_DEG = 30.0;

/** 阻力系数 [1/s]。UI 范围 [0, 1.5]。v(t) 衰减率。 */
export const SIMPLE_SPEED_DECAY_PER_S = 0.55;

/** 重力加速度 [m/s²]。 */
export const GRAVITY_MPS2 = 9.8;

/** 最大转弯率 [°/s]。追踪段尾翼转向上限；常量（无 UI 控件），改后必须重跑 golden。 */
export const MAX_TURN_RATE_DPS = 60.0;

// ---- 灯横向平移 ----------------------------------------------------------

/** 灯 Y 偏移的物理限位 [mm]。 */
export const LAMP_Y_RANGE_MM: readonly [number, number] = [-240, 240];
export const LAMP_Y_STEP_MM = 1;

/** 灯开始移动 / 到位的时刻 [s]。 */
export const LAMP_MOVE_START_S = 1.2;
export const LAMP_MOVE_END_S = 1.8;

// ---- 球体灯 --------------------------------------------------------------

/** 球直径 [mm]。 */
export const LAMP_DIAMETER_MM = 55.0;
/** 球半径 [m] = 55 / 2000 = 0.0275。 */
export const LAMP_SPHERE_RADIUS_M = LAMP_DIAMETER_MM / 2000;

// ---- 仿真 ----------------------------------------------------------------

/** 仿真时间上限 [s]。 */
export const SIM_T_END_S = 5.0;
/** 仿真步长 [s]。 */
export const SIM_DT_S = 0.01;
/** 命中判定距离 [m]（线段到灯心最近距离）。 */
export const HIT_DISTANCE_M = 0.05;
/** 最小帧数（保证 n ≥ 2，避免空结果）。 */
export const MIN_FRAMES = 2;

// ---- 无人机运动 ----------------------------------------------------------

/** 无人机最大速度 [m/s]。 */
export const UAV_MAX_SPEED_MPS = 5.0;
/** 无人机最大加速度 [m/s²]。 */
export const UAV_MAX_ACCEL_MPS2 = 2.0;

// ---- 无人机尺寸（三轴独立，长=X 宽=Y 高=Z） -------------------------------

/** 机体 X 轴（沿飞行方向）长度 [mm]。UI 范围 [50, 200]，默认 100。 */
export const UAV_SIZE_X_MM = 100.0;

/** 机体 Y 轴（场地横向）宽度 [mm]。UI 范围 [50, 200]，默认 100。 */
export const UAV_SIZE_Y_MM = 100.0;

/**
 * 机体 Z 轴（竖直）高度 [mm]。UI 范围 [50, 200]，默认 80。
 *
 * 提示：Z < 55 mm 时机身比灯球（直径 55 mm）还扁，遮挡策略可能失效（UI 红字警告）。
 * v3 默认高度 80 mm 下最紧时刻（t ≈ 0.75 s）灯球露出 ≈2.5% 面积：
 * `R_min ≈ 0.974843`、`minMargin ≈ −3.36 mm`，即**默认不再完全遮挡**。
 */
export const UAV_SIZE_Z_MM = 80.0;

// ---- 持久化 --------------------------------------------------------------

export const STORAGE_KEY = 'rm-uav-occlusion-params-v3';
export const STORAGE_SCHEMA_VERSION = 3;

// ============================================================================
// 以下为派生常量与类型定义（一般不需要改）
// ============================================================================

export type Vec3 = [number, number, number];

// ---------------------------------------------------------------------------
// 机械装配数据（唯一录入点，单位 mm）
// ---------------------------------------------------------------------------

export const LAMP_CENTER_MECH_MM: Vec3 = [25037.05, 2868.18, 1139.04];
export const LAUNCH_POINT_MECH_MM: Vec3 = [0.0, 0.0, 570.0];

/** 发射点在灯局部坐标系下的位置 [m] ≈ (-25.03705, -2.86818, -0.56904)。 */
export const LAUNCH_REL_LAMP_M: Vec3 = [
  (LAUNCH_POINT_MECH_MM[0] - LAMP_CENTER_MECH_MM[0]) / 1000,
  (LAUNCH_POINT_MECH_MM[1] - LAMP_CENTER_MECH_MM[1]) / 1000,
  (LAUNCH_POINT_MECH_MM[2] - LAMP_CENTER_MECH_MM[2]) / 1000,
];

// ---------------------------------------------------------------------------
// 灯球（平面占位 + 截面多边形离散精度）
// ---------------------------------------------------------------------------

export const LAMP_CENTER: Vec3 = [0.0, 0.0, 0.0];
/** 灯球所在的 X=0 平面法线；仅作平面/网格的几何占位，不参与遮挡计算（灯是球体，无朝向）。 */
export const LAMP_NORMAL: Vec3 = [-1.0, 0.0, 0.0];

/** 灯球离散段数：默认 1024（= Python `Point(...).buffer(r, quad_segs=256)`）。 */
export const LAMP_SEGMENTS_DEFAULT = 1024;
export const LAMP_SEGMENTS_MIN = 64;
export const LAMP_SEGMENTS_MAX = 1024;
export const LAMP_SEGMENTS_STEP = 64;

// ---------------------------------------------------------------------------
// 无人机
// ---------------------------------------------------------------------------

/**
 * 机身允许占据的轴对齐包围盒 [m]（机身中心安全边界 = 该盒 ∓ 半尺寸）。
 * v3 起机体尺寸三轴独立（`UAV_SIZE_X/Y/Z_MM`），边界按轴各自收缩。
 */
export const DEFAULT_UAV_BOUNDS = {
  x: [-26.0, 0.5] as [number, number],
  y: [-6.0, 6.0] as [number, number],
  z: [-2.0, 3.0] as [number, number],
};

// ---------------------------------------------------------------------------
// 相机
// ---------------------------------------------------------------------------

export const CAMERA_WIDTH = 640;
export const CAMERA_HEIGHT = 480;
export const CAMERA_HFOV_DEG = 70.0;
export const CAMERA_UP_REF: Vec3 = [0.0, 0.0, 1.0];
/** 第一人称成像用的灯球投影段数。 */
export const IMAGE_SPACE_SEGMENTS = 128;

// ---------------------------------------------------------------------------
// 悬停站位
// ---------------------------------------------------------------------------

export const HOVER_STATION_STANDOFF_M = 0.15;
export const HOVER_STATION_OFFSET_U_M = 0.0;
export const HOVER_STATION_OFFSET_V_M = 0.0;

// ---------------------------------------------------------------------------
// 阈值 / 播放 / 导出
// ---------------------------------------------------------------------------

export const R_TH_DEFAULT = 0.7;
export const R_TH_MIN = 0.3;
export const R_TH_MAX = 0.95;

/**
 * 播放速度滑块：0.5 = 慢放 2×（1.87 s 仿真播成 3.73 s）。
 * 滑块值本身即 playbackSpeed（不再有独立的慢放因子）。
 */
export const PLAYBACK_SPEED_DEFAULT = 0.5;
export const PLAYBACK_SPEED_MIN = 0.25;
export const PLAYBACK_SPEED_MAX = 4.0;
export const PLAYBACK_SPEED_SNAPS: readonly number[] = [0.25, 0.5, 1.0, 2.0, 4.0];
export const SNAP_EPS = 0.05;

export const EXPORT_FPS = 60;
export const EXPORT_WIDTH = 1920;
export const EXPORT_HEIGHT = 1080;
export const EXPORT_BITRATE_KBPS = 4000;

/**
 * EXPORT_SPEED 语义：仿真时间的播放速率
 *   0.5 → 慢放 2 倍（1.87 s 仿真 → 3.73 s 视频），当前默认
 *   1.0 → 正常速度（1.87 s → 1.87 s）
 * 导出速度与 UI 当前播放速度**解耦**：无论滑块拖到多少，导出固定用该值。
 */
export const EXPORT_SPEED = 0.5;
/** 预估内存超限时自动降级为 JPEG 的阈值（字节）。 */
export const EXPORT_MEMORY_BUDGET_BYTES = 120 * 1024 * 1024;

/**
 * FPV 小窗悬停放大：基准尺寸（由 hudLayout 推导）× 1.5，
 * 再按「不超过视口高度 60%」封顶（宽度口径，保持 4:3）。
 */
export const FPV_HOVER_SCALE = 1.5;
export const FPV_HOVER_MAX_HEIGHT_FRACTION = 0.6;

// ---------------------------------------------------------------------------
// 场景网格
// ---------------------------------------------------------------------------

/** 坐标轴三色——唯一色源：渲染与测试共用，任何一侧都不得再写死色值。 */
export const AXIS_COLORS = { x: 0xef4444, y: 0x22c55e, z: 0x3b82f6 } as const;
export const GRID_COARSE_OPACITY = 0.9;
export const GRID_FINE_OPACITY = 0.7;
export const GRID_WIDTH_RANGE: readonly [number, number] = [1, 4];
export const GRID_WIDTH_STEP = 0.5;

// ---------------------------------------------------------------------------
// 参数对象
// ---------------------------------------------------------------------------

/** 参与仿真重算的全部参数。 */
export interface SimParams {
  /** 出膛速度 [m/s] */
  v0: number;
  /** 初始仰角 [deg] */
  theta0Deg: number;
  /** 速度衰减率 [1/s]（沿线速度，追踪段保持速率、方向受限） */
  speedDecayPerS: number;
  /** 站位到灯平面的距离 [m] */
  standoffM: number;
  /** 灯平面内横向偏置 [m]（u 方向） */
  offsetUM: number;
  /** 灯平面内竖直偏置 [m]（v 方向，即高度） */
  offsetVM: number;
  /** 机体 X 轴（沿飞行方向）长度 [mm] */
  uavSizeXMm: number;
  /** 机体 Y 轴（场地横向）宽度 [mm] */
  uavSizeYMm: number;
  /** 机体 Z 轴（竖直）高度 [mm]：< 55 mm 时遮挡策略可能失效 */
  uavSizeZMm: number;
  /** 灯球直径 [mm] */
  lampDiameterMm: number;
  /** 灯球多边形逼近段数（64~1024） */
  lampSegments: number;
  /** 相机水平 FOV [deg] */
  hfovDeg: number;
  /** 仿真步长 [s] */
  dt: number;
  /** 有效遮挡分析阈值 */
  rTh: number;
  /** 灯的 Y 向目标偏移 [m]，范围 ±0.24 */
  lampTargetYM: number;
}

/** 网格所在平面：灯球平面 X=0 / 水平面 Z=0 / 两者都画。 */
export type GridPlane = 'lamp' | 'horizontal' | 'both';

/** 只影响显示、不影响仿真结果的开关。 */
export interface DisplayOptions {
  trail: boolean;
  shadow: boolean;
  frustum: boolean;
  comb: boolean;
  fpv: boolean;
  curve: boolean;
  /** 显示灯移动轨道（虚线 + 限位 + 当前灯位标记） */
  lampTrack: boolean;
  gridPlane: GridPlane;
  showCoarse: boolean;
  showFine: boolean;
  showAxes: boolean;
  /** 粗网格颜色（hex 字符串，绑定到 Tweakpane 颜色选择器） */
  gridCoarseColor: string;
  /** 细网格颜色（hex 字符串） */
  gridFineColor: string;
  /** 粗网格线宽 [px] */
  gridCoarseWidth: number;
  /** 细网格线宽 [px] */
  gridFineWidth: number;
}

export const DEFAULT_PARAMS: SimParams = {
  v0: SIMPLE_V0_MPS,
  theta0Deg: SIMPLE_THETA0_DEG,
  speedDecayPerS: SIMPLE_SPEED_DECAY_PER_S,
  standoffM: HOVER_STATION_STANDOFF_M,
  offsetUM: HOVER_STATION_OFFSET_U_M,
  offsetVM: HOVER_STATION_OFFSET_V_M,
  uavSizeXMm: UAV_SIZE_X_MM,
  uavSizeYMm: UAV_SIZE_Y_MM,
  uavSizeZMm: UAV_SIZE_Z_MM,
  lampDiameterMm: LAMP_DIAMETER_MM,
  lampSegments: LAMP_SEGMENTS_DEFAULT,
  hfovDeg: CAMERA_HFOV_DEG,
  dt: SIM_DT_S,
  rTh: R_TH_DEFAULT,
  lampTargetYM: 0,
};

export const DEFAULT_DISPLAY: DisplayOptions = {
  trail: true,
  shadow: true,
  frustum: true,
  comb: true,
  fpv: true,
  curve: true,
  lampTrack: true,
  gridPlane: 'horizontal',
  showCoarse: true,
  showFine: true,
  showAxes: true,
  gridCoarseColor: '#4a4a4a',
  gridFineColor: '#383838',
  gridCoarseWidth: 1,
  gridFineWidth: 1,
};

const GRID_PLANES: readonly GridPlane[] = ['lamp', 'horizontal', 'both'];
const HEX_COLOR = /^#[0-9a-f]{6}$/i;

/**
 * 显示选项净化：旧存档（缺字段）自动补默认值，非法值回落默认。
 * 纯函数、不碰 DOM —— 保证 localStorage 存档在新增字段后依然可用。
 */
export function sanitizeDisplay(raw?: unknown): DisplayOptions {
  if (raw === null || typeof raw !== 'object' || Array.isArray(raw)) {
    return { ...DEFAULT_DISPLAY };
  }
  const r = raw as Record<string, unknown>;
  const bool = (
    key:
      | 'trail'
      | 'shadow'
      | 'frustum'
      | 'comb'
      | 'fpv'
      | 'curve'
      | 'lampTrack'
      | 'showCoarse'
      | 'showFine'
      | 'showAxes',
  ): boolean => (typeof r[key] === 'boolean' ? (r[key] as boolean) : DEFAULT_DISPLAY[key]);
  const color = (key: 'gridCoarseColor' | 'gridFineColor'): string =>
    typeof r[key] === 'string' && HEX_COLOR.test(r[key] as string)
      ? (r[key] as string)
      : DEFAULT_DISPLAY[key];
  const width = (key: 'gridCoarseWidth' | 'gridFineWidth'): number => {
    const v = r[key];
    if (typeof v !== 'number' || !Number.isFinite(v)) return DEFAULT_DISPLAY[key];
    return Math.min(GRID_WIDTH_RANGE[1], Math.max(GRID_WIDTH_RANGE[0], v));
  };
  return {
    trail: bool('trail'),
    shadow: bool('shadow'),
    frustum: bool('frustum'),
    comb: bool('comb'),
    fpv: bool('fpv'),
    curve: bool('curve'),
    lampTrack: bool('lampTrack'),
    gridPlane: GRID_PLANES.includes(r.gridPlane as GridPlane)
      ? (r.gridPlane as GridPlane)
      : DEFAULT_DISPLAY.gridPlane,
    showCoarse: bool('showCoarse'),
    showFine: bool('showFine'),
    showAxes: bool('showAxes'),
    gridCoarseColor: color('gridCoarseColor'),
    gridFineColor: color('gridFineColor'),
    gridCoarseWidth: width('gridCoarseWidth'),
    gridFineWidth: width('gridFineWidth'),
  };
}

/** 参数范围（用于 Tweakpane 校验与 clamp）。 */
export const PARAM_RANGES = {
  v0: [15.0, 30.0] as [number, number],
  theta0Deg: [15.0, 45.0] as [number, number],
  speedDecayPerS: [0.0, 1.5] as [number, number],
  standoffM: [0.05, 0.3] as [number, number],
  offsetUM: [-0.1, 0.1] as [number, number],
  offsetVM: [-0.1, 0.1] as [number, number],
  uavSizeXMm: [50.0, 200.0] as [number, number],
  uavSizeYMm: [50.0, 200.0] as [number, number],
  uavSizeZMm: [50.0, 200.0] as [number, number],
  lampDiameterMm: [30.0, 100.0] as [number, number],
  hfovDeg: [30.0, 120.0] as [number, number],
  dt: [0.005, 0.05] as [number, number],
  rTh: [R_TH_MIN, R_TH_MAX] as [number, number],
  lampTargetYM: [LAMP_Y_RANGE_MM[0] / 1000, LAMP_Y_RANGE_MM[1] / 1000] as [number, number],
};

/** HUD 配色（深色主题）。 */
export const COLORS = {
  background: '#0f172a',
  primary: '#3b82f6',
  primaryGlow: 'rgba(59, 130, 246, 0.25)',
  threshold: '#ef4444',
  effective: '#10b981',
  effectiveFill: 'rgba(16, 185, 129, 0.35)',
  effectiveFillEnd: 'rgba(16, 185, 129, 0.0)',
  text: '#f8fafc',
  accent: '#f59e0b',
  grid: '#1e293b',
} as const;
