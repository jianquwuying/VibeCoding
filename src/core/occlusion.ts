/**
 * 遮挡率计算 R_occ = A_blocked / A_lamp（M2：视线方向圆盘近似）。
 *
 * 灯已升级为**球体**：把无人机 8 顶点从相机投影到「过灯心、且垂直于当前视线」的平面，
 * 取其凸包与半径 r 的圆盘（平面局部二维多边形）求交；分母仍为**解析面积** πr²，
 * 因此"完全遮挡"时 R_occ ≈ 0.999994 而非 1.0。
 *
 * 对应 Python ``tools/python_ref/occlusion.py::compute_occlusion``。
 */

import {
  GeometryError,
  intersectConvexPolygons,
  makeConvexPolygon,
  polygonArea,
  projectUavShadow,
  type ConvexPolygon,
} from './geometry';

/** 一帧的灯状态（球心 + 视线方向 + 圆盘近似）。 */
export interface LampFrame {
  /** 灯心世界坐标 L(t) */
  center: readonly number[];
  /** 视线方向 normalize(L(t) − C) */
  losHat: readonly number[];
  /** 球半径 [m] */
  radiusM: number;
  /** 解析面积 πr²（R_occ 的分母） */
  areaM2: number;
  /** 平面局部坐标下的圆盘多边形（与 losHat 无关：只是过 L 的平面内的圆） */
  polygon2D: ConvexPolygon;
}

export interface OcclusionResult {
  /**
   * 灯球被阴影覆盖的面积比。
   *
   * 无效几何（射线与灯平面平行 / 阴影投影退化）与"阴影完全没碰到灯"都返回 0——
   * 帧级"几何前提是否成立"由 `SimResult.valid` 单独表达，本结构不再重复标注。
   */
  Rocc: number;
  /** 平面局部二维坐标下的阴影凸包顶点 */
  shadowPolygon: number[][];
  /** 世界坐标下的 8 个投影点 */
  shadowPoints3D: number[][];
}

const EMPTY: number[][] = [];

function aabbOverlap(
  a: { minX: number; minY: number; maxX: number; maxY: number },
  b: { minX: number; minY: number; maxX: number; maxY: number },
): boolean {
  return !(a.maxX < b.minX || b.maxX < a.minX || a.maxY < b.minY || b.maxY < a.minY);
}

/**
 * 计算一帧的几何遮挡率。
 *
 * @param C          相机（飞镖）位置 [m]
 * @param uavCenter  无人机中心 [m]
 * @param uavSize    无人机三轴尺寸 [m]
 * @param lamp       该帧灯状态（LampFrame）
 */
export function computeOcclusion(
  C: readonly number[],
  uavCenter: readonly number[],
  uavSize: readonly number[],
  lamp: LampFrame,
): OcclusionResult {
  let shadowPolygon: number[][];
  let shadowPoints3D: number[][];
  try {
    const shadow = projectUavShadow(C, uavCenter, uavSize, lamp.center, lamp.losHat);
    shadowPolygon = shadow.polygon2D;
    shadowPoints3D = shadow.points3D;
  } catch (error) {
    if (!(error instanceof GeometryError)) throw error;
    return { Rocc: 0, shadowPolygon: EMPTY, shadowPoints3D: EMPTY };
  }

  // 退化几何（空 / 共线 / 重合）→ R_occ = 0。
  if (shadowPolygon.length < 3 || polygonArea(shadowPolygon) <= 0) {
    return { Rocc: 0, shadowPolygon, shadowPoints3D };
  }

  const shadow = makeConvexPolygon(shadowPolygon);
  if (!aabbOverlap(shadow.bounds, lamp.polygon2D.bounds)) {
    return { Rocc: 0, shadowPolygon, shadowPoints3D };
  }

  const { area } = intersectConvexPolygons(shadow, lamp.polygon2D);
  const ratio = area / lamp.areaM2;
  const Rocc = Math.min(1, Math.max(0, ratio));
  return { Rocc, shadowPolygon, shadowPoints3D };
}

/**
 * 遮挡余量 [m] = 灯心（平面局部原点）到阴影多边形最近边的距离 − 灯球半径。
 *
 * ``>= 0`` 表示灯球被阴影完全覆盖（R_occ 取满值）；``< 0`` 表示灯球外露。
 * 与 Python ``experiment.occlusion_margin`` 同式，用于诊断"R 为什么是/不是 1.0"。
 */
export function occlusionMargin(
  shadowPolygon: readonly number[][],
  lampRadiusM: number,
): number {
  if (shadowPolygon.length < 3) return Number.NEGATIVE_INFINITY;
  let best = Number.POSITIVE_INFINITY;
  for (let i = 0; i < shadowPolygon.length; i += 1) {
    const a = shadowPolygon[i]!;
    const b = shadowPolygon[(i + 1) % shadowPolygon.length]!;
    const ex = b[0]! - a[0]!;
    const ey = b[1]! - a[1]!;
    const len = Math.hypot(ex, ey);
    if (len < 1e-15) continue;
    // 原点到直线 ab 的距离（平面局部坐标下灯心即原点）
    const dist = Math.abs(ex * (0 - a[1]!) - ey * (0 - a[0]!)) / len;
    if (dist < best) best = dist;
  }
  if (!Number.isFinite(best)) return Number.NEGATIVE_INFINITY;
  return best - lampRadiusM;
}
