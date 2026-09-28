/**
 * 数值一致性测试：Web 端 M2 核心算法 vs Python 参考实现（`tools/python_ref/` 黄金基准）。
 *
 * 容差：
 *   n / tLast / tLock / tHit / hitLamp 严格一致；
 *   Pd / Vd / lampPos / uavPos / lampY / uavY / uavVy 逐点 |Δ| ≤ 1e-9；
 *   有效帧 R / imageR / fovMarginDeg 逐点 |Δ| ≤ 1e-6；
 *   locked / valid 逐帧一致；R 的 null ⟺ TS 端 Number.isNaN(R[i])。
 */

import { describe, expect, it } from 'vitest';

import {
  DEFAULT_PARAMS,
  LAMP_NORMAL,
  type SimParams,
} from '../src/config';
import { losBasis, planeBasis, uavCorners } from '../src/core/geometry';
import { isFullyVisible, lampVyAt, lampYAt, segmentToPointDistance, stepDartM2, uavStep } from '../src/core/motion';
import { runSimulation } from '../src/core/simulate';

import defaultCase from './golden/default.json';
import geometryCase from './golden/geometry.json';
import lamp64Case from './golden/lamp_segments_64.json';
import lampYNeg from './golden/lamp_y_neg.json';
import lampYPos from './golden/lamp_y_pos.json';
import lampYPosV022 from './golden/lamp_y_pos_v0_22.json';
import standoff012 from './golden/standoff_012.json';
import standoff025 from './golden/standoff_025.json';
import toolsParity from './golden/tools_parity.json';
import uavRectangular from './golden/uav_rectangular.json';

const R_TOL = 1e-6;
const POS_TOL = 1e-9;
const FUNC_TOL = 1e-12;

type MaybeNumber = number | null;

interface GoldenCase {
  params: {
    standoff_m: number;
    offset_u_m: number;
    offset_v_m: number;
    /** v3：三轴独立尺寸 [X, Y, Z]，单位 mm */
    uav_size_mm: number[];
    lamp_diameter_mm: number;
    lamp_quad_segs: number;
    lamp_segments: number;
    hfov_deg: number;
    v0_mps: number;
    theta0_deg: number;
    speed_decay_per_s: number;
    dt: number;
    R_th: number;
    lamp_target_y_m: number;
  };
  n: number;
  tLast: number;
  lampYTargetM: number;
  tLock: number;
  tHit: MaybeNumber;
  hitLamp: boolean;
  lampY: number[];
  lampPos: number[];
  uavY: number[];
  uavVy: number[];
  uavPos: number[];
  Pd: number[];
  Vd: number[];
  R: MaybeNumber[];
  imageR: number[];
  fovMarginDeg: number[];
  locked: number[];
  valid: number[];
  lamp: { radiusM: number; areaM2: number; polygonAreaM2: number };
  camera: { width: number; height: number; fovHDeg: number; focalPx: number };
  marginMinM: number;
  marginMinT: number;
}

function paramsFromGolden(g: GoldenCase): SimParams {
  return {
    ...DEFAULT_PARAMS,
    v0: g.params.v0_mps,
    theta0Deg: g.params.theta0_deg,
    speedDecayPerS: g.params.speed_decay_per_s,
    standoffM: g.params.standoff_m,
    offsetUM: g.params.offset_u_m,
    offsetVM: g.params.offset_v_m,
    uavSizeXMm: g.params.uav_size_mm[0]!,
    uavSizeYMm: g.params.uav_size_mm[1]!,
    uavSizeZMm: g.params.uav_size_mm[2]!,
    lampDiameterMm: g.params.lamp_diameter_mm,
    lampSegments: g.params.lamp_segments,
    hfovDeg: g.params.hfov_deg,
    dt: g.params.dt,
    rTh: g.params.R_th,
    lampTargetYM: g.params.lamp_target_y_m,
  };
}

/** 扁平数组逐点最大偏差。 */
function maxFlatDiff(a: ArrayLike<number>, b: readonly number[]): number {
  let worst = 0;
  for (let i = 0; i < b.length; i += 1) worst = Math.max(worst, Math.abs(a[i]! - b[i]!));
  return worst;
}

function checkCase(name: string, g: GoldenCase): void {
  const result = runSimulation(paramsFromGolden(g));

  it(`${name}: n / tLast / tLock / tHit / hitLamp 严格一致`, () => {
    expect(result.n).toBe(g.n);
    expect(result.tLast).toBe(g.tLast);
    expect(result.tLock).toBe(g.tLock);
    expect(result.hitLamp).toBe(g.hitLamp);
    if (g.tHit === null) expect(result.tHit).toBeNull();
    else expect(result.tHit).toBe(g.tHit);
  });

  it(`${name}: Pd / Vd / lampPos / uavPos / lampY / uavVy 逐点对齐（|Δ| ≤ ${POS_TOL}）`, () => {
    expect(maxFlatDiff(result.Pd, g.Pd)).toBeLessThanOrEqual(POS_TOL);
    expect(maxFlatDiff(result.Vd, g.Vd)).toBeLessThanOrEqual(POS_TOL);
    expect(maxFlatDiff(result.lampPos, g.lampPos)).toBeLessThanOrEqual(POS_TOL);
    expect(maxFlatDiff(result.uavPos, g.uavPos)).toBeLessThanOrEqual(POS_TOL);
    expect(maxFlatDiff(result.lampY, g.lampY)).toBeLessThanOrEqual(POS_TOL);
    expect(maxFlatDiff(result.uavVy, g.uavVy)).toBeLessThanOrEqual(POS_TOL);
    // 逐帧无人机中心 Y（lampY 之外的另一路读数）
    for (let i = 0; i < result.n; i += 1) {
      expect(Math.abs(result.uavPos[i * 3 + 1]! - g.uavPos[i * 3 + 1]!)).toBeLessThanOrEqual(POS_TOL);
    }
  });

  it(`${name}: R 的 null/NaN 位置严格一致，有效帧 |Δ| ≤ ${R_TOL}`, () => {
    let maxDiff = 0;
    let worstIndex = -1;
    for (let i = 0; i < g.R.length; i += 1) {
      const golden = g.R[i];
      const mine = result.R[i]!;
      if (golden === null) {
        expect(Number.isNaN(mine), `frame #${i} 应为无效帧（R = NaN）`).toBe(true);
        continue;
      }
      expect(Number.isNaN(mine), `frame #${i} 应为有效帧`).toBe(false);
      const d = Math.abs(mine - golden);
      if (d > maxDiff) {
        maxDiff = d;
        worstIndex = i;
      }
    }
    expect(maxDiff, `worst frame #${worstIndex}`).toBeLessThanOrEqual(R_TOL);
  });

  it(`${name}: locked / valid 逐帧一致`, () => {
    expect(Array.from(result.locked)).toEqual(g.locked);
    expect(Array.from(result.valid)).toEqual(g.valid);
  });

  it(`${name}: 灯球几何与相机配置一致`, () => {
    expect(Math.abs(result.lampRadiusM - g.lamp.radiusM)).toBeLessThanOrEqual(POS_TOL);
    expect(Math.abs(result.lampAreaM2 - g.lamp.areaM2)).toBeLessThanOrEqual(1e-18);
    expect(result.camera.width).toBe(g.camera.width);
    expect(result.camera.height).toBe(g.camera.height);
    expect(result.camera.fovHDeg).toBe(g.camera.fovHDeg);
  });
}

checkCase('默认 (20, 30, yT=0)', defaultCase as unknown as GoldenCase);
checkCase('灯 Y = +0.24 m', lampYPos as unknown as GoldenCase);
checkCase('灯 Y = −0.24 m', lampYNeg as unknown as GoldenCase);
checkCase('灯 Y = +0.24 m & v0 = 22', lampYPosV022 as unknown as GoldenCase);
checkCase('standoff = 0.12（满遮挡）', standoff012 as unknown as GoldenCase);
checkCase('standoff = 0.25（末端失效）', standoff025 as unknown as GoldenCase);
checkCase('LampSegments = 64', lamp64Case as unknown as GoldenCase);
checkCase('三轴机体 (150, 70, 90)', uavRectangular as unknown as GoldenCase);

describe('黄金基准：像面遮挡比与视场余量（FPV）', () => {
  const g = defaultCase as unknown as GoldenCase;
  const result = runSimulation(paramsFromGolden(g));

  it('imageR 逐点对齐（|Δ| ≤ 1e-6）', () => {
    expect(maxFlatDiff(result.imageR, g.imageR)).toBeLessThanOrEqual(R_TOL);
  });

  it('fovMargin 逐点对齐（|Δ| ≤ 1e-6）', () => {
    expect(maxFlatDiff(result.fovMargin, g.fovMarginDeg)).toBeLessThanOrEqual(R_TOL);
  });
});

describe('黄金基准：单帧函数表（跨语言 ≤ 1e-12）', () => {
  const camera = toolsParity.camera;

  it('lampYAt 分段与边界', () => {
    for (const row of toolsParity.lampYAt) {
      expect(Math.abs(lampYAt(row.t, row.yTarget) - row.expected)).toBeLessThanOrEqual(FUNC_TOL);
    }
  });

  it('lampVyAt 分段与边界', () => {
    for (const row of toolsParity.lampVyAt) {
      expect(Math.abs(lampVyAt(row.t, row.yTarget) - row.expected)).toBeLessThanOrEqual(FUNC_TOL);
    }
  });

  it('stepDartM2（弹道段 / 追踪段 / 速度退化）', () => {
    for (const row of toolsParity.stepDartM2) {
      const out = stepDartM2({
        P: row.P,
        V: row.V,
        L: row.L,
        dt: row.dt,
        k: row.k,
        g: row.g,
        omegaMaxRad: row.omegaMaxRad,
        camera,
      });
      for (let a = 0; a < 3; a += 1) {
        expect(Math.abs(out.P_next[a]! - row.expected.P_next[a]!)).toBeLessThanOrEqual(FUNC_TOL);
        expect(Math.abs(out.V_next[a]! - row.expected.V_next[a]!)).toBeLessThanOrEqual(FUNC_TOL);
      }
      expect(out.locked).toBe(row.expected.locked);
    }
  });

  it('segmentToPointDistance', () => {
    for (const row of toolsParity.segmentToPointDistance) {
      expect(
        Math.abs(segmentToPointDistance(row.A, row.B, row.Q) - row.expected),
      ).toBeLessThanOrEqual(FUNC_TOL);
    }
  });

  it('uavStep（加速 / 匀速 / 减速 / 反向 / 已在目标）', () => {
    for (const row of toolsParity.uavStep) {
      const out = uavStep(row.y0, row.v0, row.y1, row.vMax, row.aMax, row.dt);
      expect(Math.abs(out.y - row.expected.y)).toBeLessThanOrEqual(FUNC_TOL);
      expect(Math.abs(out.v - row.expected.v)).toBeLessThanOrEqual(FUNC_TOL);
    }
  });
});

describe('黄金基准：纯几何函数', () => {
  it('uavCorners 顶点顺序与坐标一致', () => {
    const corners = uavCorners(geometryCase.center, geometryCase.size);
    expect(corners.length).toBe(8);
    for (let i = 0; i < 8; i += 1) {
      for (let a = 0; a < 3; a += 1) {
        expect(Math.abs(corners[i]![a]! - geometryCase.corners[i]![a]!)).toBeLessThanOrEqual(1e-15);
      }
    }
  });

  it('planeBasis 与 Python 完全一致（u=(0,-1,0), v=(0,0,1)）', () => {
    const [u, v] = planeBasis(LAMP_NORMAL);
    for (let a = 0; a < 3; a += 1) {
      expect(Math.abs(u[a]! - geometryCase.basisU[a]!)).toBeLessThanOrEqual(1e-15);
      expect(Math.abs(v[a]! - geometryCase.basisV[a]!)).toBeLessThanOrEqual(1e-15);
    }
  });

  it('losBasis（视线法线的平面基）与 Python 一致', () => {
    for (const row of geometryCase.losBasis) {
      const [u, v] = losBasis(row.C, row.L);
      for (let a = 0; a < 3; a += 1) {
        expect(Math.abs(u[a]! - row.u[a]!)).toBeLessThanOrEqual(FUNC_TOL);
        expect(Math.abs(v[a]! - row.v[a]!)).toBeLessThanOrEqual(FUNC_TOL);
      }
    }
  });

  it('isFullyVisible（J2 判据）与 Python 一致', () => {
    for (const row of geometryCase.isFullyVisible) {
      expect(isFullyVisible(row.P, row.d, row.L, toolsParity.camera)).toBe(row.expected);
    }
  });
});

describe('灯球离散精度演示（Q4 改进 1 的数值依据）', () => {
  it('64 边形相对 1024 边形的偏差为 0.0016（与 Python 一致）', () => {
    const p64: SimParams = { ...DEFAULT_PARAMS, lampSegments: 64 };
    const p1024: SimParams = { ...DEFAULT_PARAMS, lampSegments: 1024 };
    const r64 = runSimulation(p64).R;
    const r1024 = runSimulation(p1024).R;
    expect(r64[0]!).toBeCloseTo(0.998394, 6);
    expect(r1024[0]!).toBeCloseTo(0.999994, 6);
    expect(Math.abs(r1024[0]! - r64[0]!)).toBeCloseTo(0.0016, 3);
  });
});
