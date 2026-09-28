/**
 * 遮挡余量回归：
 *
 * `margin = 灯心到阴影多边形最近边的距离 − 灯球半径`，是"灯球是否被完全覆盖"的
 * 独立判据（与 `R_occ` 用同一套几何但不同算法）。两条被固化的事实（v3 默认值）：
 *
 *  1. **默认 (20, 30) + 机体 (100, 100, 80)**：最紧时刻 t = 0.75 s、余量
 *     ≈ **−3.36 mm < 0** → `R_min ≈ 0.974843`。即 **v3 默认不再完全遮挡**
 *     （v2 的 120 mm 立方体是 +18.09 mm / 0.999994）；机身高度 Z 是决定性那一维。
 *  2. **(22, 35)**：余量 ≈ −27.33 mm → `R_min ≈ 0.075741`（v2 是 −15.1 mm / 0.777）。
 *     S3.5 已三重验证这类失效是**模型特性**（θ0 大 → 弹道段更晚锁定 → 无人机站位
 *     相对灯心偏得更多），不是 `computeOcclusion` 的 bug。
 */

import { describe, expect, it } from 'vitest';

import { DEFAULT_PARAMS, type SimParams } from '../src/config';
import { runSimulation } from '../src/core/simulate';

const MM = 1000;

/** 有效帧（valid=1）的 R 最小值；无效帧（NaN）不参与统计。 */
function minValidR(result: { R: Float64Array }): number {
  let min = Number.POSITIVE_INFINITY;
  for (const r of result.R) if (Number.isFinite(r)) min = Math.min(min, r);
  return min;
}

describe('遮挡余量：默认参数 (v0=20, θ0=30, 机体 100/100/80)', () => {
  const result = runSimulation(DEFAULT_PARAMS);

  it('最紧时刻 t = 0.75 s，余量 ≈ −3.36 mm < 0（默认不再完全遮挡）', () => {
    expect(result.marginMinT).toBeCloseTo(0.75, 9);
    expect(Math.abs(result.marginMin * MM + 3.36)).toBeLessThanOrEqual(1.0);
    expect(result.marginMin).toBeLessThan(0);
  });

  it('R_min ≈ 0.974843（余量为负 ⟹ 灯球露出 ≈2.5% 面积，不再是 0.999994）', () => {
    expect(Math.abs(minValidR(result) - 0.974843)).toBeLessThanOrEqual(0.02);
  });

  it('最后一帧为无效帧（相机飞到无人机之后）：R = NaN', () => {
    expect(Number.isNaN(result.R[result.n - 1]!)).toBe(true);
  });
});

describe('遮挡余量：(v0=22, θ0=35) 的模型特性回归（S3.5）', () => {
  const params: SimParams = { ...DEFAULT_PARAMS, v0: 22.0, theta0Deg: 35.0 };
  const result = runSimulation(params);

  it('余量 ≈ −27.33 mm < 0（灯球露出一大块）', () => {
    expect(Math.abs(result.marginMin * MM + 27.33)).toBeLessThanOrEqual(1.0);
    expect(result.marginMin).toBeLessThan(0);
  });

  it('R_min ≈ 0.075741（余量为负 ⟹ R < 1；两者一致）', () => {
    expect(Math.abs(minValidR(result) - 0.075741)).toBeLessThanOrEqual(0.02);
  });
});
