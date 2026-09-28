/**
 * M2 运动学单测：灯横向平移 / 线段-点距离 / 无人机一维规划 / 飞镖六步积分。
 *
 * 断言口径来自 M2 定稿：分段边界、时间最优规划（三角/梯形/远离目标）、
 * 以及"追踪段保持速率、每步转角 ≤ ω_max·dt"两条硬约束。
 */

import { describe, expect, it } from 'vitest';

import { DEFAULT_PARAMS } from '../src/config';
import {
  lampCenterAt,
  lampVyAt,
  lampYAt,
  segmentToPointDistance,
  stepDartM2,
  uavStep,
} from '../src/core/motion';
import { makeCameraConfig } from '../src/core/simulate';

const camera = makeCameraConfig(DEFAULT_PARAMS);

describe('灯横向平移（分段线性，T1=1.2 s → 1.8 s）', () => {
  it('t < 1.2 不动；1.2 ≤ t < 1.8 匀速；t ≥ 1.8 到位', () => {
    expect(lampYAt(0, 0.24)).toBe(0);
    expect(lampYAt(1.19, 0.24)).toBe(0);
    expect(lampYAt(1.2, 0.24)).toBe(0);
    expect(lampYAt(1.5, 0.24)).toBeCloseTo(0.12, 12);
    expect(lampYAt(1.79, 0.24)).toBeCloseTo(0.236, 12);
    expect(lampYAt(1.8, 0.24)).toBe(0.24);
    expect(lampYAt(2.0, 0.24)).toBe(0.24);
  });

  it('负方向对称（±240 mm 物理限位）', () => {
    expect(lampYAt(1.5, -0.24)).toBeCloseTo(-0.12, 12);
    expect(lampYAt(2.0, -0.24)).toBe(-0.24);
    expect(lampYAt(0.5, -0.24)).toBe(0);
  });

  it('灯速：移动区间内为 yTarget / 0.6，其余为 0', () => {
    expect(lampVyAt(0.5, 0.24)).toBe(0);
    expect(lampVyAt(1.19, 0.24)).toBe(0);
    expect(lampVyAt(1.2, 0.24)).toBeCloseTo(0.4, 12);
    expect(lampVyAt(1.5, 0.24)).toBeCloseTo(0.4, 12);
    expect(lampVyAt(1.799, 0.24)).toBeCloseTo(0.4, 12);
    expect(lampVyAt(1.8, 0.24)).toBe(0);
    expect(lampVyAt(2.0, 0.24)).toBe(0);
  });

  it('lampCenterAt 只改 Y（灯在原点平面内平移）', () => {
    expect(lampCenterAt(1.5, 0.24)).toEqual([0, 0.12, 0]);
    expect(lampCenterAt(0, 0.24)).toEqual([0, 0, 0]);
  });
});

describe('segmentToPointDistance（命中判据的线段-点距离）', () => {
  it('垂足在线段内 → 点到直线距离', () => {
    expect(segmentToPointDistance([0, 0, 0], [1, 0, 0], [0.5, 0.1, 0])).toBeCloseTo(0.1, 12);
  });

  it('垂足在线段外 → 取近端点距离', () => {
    expect(segmentToPointDistance([0, 0, 0], [1, 0, 0], [2, 0, 0])).toBeCloseTo(1.0, 12);
  });

  it('退化线段（A = B）→ 直接取 |Q − A|', () => {
    expect(segmentToPointDistance([0, 0, 0], [0, 0, 0], [1, 0, 0])).toBeCloseTo(1.0, 12);
  });
});

describe('uavStep（一维时间最优规划）', () => {
  it('0 → 0.24 m：离散步进下 ~0.70 s 到位，峰值速度 ≈ √(2·0.24)', () => {
    let y = 0;
    let v = 0;
    let step = 0;
    let peak = 0;
    for (let i = 0; i < 500; i += 1) {
      const st = uavStep(y, v, 0.24, 5, 2, 0.01);
      y = st.y;
      v = st.v;
      peak = Math.max(peak, Math.abs(v));
      step += 1;
      if (y >= 0.24 - 1e-12 && Math.abs(v) < 1e-12) break;
    }
    // 解析最短 0.6928 s；按 dt = 0.01 s 的逐步规划落在 0.70~0.71 s
    expect(step).toBeGreaterThanOrEqual(69);
    expect(step).toBeLessThanOrEqual(71);
    expect(y).toBeCloseTo(0.24, 12);
    expect(v).toBeCloseTo(0, 12);
    // 解析峰值 √(2·0.24) ≈ 0.6928；离散步进（0.01 s）略低（≈ 0.6856）
    expect(peak).toBeGreaterThan(0.68);
    expect(peak).toBeLessThanOrEqual(Math.sqrt(0.48) + 1e-12);
  });

  it('0 → 100 m：速度封顶 vMax = 5 m/s（梯形曲线），且不超调', () => {
    let y = 0;
    let v = 0;
    let peak = 0;
    for (let i = 0; i < 5000; i += 1) {
      const st = uavStep(y, v, 100, 5, 2, 0.01);
      y = st.y;
      v = st.v;
      peak = Math.max(peak, Math.abs(v));
    }
    expect(peak).toBeCloseTo(5.0, 12);
    expect(peak).toBeLessThanOrEqual(5.0 + 1e-12);
    expect(y).toBeGreaterThan(0);
  });

  it('远离目标（v0 = −1 而目标在 +Y）：整步减速不越界', () => {
    const st = uavStep(0, -1, 0.24, 5, 2, 0.01);
    expect(st.y).toBeCloseTo(-0.0099, 12);
    expect(st.v).toBeCloseTo(-0.98, 12);
  });

  it('已在目标且速度小 → 立即归零；速度大 → 一步减速但位置不越过目标', () => {
    expect(uavStep(0.24, 0, 0.24, 5, 2, 0.01)).toEqual({ y: 0.24, v: 0 });
    expect(uavStep(0.24, 0.01, 0.24, 5, 2, 0.01)).toEqual({ y: 0.24, v: 0 });
    const braking = uavStep(0.24, -0.6, 0.24, 5, 2, 0.01);
    expect(braking.y).toBeCloseTo(0.24, 12);
    expect(braking.v).toBeCloseTo(-0.58, 12);
  });
});

describe('stepDartM2（M2 六步积分）', () => {
  const base = {
    dt: 0.01,
    k: 0.55,
    g: 9.8,
    omegaMaxRad: (60 * Math.PI) / 180,
    camera,
  };

  it('弹道段：V_next 逐分量等于 V + a_phys·dt', () => {
    const P = [-25.03705, -2.86818, -0.56904];
    const V = [10, 1, 8];
    const L = [0, 0, 0];
    const out = stepDartM2({ ...base, P, V, L });
    expect(out.locked).toBe(false);
    const a = [-base.k * V[0]!, -base.k * V[1]!, -base.g - base.k * V[2]!];
    for (let i = 0; i < 3; i += 1) {
      expect(out.V_next[i]!).toBe(V[i]! + a[i]! * base.dt);
    }
  });

  it('追踪段：速率守恒 |V_next| = |V + a_phys·dt|（1e-12）', () => {
    const P = [-5, -0.5, 2];
    const V = [8, 0.2, 0.5];
    const L = [0, 0, 0];
    const out = stepDartM2({ ...base, P, V, L });
    expect(out.locked).toBe(true);
    const a = [-base.k * V[0]!, -base.k * V[1]!, -base.g - base.k * V[2]!];
    const vPhys = [V[0]! + a[0]! * base.dt, V[1]! + a[1]! * base.dt, V[2]! + a[2]! * base.dt];
    const mag = (u: readonly number[]): number => Math.hypot(u[0]!, u[1]!, u[2]!);
    expect(Math.abs(mag(out.V_next) - mag(vPhys))).toBeLessThanOrEqual(1e-12);
  });

  it('追踪段：方向转角 ≤ ω_max·dt（限幅生效）', () => {
    const P = [-5, -0.5, 2];
    const V = [8, 0.2, 0.5];
    const L = [0, 0, 0];
    const out = stepDartM2({ ...base, P, V, L });
    const unit = (u: readonly number[]): number[] => {
      const m = Math.hypot(u[0]!, u[1]!, u[2]!);
      return [u[0]! / m, u[1]! / m, u[2]! / m];
    };
    const d0 = unit(V);
    const d1 = unit(out.V_next);
    const cos = Math.min(
      1,
      Math.max(-1, d0[0]! * d1[0]! + d0[1]! * d1[1]! + d0[2]! * d1[2]!),
    );
    const turn = Math.acos(cos);
    expect(turn).toBeLessThanOrEqual(base.omegaMaxRad * base.dt + 1e-12);
    expect(turn).toBeGreaterThan(0);
  });

  it('锁定时若目标方向在 ω_max·dt 内 → 直接对齐 d_aim', () => {
    // 灯几乎正前方：所需转角远小于 1.047e-2 rad
    const P = [-0.5, 0, 0];
    const V = [10, 0, 0];
    const L = [0, 0, 0];
    const out = stepDartM2({ ...base, P, V, L });
    expect(out.locked).toBe(true);
    const m = Math.hypot(out.V_next[0]!, out.V_next[1]!, out.V_next[2]!);
    // d_aim = (1,0,0) → V_next 应平行 +X
    expect(out.V_next[0]! / m).toBeCloseTo(1, 12);
    expect(out.V_next[1]! / m).toBeCloseTo(0, 12);
    expect(out.V_next[2]! / m).toBeCloseTo(0, 12);
  });

  it('速度退化（|V| ≈ 1e-13）不产生 NaN，且不主动转向', () => {
    const out = stepDartM2({
      ...base,
      P: [-0.5, 0, 1],
      V: [1e-13, 0, 0],
      L: [0, 0, 0],
    });
    for (const v of out.V_next) expect(Number.isFinite(v)).toBe(true);
    for (const p of out.P_next) expect(Number.isFinite(p)).toBe(true);
  });
});
