/**
 * v3 无人机三轴独立尺寸（长=X / 宽=Y / 高=Z）。
 *
 * 覆盖：`uavCorners` 的三轴顶点坐标与索引规则、`uavEdges` 12 条棱、
 * `bodySafeBounds` 逐轴收缩、`hoverStationPositionAt` 的三轴站位裁剪、
 * `runSimulation` 的三轴透传与"三轴互不联动"、`PARAM_RANGES` 与默认值。
 *
 * 注：测试环境是 node（vite.config.ts 的 `test` 未启用 jsdom），
 * 因此**不能**直接驱动 Tweakpane 面板；"无联动"改用纯数据路径断言。
 */

import { describe, expect, it } from 'vitest';

import { DEFAULT_PARAMS, PARAM_RANGES, type SimParams } from '../src/config';
import { uavCorners, uavEdges } from '../src/core/geometry';
import { bodySafeBounds, hoverStationPositionAt, runSimulation } from '../src/core/simulate';
import { sanitizeParams } from '../src/ui/panel';

/** 三轴显著不同的样例尺寸 [m]。 */
const SIZE = [0.15, 0.07, 0.09];

describe('uavCorners：三轴独立', () => {
  const corners = uavCorners([0, 0, 0], SIZE);

  it('8 个顶点逐坐标正确（索引 = 4a+2b+c，a/b/c 对应 x/y/z 正负）', () => {
    expect(corners.length).toBe(8);
    const half = [0.075, 0.035, 0.045];
    for (let k = 0; k < 8; k += 1) {
      expect(corners[k]![0]!).toBeCloseTo((k & 4 ? 1 : -1) * half[0]!, 15);
      expect(corners[k]![1]!).toBeCloseTo((k & 2 ? 1 : -1) * half[1]!, 15);
      expect(corners[k]![2]!).toBeCloseTo((k & 1 ? 1 : -1) * half[2]!, 15);
    }
  });

  it('前 4 个顶点固定 x = −0.075，后 4 个固定 x = +0.075', () => {
    expect(corners.slice(0, 4).every((c) => c[0]! < 0)).toBe(true);
    expect(corners.slice(4).every((c) => c[0]! > 0)).toBe(true);
  });

  it('idx^1 / idx^2 / idx^4 必为相邻顶点（每条棱共享两个坐标）', () => {
    for (const bit of [1, 2, 4]) {
      for (let k = 0; k < 8; k += 1) {
        const a = corners[k]!;
        const b = corners[k ^ bit]!;
        let same = 0;
        for (let axis = 0; axis < 3; axis += 1) {
          if (Math.abs(a[axis]! - b[axis]!) < 1e-15) same += 1;
        }
        expect(same).toBe(2);
      }
    }
  });

  it('长方体线框仍是 12 条棱', () => {
    expect(uavEdges().length).toBe(12);
  });
});

describe('bodySafeBounds：逐轴收缩', () => {
  it('三轴半尺寸各自生效（与 DEFAULT_UAV_BOUNDS 一致）', () => {
    const b = bodySafeBounds(SIZE);
    expect(b.x).toEqual([-26.0 + 0.075, 0.5 - 0.075]);
    expect(b.y).toEqual([-6.0 + 0.035, 6.0 - 0.035]);
    expect(b.z).toEqual([-2.0 + 0.045, 3.0 - 0.045]);
  });
});

describe('hoverStationPositionAt：三轴站位', () => {
  it('standoff 沿 −X，站位落在 (−0.15, 0, 0) 且在三轴安全边界内', () => {
    const params: SimParams = {
      ...DEFAULT_PARAMS,
      uavSizeXMm: 150,
      uavSizeYMm: 70,
      uavSizeZMm: 90,
      standoffM: 0.15,
      offsetUM: 0,
      offsetVM: 0,
    };
    const pos = hoverStationPositionAt(params, 0);
    expect(pos[0]!).toBeCloseTo(-0.15, 9);
    expect(pos[1]!).toBeCloseTo(0, 9);
    expect(pos[2]!).toBeCloseTo(0, 9);
  });
});

describe('runSimulation：三轴透传与互不联动', () => {
  it('SimResult.uavSizeM 三元组与输入一致', () => {
    const params: SimParams = {
      ...DEFAULT_PARAMS,
      uavSizeXMm: 150,
      uavSizeYMm: 70,
      uavSizeZMm: 90,
    };
    const result = runSimulation(params);
    expect(result.uavSizeM[0]!).toBeCloseTo(0.15, 12);
    expect(result.uavSizeM[1]!).toBeCloseTo(0.07, 12);
    expect(result.uavSizeM[2]!).toBeCloseTo(0.09, 12);
  });

  it('只改 X 时 Y / Z 完全不受影响（无任何联动逻辑）', () => {
    const base: SimParams = { ...DEFAULT_PARAMS, uavSizeXMm: 100, uavSizeYMm: 100, uavSizeZMm: 80 };
    const wider: SimParams = { ...base, uavSizeXMm: 200 };
    const a = hoverStationPositionAt(base, 0);
    const b = hoverStationPositionAt(wider, 0);
    expect(b[1]!).toBeCloseTo(a[1]!, 15);
    expect(b[2]!).toBeCloseTo(a[2]!, 15);
    expect(b[0]!).toBeCloseTo(a[0]!, 15); // standoff 沿 −X，X 尺寸不改变中心站位
    const ra = runSimulation(base);
    const rb = runSimulation(wider);
    expect(rb.uavSizeM[1]!).toBeCloseTo(ra.uavSizeM[1]!, 15);
    expect(rb.uavSizeM[2]!).toBeCloseTo(ra.uavSizeM[2]!, 15);
    // 机身变长 → 阴影变大 → 遮挡率不应变差
    const finite = (r: typeof ra): number[] => Array.from(r.R).filter((v) => Number.isFinite(v));
    expect(Math.min(...finite(rb))).toBeGreaterThanOrEqual(Math.min(...finite(ra)) - 1e-9);
  });
});

describe('三轴参数契约', () => {
  it('PARAM_RANGES 三条各自 [50, 200]', () => {
    expect(PARAM_RANGES.uavSizeXMm).toEqual([50, 200]);
    expect(PARAM_RANGES.uavSizeYMm).toEqual([50, 200]);
    expect(PARAM_RANGES.uavSizeZMm).toEqual([50, 200]);
  });

  it('DEFAULT_PARAMS = (100, 100, 80) 且三字段互相独立', () => {
    expect(DEFAULT_PARAMS.uavSizeXMm).toBe(100);
    expect(DEFAULT_PARAMS.uavSizeYMm).toBe(100);
    expect(DEFAULT_PARAMS.uavSizeZMm).toBe(80);
    expect(Object.keys(DEFAULT_PARAMS)).not.toContain('uavSizeMm');
  });

  it('sanitizeParams：越界 clamp 到 [50,200]、非有限数回落默认、三轴互不影响', () => {
    expect(sanitizeParams({ uavSizeXMm: 300 }).uavSizeXMm).toBe(200);
    expect(sanitizeParams({ uavSizeYMm: 10 }).uavSizeYMm).toBe(50);
    expect(sanitizeParams({ uavSizeZMm: Number.NaN }).uavSizeZMm).toBe(DEFAULT_PARAMS.uavSizeZMm);
    const out = sanitizeParams({ uavSizeXMm: 200 });
    expect(out.uavSizeYMm).toBe(DEFAULT_PARAMS.uavSizeYMm);
    expect(out.uavSizeZMm).toBe(DEFAULT_PARAMS.uavSizeZMm);
  });
});
