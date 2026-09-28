/**
 * 凸多边形交集交叉验证 + 性能对比。
 *
 * 运行时用的是自写 Sutherland–Hodgman（凸-凸精确裁剪）；这里用 npm 包
 * `polygon-clipping`（通用布尔运算，成熟实现）作为独立参照，逐帧比对交集面积，
 * 并给出两种实现的耗时对比——这正是计划里"若热点超预算则换自写裁剪"的判定依据。
 */

import polygonClipping from 'polygon-clipping';
import { describe, expect, it } from 'vitest';

import { DEFAULT_PARAMS } from '../src/config';
import {
  intersectConvexPolygons,
  makeConvexPolygon,
} from '../src/core/geometry';
import { runSimulation } from '../src/core/simulate';

type Pair = [number, number];

function closeRing(points: readonly number[][]): Pair[] {
  const ring: Pair[] = points.map((p) => [p[0]!, p[1]!]);
  ring.push([points[0]![0]!, points[0]![1]!]);
  return ring;
}

function polygonClippingArea(a: readonly number[][], b: readonly number[][]): number {
  const result = polygonClipping.intersection([closeRing(a)], [closeRing(b)]);
  let area = 0;
  for (const polygon of result) {
    for (let r = 0; r < polygon.length; r += 1) {
      const ring = polygon[r]!;
      let acc = 0;
      for (let i = 0; i < ring.length; i += 1) {
        const p = ring[i]!;
        const q = ring[(i + 1) % ring.length]!;
        acc += p[0]! * q[1]! - q[0]! * p[1]!;
      }
      const sign = r === 0 ? 1 : -1; // 第一个环是外环，其余为孔洞
      area += (sign * Math.abs(acc)) / 2;
    }
  }
  return area;
}

describe('凸多边形交集：自写裁剪 vs polygon-clipping', () => {
  const result = runSimulation(DEFAULT_PARAMS);
  // M2：灯球多边形按当前 lampSegments 离散，整段不变，直接取第 0 帧的 LampFrame
  const lamp = result.lampFrames[0]!.polygon2D;

  it('逐帧面积一致（|Δ| ≤ 1e-9 m²）', () => {
    let maxDiff = 0;
    let worst = -1;
    for (let i = 0; i < result.n; i += 1) {
      const count = result.shadowCount[i]!;
      if (count < 3) continue;
      const shadowPts: number[][] = [];
      for (let k = 0; k < count; k += 1) {
        shadowPts.push([result.shadow2D[i * 16 + k * 2]!, result.shadow2D[i * 16 + k * 2 + 1]!]);
      }
      const shadow = makeConvexPolygon(shadowPts);
      const mine = intersectConvexPolygons(shadow, lamp).area;
      const reference = polygonClippingArea(shadow.points, lamp.points);
      const diff = Math.abs(mine - reference);
      if (diff > maxDiff) {
        maxDiff = diff;
        worst = i;
      }
    }
    expect(maxDiff, `worst frame #${worst}`).toBeLessThanOrEqual(1e-9);
  });

  it('自写裁剪在 1024 边形上更快（记录耗时对比）', () => {
    const shadowPtsList: number[][][] = [];
    for (let i = 0; i < result.n; i += 1) {
      const count = result.shadowCount[i]!;
      if (count < 3) continue;
      const pts: number[][] = [];
      for (let k = 0; k < count; k += 1) {
        pts.push([result.shadow2D[i * 16 + k * 2]!, result.shadow2D[i * 16 + k * 2 + 1]!]);
      }
      shadowPtsList.push(pts);
    }
    // 预热
    for (const pts of shadowPtsList.slice(0, 10)) {
      const s = makeConvexPolygon(pts);
      intersectConvexPolygons(s, lamp);
      polygonClippingArea(s.points, lamp.points);
    }
    const t0 = performance.now();
    for (const pts of shadowPtsList) intersectConvexPolygons(makeConvexPolygon(pts), lamp);
    const mineMs = performance.now() - t0;
    const t1 = performance.now();
    for (const pts of shadowPtsList) polygonClippingArea(pts, lamp.points);
    const refMs = performance.now() - t1;
    // eslint-disable-next-line no-console
    console.log(
      `[polygon] ${shadowPtsList.length} 帧交集：自写 Sutherland–Hodgman ${mineMs.toFixed(
        1,
      )} ms vs polygon-clipping ${refMs.toFixed(1)} ms`,
    );
    expect(mineMs).toBeGreaterThan(0);
    expect(refMs).toBeGreaterThan(0);
  });
});
