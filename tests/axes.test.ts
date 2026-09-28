/**
 * 三色坐标轴单测（Node 环境，无需 WebGL）。
 *
 * Line2 / LineGeometry / LineMaterial 都是纯 JS 对象：构造只创建几何与 uniform 容器，
 * 只有 WebGLRenderer 才需要 GL 上下文，因此颜色与线宽可以在 Node 里直接断言。
 */

import type { Line2 } from 'three/examples/jsm/lines/Line2.js';
import { describe, expect, it } from 'vitest';

import { AXIS_COLORS } from '../src/config';
import { buildAxes } from '../src/render/grid';

const hex = (value: number): string => value.toString(16).padStart(6, '0');

describe('buildAxes', () => {
  const range = 30;
  const axes = buildAxes(range);

  it('返回 3 条 Line2', () => {
    expect(axes).toHaveLength(3);
    for (const line of axes) expect(line.isLine2).toBe(true);
  });

  it('每条轴各自持有独立材质（便于分别设色/线宽）', () => {
    const materials: unknown[] = axes.map((line: Line2) => line.material);
    expect(new Set(materials).size).toBe(3);
  });

  it('三色从 config.AXIS_COLORS 读取（测试不硬编码色值）', () => {
    expect(axes[0]!.material.color.getHexString()).toBe(hex(AXIS_COLORS.x));
    expect(axes[1]!.material.color.getHexString()).toBe(hex(AXIS_COLORS.y));
    expect(axes[2]!.material.color.getHexString()).toBe(hex(AXIS_COLORS.z));
  });

  it('线宽为 2（CSS 像素口径），且为屏幕单位而非世界单位', () => {
    for (const line of axes) {
      expect(line.material.linewidth).toBe(2);
      expect(line.material.worldUnits).toBe(false);
    }
  });

  it('renderOrder = -1、不参与视锥剔除、不写深度', () => {
    for (const line of axes) {
      expect(line.renderOrder).toBe(-1);
      expect(line.frustumCulled).toBe(false);
      expect(line.material.depthWrite).toBe(false);
    }
  });

  it('每条轴是 1 段，端点覆盖 ±range', () => {
    for (const line of axes) {
      const start = line.geometry.attributes.instanceStart;
      const end = line.geometry.attributes.instanceEnd;
      expect(start?.count).toBe(1);
      expect(end?.count).toBe(1);
    }
    // X 轴：从 -range 到 +range
    const xStart = axes[0]!.geometry.attributes.instanceStart!;
    const xEnd = axes[0]!.geometry.attributes.instanceEnd!;
    expect(xStart.getX(0)).toBeCloseTo(-range, 6);
    expect(xEnd.getX(0)).toBeCloseTo(range, 6);
    // Y 轴沿 Y，Z 轴沿 Z
    expect(axes[1]!.geometry.attributes.instanceStart!.getY(0)).toBeCloseTo(-range, 6);
    expect(axes[2]!.geometry.attributes.instanceStart!.getZ(0)).toBeCloseTo(-range, 6);
  });

  it('resolution 挂在材质上（Line2 只持有材质）并可写', () => {
    for (const line of axes) {
      line.material.resolution.set(1920, 1080);
      expect(line.material.resolution.x).toBeCloseTo(1920, 6);
      expect(line.material.resolution.y).toBeCloseTo(1080, 6);
    }
  });
});
