/**
 * 场景网格单测（Node 环境，无需 WebGL）：几何段数 / 平面朝向 / 可见性矩阵 /
 * 样式只改材质 / resolution 传播 / 释放。
 *
 * 注意：three r169 的 LineSegmentsGeometry.setPositions() 会自动计算包围盒与包围球，
 * 段数必须看 attributes.instanceStart.count（geometry.instanceCount 恒为 Infinity，不可断言）。
 */

import * as THREE from 'three';
import type { LineSegments2 } from 'three/examples/jsm/lines/LineSegments2.js';
import { describe, expect, it } from 'vitest';

import { DEFAULT_DISPLAY, GRID_COARSE_OPACITY, GRID_FINE_OPACITY } from '../src/config';
import { SceneGrid } from '../src/render/grid';

function attr(object: LineSegments2, name: 'instanceStart' | 'instanceEnd'): THREE.InterleavedBufferAttribute {
  const a = object.geometry.attributes[name];
  if (!a) throw new Error(`缺少 ${name} 属性`);
  return a as THREE.InterleavedBufferAttribute;
}

function count(object: LineSegments2): number {
  return attr(object, 'instanceStart').count;
}

/** 校验某组网格的所有段端点都满足坐标约束（如灯球面 x=0）。 */
function allEndpointsSatisfy(
  object: LineSegments2,
  check: (v: { x: number; y: number; z: number }) => boolean,
): boolean {
  const start = attr(object, 'instanceStart');
  const end = attr(object, 'instanceEnd');
  for (let i = 0; i < start.count; i += 1) {
    const s = { x: start.getX(i), y: start.getY(i), z: start.getZ(i) };
    const e = { x: end.getX(i), y: end.getY(i), z: end.getZ(i) };
    if (!check(s) || !check(e)) return false;
  }
  return true;
}

describe('SceneGrid 结构', () => {
  const grid = new SceneGrid();

  it('根 group 恰好 5 个扁平子 group（4 网格 + 1 坐标轴）', () => {
    expect(grid.group.children).toHaveLength(5);
    for (const child of grid.group.children) expect(child).toBeInstanceOf(THREE.Group);
    expect(grid.groupCoarseHorizontal.name).toBe('grid-coarse-horizontal');
    expect(grid.groupAxes.name).toBe('grid-axes');
  });

  it('每个网格 group 内只有 1 个 LineSegments2', () => {
    for (const g of [
      grid.groupCoarseHorizontal,
      grid.groupCoarseLamp,
      grid.groupFineHorizontal,
      grid.groupFineLamp,
    ]) {
      expect(g.children).toHaveLength(1);
      expect((g.children[0] as LineSegments2).isLineSegments2).toBe(true);
    }
  });

  it('段数：粗 122/122、细 42/42', () => {
    expect(count(grid.grid.groupCoarseHorizontal)).toBe(122);
    expect(count(grid.grid.groupCoarseLamp)).toBe(122);
    expect(count(grid.grid.groupFineHorizontal)).toBe(42);
    expect(count(grid.grid.groupFineLamp)).toBe(42);
  });

  it('平面朝向：灯球面所有端点 x=0，水平面所有端点 z=0', () => {
    const xZero = (v: { x: number }): boolean => Math.abs(v.x) <= 1e-9;
    const zZero = (v: { z: number }): boolean => Math.abs(v.z) <= 1e-9;
    expect(allEndpointsSatisfy(grid.grid.groupCoarseLamp, xZero)).toBe(true);
    expect(allEndpointsSatisfy(grid.grid.groupFineLamp, xZero)).toBe(true);
    expect(allEndpointsSatisfy(grid.grid.groupCoarseHorizontal, zZero)).toBe(true);
    expect(allEndpointsSatisfy(grid.grid.groupFineHorizontal, zZero)).toBe(true);
  });

  it('网格与坐标轴统一 frustumCulled=false', () => {
    const objects = [
      grid.grid.groupCoarseHorizontal,
      grid.grid.groupCoarseLamp,
      grid.grid.groupFineHorizontal,
      grid.grid.groupFineLamp,
      ...grid.axes,
    ];
    for (const o of objects) expect(o.frustumCulled).toBe(false);
  });
});

describe('SceneGrid.apply 可见性矩阵', () => {
  const mk = (): SceneGrid => new SceneGrid();

  it('horizontal：仅水平面两组可见', () => {
    const grid = mk();
    grid.apply({ gridPlane: 'horizontal', showCoarse: true, showFine: true, showAxes: true });
    expect(grid.groupCoarseHorizontal.visible).toBe(true);
    expect(grid.groupFineHorizontal.visible).toBe(true);
    expect(grid.groupCoarseLamp.visible).toBe(false);
    expect(grid.groupFineLamp.visible).toBe(false);
    expect(grid.groupAxes.visible).toBe(true);
  });

  it('lamp：仅灯球面两组可见', () => {
    const grid = mk();
    grid.apply({ gridPlane: 'lamp', showCoarse: true, showFine: true, showAxes: true });
    expect(grid.groupCoarseHorizontal.visible).toBe(false);
    expect(grid.groupFineHorizontal.visible).toBe(false);
    expect(grid.groupCoarseLamp.visible).toBe(true);
    expect(grid.groupFineLamp.visible).toBe(true);
  });

  it('both：四个网格组都可见', () => {
    const grid = mk();
    grid.apply({ gridPlane: 'both', showCoarse: true, showFine: true, showAxes: true });
    expect(grid.groupCoarseHorizontal.visible).toBe(true);
    expect(grid.groupCoarseLamp.visible).toBe(true);
    expect(grid.groupFineHorizontal.visible).toBe(true);
    expect(grid.groupFineLamp.visible).toBe(true);
  });

  it('三个开关互相独立（粗隐、细显、轴隐）', () => {
    const grid = mk();
    grid.apply({ gridPlane: 'both', showCoarse: false, showFine: true, showAxes: false });
    expect(grid.groupCoarseHorizontal.visible).toBe(false);
    expect(grid.groupCoarseLamp.visible).toBe(false);
    expect(grid.groupFineHorizontal.visible).toBe(true);
    expect(grid.groupFineLamp.visible).toBe(true);
    expect(grid.groupAxes.visible).toBe(false);
  });
});

describe('SceneGrid 样式与分辨率', () => {
  it('setStyle 只改材质（颜色/线宽），不重建几何、不改 opacity', () => {
    const grid = new SceneGrid();
    const geomBefore = grid.grid.groupCoarseHorizontal.geometry;
    const fineGeomBefore = grid.grid.groupFineLamp.geometry;

    grid.setStyle({
      gridCoarseColor: '#ff0000',
      gridFineColor: '#00ff00',
      gridCoarseWidth: 2,
      gridFineWidth: 3,
    });

    expect(grid.grid.groupCoarseHorizontal.material.color.getHexString()).toBe('ff0000');
    expect(grid.grid.groupCoarseLamp.material.color.getHexString()).toBe('ff0000');
    expect(grid.grid.groupFineHorizontal.material.color.getHexString()).toBe('00ff00');
    expect(grid.grid.groupFineLamp.material.color.getHexString()).toBe('00ff00');
    expect(grid.grid.groupCoarseHorizontal.material.linewidth).toBe(2);
    expect(grid.grid.groupCoarseLamp.material.linewidth).toBe(2);
    expect(grid.grid.groupFineHorizontal.material.linewidth).toBe(3);
    expect(grid.grid.groupFineLamp.material.linewidth).toBe(3);

    // opacity 与几何引用都不变（证明只是材质属性变更）
    expect(grid.grid.groupCoarseHorizontal.material.opacity).toBeCloseTo(GRID_COARSE_OPACITY, 9);
    expect(grid.grid.groupFineLamp.material.opacity).toBeCloseTo(GRID_FINE_OPACITY, 9);
    expect(grid.grid.groupCoarseHorizontal.geometry).toBe(geomBefore);
    expect(grid.grid.groupFineLamp.geometry).toBe(fineGeomBefore);
  });

  it('默认颜色/线宽来自 DEFAULT_DISPLAY', () => {
    const grid = new SceneGrid();
    expect(grid.grid.groupCoarseHorizontal.material.color.getHexString()).toBe(
      DEFAULT_DISPLAY.gridCoarseColor.replace('#', ''),
    );
    expect(grid.grid.groupFineLamp.material.color.getHexString()).toBe(
      DEFAULT_DISPLAY.gridFineColor.replace('#', ''),
    );
    expect(grid.grid.groupCoarseHorizontal.material.linewidth).toBe(DEFAULT_DISPLAY.gridCoarseWidth);
    expect(grid.grid.groupFineLamp.material.linewidth).toBe(DEFAULT_DISPLAY.gridFineWidth);
  });

  it('setResolution 传播到全部 7 个材质（4 网格 + 3 轴）', () => {
    const grid = new SceneGrid();
    const materials = grid.materials();
    expect(materials).toHaveLength(7);
    grid.setResolution(1600, 900);
    for (const m of materials) {
      expect(m.resolution.x).toBeCloseTo(1600, 6);
      expect(m.resolution.y).toBeCloseTo(900, 6);
    }
  });

  it('setResolution 对非法尺寸是 no-op', () => {
    const grid = new SceneGrid();
    grid.setResolution(1600, 900);
    grid.setResolution(0, 0);
    for (const m of grid.materials()) expect(m.resolution.x).toBeCloseTo(1600, 6);
  });

  it('dispose 不抛异常且清空子节点', () => {
    const grid = new SceneGrid();
    expect(() => grid.dispose()).not.toThrow();
    expect(grid.group.children).toHaveLength(0);
  });
});
