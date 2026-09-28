/**
 * 场景网格：灯球平面（X=0，灯球始终在此平面内沿 Y 平移）/ 水平面（Z=0，传统地面）/ 两者都画。
 *
 * 设计约束：
 *  - 几何**一次构建**，切换平面/开关只改 `visible`，改颜色/线宽只改材质属性；
 *  - 网格与坐标轴统一使用 three 的 fat line（`LineSegments2` / `Line2`）：
 *    `LineBasicMaterial.linewidth` 在 Chrome/ANGLE 会被忽略，只有 fat line 才能真实控制线宽。
 *    每个「平面 × 密度」组合只建 **1 个** `LineSegments2`（整条线段列表一次实例化绘制），
 *    全开时 4 个网格对象 + 3 条坐标轴，共 7 次 draw call。
 *  - ⚠️ 透明渲染队列护栏：7 个 fat-line 材质统一 `transparent: true`，靠 renderOrder
 *    （粗网格 −3 < 细网格 −2 < 坐标轴 −1）**确定性**保证「网格在下、坐标轴在上」。
 *    若坐标轴走不透明队列，会被后画的透明网格压住。**新增任何透明物体，renderOrder 必须 > −1**。
 *  - 网格/坐标轴 `depthWrite: false`：网格是参考系，不参与深度遮挡，也因此不会与共面的灯球
 *    z-fighting；场景物体（UAV/灯球/弹道）随后绘制并压在网格之上。
 *  - `frustumCulled: false`（网格 + 坐标轴统一）：网格跨度 ±30 m、始终在视野语义内，
 *    剔除收益为零。（注：three r169 的 `LineSegmentsGeometry.setPositions()` 会自动计算
 *    boundingBox/boundingSphere，开启剔除也不会闪没——这里是显式选择，不是修 bug。）
 *
 * resolution 单一入口：`setResolution()` 是本模块**唯一**写 `LineMaterial.resolution` 的函数；
 * `mainScene.setGridResolution()` 只是转发，`export/mp4.ts` 只调用
 * `deps.onRenderResolutionChange`（由 main.ts 接到 `setGridResolution`）。
 * 交互态下 `LineSegments2.onBeforeRender` 每帧从当前 renderer 的 viewport（CSS 像素）同步
 * resolution，因此线宽单位是 CSS 像素；`setResolution` 服务于离屏导出的确定性。
 */

import * as THREE from 'three';
import { Line2 } from 'three/examples/jsm/lines/Line2.js';
import { LineGeometry } from 'three/examples/jsm/lines/LineGeometry.js';
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js';
import { LineSegments2 } from 'three/examples/jsm/lines/LineSegments2.js';
import { LineSegmentsGeometry } from 'three/examples/jsm/lines/LineSegmentsGeometry.js';

import {
  AXIS_COLORS,
  DEFAULT_DISPLAY,
  GRID_COARSE_OPACITY,
  GRID_FINE_OPACITY,
  type DisplayOptions,
} from '../config';
import { planeBasis } from '../core/geometry';

/** 网格几何参数（只管"画多大、多密、在哪个平面"；颜色/线宽归 setStyle）。 */
export interface GridOptions {
  /** 灯球平面中心（世界坐标） */
  lampPlaneCenter: THREE.Vector3;
  /** 灯球平面法线（世界坐标，本场景 (-1,0,0)） */
  lampPlaneNormal: THREE.Vector3;
  /** 粗网格半幅 [m] */
  coarseExtent: number;
  /** 粗网格间距 [m] */
  coarseStep: number;
  /** 细网格半幅 [m] */
  fineExtent: number;
  /** 细网格间距 [m] */
  fineStep: number;
}

export const DEFAULT_GRID_OPTIONS: GridOptions = {
  lampPlaneCenter: new THREE.Vector3(0, 0, 0),
  lampPlaneNormal: new THREE.Vector3(-1, 0, 0),
  coarseExtent: 30,
  coarseStep: 1,
  fineExtent: 1,
  fineStep: 0.1,
};

/** 4 个网格对象（粗/细 × 水平面/灯球面）。 */
export interface GridGroups {
  groupCoarseHorizontal: LineSegments2;
  groupCoarseLamp: LineSegments2;
  groupFineHorizontal: LineSegments2;
  groupFineLamp: LineSegments2;
}

type GridVisibility = Pick<DisplayOptions, 'gridPlane' | 'showCoarse' | 'showFine' | 'showAxes'>;
type GridStyle = Pick<
  DisplayOptions,
  'gridCoarseColor' | 'gridFineColor' | 'gridCoarseWidth' | 'gridFineWidth'
>;

/**
 * 在 u/v 张成的平面内构造方格线段，返回 `[x1,y1,z1,x2,y2,z2, ...]` 的扁平数组。
 *
 * 每一步产生「沿 v 的平行线」与「沿 u 的平行线」各一条，
 * 步数 = floor(extent / step) → 粗网格 ±30/1 = 61 条/方向（122 段），细网格 ±1/0.1 = 21 条/方向（42 段）。
 */
function planeGridPositions(
  center: THREE.Vector3,
  u: THREE.Vector3,
  v: THREE.Vector3,
  extent: number,
  step: number,
): Float32Array {
  const steps = Math.floor(extent / step);
  const positions: number[] = [];
  const push = (a: THREE.Vector3, b: THREE.Vector3): void => {
    positions.push(a.x, a.y, a.z, b.x, b.y, b.z);
  };
  const at = (a: number, b: number): THREE.Vector3 =>
    center.clone().addScaledVector(u, a).addScaledVector(v, b);

  for (let i = -steps; i <= steps; i += 1) {
    const offset = i * step;
    push(at(offset, -extent), at(offset, extent));
  }
  for (let i = -steps; i <= steps; i += 1) {
    const offset = i * step;
    push(at(-extent, offset), at(extent, offset));
  }
  return new Float32Array(positions);
}

function gridMaterial(color: string, opacity: number, linewidth: number): LineMaterial {
  return new LineMaterial({
    color,
    linewidth,
    worldUnits: false,
    dashed: false,
    transparent: true,
    opacity,
    depthTest: true,
    depthWrite: false,
  });
}

function axisMaterial(color: number): LineMaterial {
  return gridMaterial(`#${color.toString(16).padStart(6, '0')}`, 1, 2);
}

function wrapGrid(
  positions: Float32Array,
  material: LineMaterial,
  renderOrder: number,
): LineSegments2 {
  const geometry = new LineSegmentsGeometry();
  geometry.setPositions(positions);
  const object = new LineSegments2(geometry, material);
  object.renderOrder = renderOrder;
  object.frustumCulled = false;
  return object;
}

/** 纯构建：4 个网格对象（不碰 DOM、不建 renderer）。 */
export function buildGrid(opts?: Partial<GridOptions>): GridGroups {
  const o: GridOptions = { ...DEFAULT_GRID_OPTIONS, ...(opts ?? {}) };
  const origin = new THREE.Vector3(0, 0, 0);
  const axisX = new THREE.Vector3(1, 0, 0);
  const axisY = new THREE.Vector3(0, 1, 0);
  // 灯球平面：由法线推出平面内正交基（本场景 normal=(-1,0,0) → u=(0,-1,0), v=(0,0,1)）
  const [uHat, vHat] = planeBasis([o.lampPlaneNormal.x, o.lampPlaneNormal.y, o.lampPlaneNormal.z]);
  const lampU = new THREE.Vector3(uHat[0]!, uHat[1]!, uHat[2]!);
  const lampV = new THREE.Vector3(vHat[0]!, vHat[1]!, vHat[2]!);

  const coarseColor = DEFAULT_DISPLAY.gridCoarseColor;
  const fineColor = DEFAULT_DISPLAY.gridFineColor;
  const coarseWidth = DEFAULT_DISPLAY.gridCoarseWidth;
  const fineWidth = DEFAULT_DISPLAY.gridFineWidth;

  return {
    groupCoarseHorizontal: wrapGrid(
      planeGridPositions(origin, axisX, axisY, o.coarseExtent, o.coarseStep),
      gridMaterial(coarseColor, GRID_COARSE_OPACITY, coarseWidth),
      -3,
    ),
    groupCoarseLamp: wrapGrid(
      planeGridPositions(o.lampPlaneCenter, lampU, lampV, o.coarseExtent, o.coarseStep),
      gridMaterial(coarseColor, GRID_COARSE_OPACITY, coarseWidth),
      -3,
    ),
    groupFineHorizontal: wrapGrid(
      planeGridPositions(origin, axisX, axisY, o.fineExtent, o.fineStep),
      gridMaterial(fineColor, GRID_FINE_OPACITY, fineWidth),
      -2,
    ),
    groupFineLamp: wrapGrid(
      planeGridPositions(o.lampPlaneCenter, lampU, lampV, o.fineExtent, o.fineStep),
      gridMaterial(fineColor, GRID_FINE_OPACITY, fineWidth),
      -2,
    ),
  };
}

/** 纯构建：3 条三色坐标轴（X 红 / Y 绿 / Z 蓝，±range，2 CSS px）。 */
export function buildAxes(range: number): Line2[] {
  const specs: Array<{ color: number; from: [number, number, number]; to: [number, number, number] }> = [
    { color: AXIS_COLORS.x, from: [-range, 0, 0], to: [range, 0, 0] },
    { color: AXIS_COLORS.y, from: [0, -range, 0], to: [0, range, 0] },
    { color: AXIS_COLORS.z, from: [0, 0, -range], to: [0, 0, range] },
  ];
  return specs.map((spec) => {
    const geometry = new LineGeometry();
    geometry.setPositions([...spec.from, ...spec.to]);
    const line = new Line2(geometry, axisMaterial(spec.color));
    line.renderOrder = -1;
    line.frustumCulled = false;
    return line;
  });
}

/** 场景网格：5 个扁平子 group（4 网格 + 1 坐标轴），无嵌套。 */
export class SceneGrid {
  public readonly group: THREE.Group;
  public readonly grid: GridGroups;
  public readonly axes: Line2[];
  public readonly groupCoarseHorizontal: THREE.Group;
  public readonly groupCoarseLamp: THREE.Group;
  public readonly groupFineHorizontal: THREE.Group;
  public readonly groupFineLamp: THREE.Group;
  public readonly groupAxes: THREE.Group;

  constructor(opts?: Partial<GridOptions>) {
    const resolved: GridOptions = { ...DEFAULT_GRID_OPTIONS, ...(opts ?? {}) };
    this.grid = buildGrid(resolved);
    this.axes = buildAxes(resolved.coarseExtent);

    this.groupCoarseHorizontal = wrap(this.grid.groupCoarseHorizontal, 'grid-coarse-horizontal');
    this.groupCoarseLamp = wrap(this.grid.groupCoarseLamp, 'grid-coarse-lamp');
    this.groupFineHorizontal = wrap(this.grid.groupFineHorizontal, 'grid-fine-horizontal');
    this.groupFineLamp = wrap(this.grid.groupFineLamp, 'grid-fine-lamp');
    this.groupAxes = new THREE.Group();
    this.groupAxes.name = 'grid-axes';
    for (const line of this.axes) this.groupAxes.add(line);

    this.group = new THREE.Group();
    this.group.name = 'scene-grid';
    this.group.add(
      this.groupCoarseHorizontal,
      this.groupCoarseLamp,
      this.groupFineHorizontal,
      this.groupFineLamp,
      this.groupAxes,
    );
  }

  /** 应用可见性：只切 5 个 group 的 visible，不重建任何几何。 */
  apply(cfg: GridVisibility): void {
    const on = (plane: 'lamp' | 'horizontal'): boolean => cfg.gridPlane === 'both' || cfg.gridPlane === plane;
    this.groupCoarseHorizontal.visible = cfg.showCoarse && on('horizontal');
    this.groupCoarseLamp.visible = cfg.showCoarse && on('lamp');
    this.groupFineHorizontal.visible = cfg.showFine && on('horizontal');
    this.groupFineLamp.visible = cfg.showFine && on('lamp');
    this.groupAxes.visible = cfg.showAxes;
  }

  /** 应用样式：只改材质属性（颜色 / 线宽），不重建几何、不改 opacity。 */
  setStyle(style: GridStyle): void {
    for (const object of [this.grid.groupCoarseHorizontal, this.grid.groupCoarseLamp]) {
      object.material.color.set(style.gridCoarseColor);
      object.material.linewidth = style.gridCoarseWidth;
    }
    for (const object of [this.grid.groupFineHorizontal, this.grid.groupFineLamp]) {
      object.material.color.set(style.gridFineColor);
      object.material.linewidth = style.gridFineWidth;
    }
  }

  /** 唯一写 LineMaterial.resolution 的入口：网格 4 + 坐标轴 3 = 7 个材质。 */
  setResolution(width: number, height: number): void {
    if (width <= 0 || height <= 0) return;
    for (const material of this.materials()) material.resolution.set(width, height);
  }

  dispose(): void {
    for (const object of Object.values(this.grid)) {
      object.geometry.dispose();
      object.material.dispose();
    }
    for (const line of this.axes) {
      line.geometry.dispose();
      line.material.dispose();
    }
    this.group.clear();
  }

  /** 全部 7 个 LineMaterial（供 setResolution 与测试使用）。 */
  materials(): LineMaterial[] {
    return [
      this.grid.groupCoarseHorizontal.material,
      this.grid.groupCoarseLamp.material,
      this.grid.groupFineHorizontal.material,
      this.grid.groupFineLamp.material,
      ...this.axes.map((line) => line.material),
    ];
  }
}

function wrap(object: LineSegments2, name: string): THREE.Group {
  const group = new THREE.Group();
  group.name = name;
  group.add(object);
  return group;
}
