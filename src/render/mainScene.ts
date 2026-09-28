/**
 * 主视口：全屏 WebGL 渲染器 + OrbitControls + 视角预设 + 逐帧更新。
 *
 * 逐帧更新只读取 SimResult（阴影多边形/投影线都是仿真阶段算好存下来的），
 * 动画循环里不做任何几何或仿真计算。M2 下只有"平面基"需要逐帧取
 * `losBasis(P_i, L_i)`（阴影多边形存在该平面基的局部坐标里）。
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

import { LAMP_CENTER, LAMP_NORMAL, type DisplayOptions } from '../config';
import { frustumCornerRays, type CameraPose } from '../core/camera';
import { losBasis, planeLocalToWorld } from '../core/geometry';
import { lampAt, pdAt, uavAt, type SimResult } from '../core/simulate';
import { buildScene, type SceneParts } from './buildScene';
import { SceneGrid } from './grid';

export type ViewPreset =
  | 'closeup'
  | 'overview'
  | 'top'
  | 'side'
  | 'isometric'
  | 'firstPerson';

export interface ViewPresetSpec {
  id: ViewPreset;
  label: string;
  position: [number, number, number];
  /** 基准 target（灯 Y = 0 时）；实际 target Y = target[1] + targetLampFactor · lampY */
  target: [number, number, number];
  /** 灯平移对 target Y 的传递系数（0 = 不跟随） */
  targetLampFactor: number;
}

export const VIEW_PRESETS: ViewPresetSpec[] = [
  {
    id: 'closeup',
    label: '末端特写',
    position: [-0.55, -0.42, 0.3],
    target: [-0.07, 0.0, 0.0],
    targetLampFactor: 1,
  },
  {
    id: 'overview',
    label: '全程',
    position: [-12.5, -28.0, 10.0],
    target: [-12.5, -1.43, 1.0],
    targetLampFactor: 0.5,
  },
  {
    id: 'top',
    label: '俯视',
    position: [-12.5, -1.43, 30.0],
    target: [-12.5, -1.43, 0.0],
    targetLampFactor: 1,
  },
  {
    id: 'side',
    label: '侧视',
    position: [-12.5, -30.0, 1.0],
    target: [-12.5, -1.43, 1.0],
    targetLampFactor: 0.5,
  },
  {
    id: 'isometric',
    label: '等轴测',
    position: [-35.0, -28.0, 20.0],
    target: [-12.5, -1.43, 1.0],
    targetLampFactor: 0.5,
  },
  {
    id: 'firstPerson',
    label: '第一人称',
    position: [0, 0, 0],
    target: [0, 0, 0],
    targetLampFactor: 0,
  },
];

const TRANSITION_MS = 600;
const FRUSTUM_LENGTH_M = 1.5;

export interface MainScene {
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  renderer: THREE.WebGLRenderer;
  controls: OrbitControls;
  parts: SceneParts;
  setPreset(preset: ViewPreset, animate?: boolean): void;
  getPreset(): ViewPreset;
  isTransitioning(): boolean;
  resetView(): void;
  setDisplay(display: DisplayOptions): void;
  /** 供 Line2 坐标轴更新线宽分辨率（CSS 像素口径） */
  setGridResolution(width: number, height: number): void;
  syncResult(result: SimResult): void;
  updateFrame(result: SimResult, frameIndex: number): void;
  updateTransition(now: number): void;
  resize(width: number, height: number): void;
  dispose(): void;
}

const easeInOutCubic = (x: number): number =>
  x < 0.5 ? 4 * x * x * x : 1 - Math.pow(-2 * x + 2, 3) / 2;

export function createMainScene(
  canvas: HTMLCanvasElement,
  result: SimResult,
  display: DisplayOptions,
): MainScene {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0f172a);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
  camera.up.set(0, 0, 1);

  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(window.innerWidth, window.innerHeight, false);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.object.up.set(0, 0, 1);
  controls.minDistance = 0.1;
  controls.maxDistance = 60;
  controls.zoomSpeed = 0.8;
  controls.minPolarAngle = 0;
  controls.maxPolarAngle = Math.PI - 0.05;
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.screenSpacePanning = true;
  // three 默认鼠标映射：左键 ROTATE / 右键 PAN / 滚轮 DOLLY（不重映射）

  const parts = buildScene(result);
  scene.add(parts.root);

  // 场景网格：粗/细网格 + 三色坐标轴（几何一次构建，切换只改 visible）
  const grid = new SceneGrid({
    lampPlaneCenter: new THREE.Vector3(LAMP_CENTER[0], LAMP_CENTER[1], LAMP_CENTER[2]),
    lampPlaneNormal: new THREE.Vector3(LAMP_NORMAL[0], LAMP_NORMAL[1], LAMP_NORMAL[2]),
    coarseExtent: 30,
    coarseStep: 1,
    fineExtent: 1,
    fineStep: 0.1,
  });
  scene.add(grid.group);
  grid.setResolution(window.innerWidth, window.innerHeight);

  let preset: ViewPreset = 'closeup';
  let transition: {
    fromPos: THREE.Vector3;
    toPos: THREE.Vector3;
    fromTarget: THREE.Vector3;
    toTarget: THREE.Vector3;
    start: number;
    duration: number;
  } | null = null;
  let transitioning = false;
  /** 当前帧的灯 Y（预设 target 与逐帧跟随都读它） */
  let frameLampY = result.lampY[0] ?? 0;
  /** 上一次已跟随的灯 Y：只用增量做平移，避免覆盖用户的鼠标平移 */
  let followedLampY = frameLampY;
  /** 最近一次 updateFrame 的帧索引（重算后用它恢复跟随基准，避免"假平移"） */
  let lastFrameIndex = 0;

  const presetSpec = (id: ViewPreset): ViewPresetSpec | undefined =>
    VIEW_PRESETS.find((p) => p.id === id);

  const applyPresetImmediate = (spec: ViewPresetSpec, lampY: number): void => {
    camera.position.set(spec.position[0], spec.position[1], spec.position[2]);
    controls.target.set(spec.target[0], spec.target[1] + spec.targetLampFactor * lampY, spec.target[2]);
    camera.lookAt(controls.target);
    controls.update();
  };

  applyPresetImmediate(VIEW_PRESETS.find((p) => p.id === 'closeup')!, frameLampY);

  const setPreset = (next: ViewPreset, animate = true): void => {
    preset = next;
    if (next === 'firstPerson') {
      // 第一人称由 updateFrame 每帧直接写相机位姿；期间禁止鼠标打断
      controls.enabled = false;
      transition = null;
      transitioning = false;
      return;
    }
    const spec = presetSpec(next);
    if (!spec) return;
    // target 按"当前帧的灯 Y"计算（灯已在 ±0.24 m 时点预设不再偏）
    const targetY = spec.target[1] + spec.targetLampFactor * frameLampY;
    if (!animate) {
      applyPresetImmediate(spec, frameLampY);
      controls.enabled = true;
      return;
    }
    // 过渡期间禁用 controls，避免用户鼠标打断动画
    controls.enabled = false;
    transitioning = true;
    transition = {
      fromPos: camera.position.clone(),
      toPos: new THREE.Vector3(spec.position[0], spec.position[1], spec.position[2]),
      fromTarget: controls.target.clone(),
      toTarget: new THREE.Vector3(spec.target[0], targetY, spec.target[2]),
      start: performance.now(),
      duration: TRANSITION_MS,
    };
  };

  const syncResult = (next: SimResult): void => {
    parts.reset(next);
    // 用"当前显示帧"在新结果里的灯 Y 作基准：参数重算后画面不会无谓地平移一下
    lastFrameIndex = Math.min(Math.max(lastFrameIndex, 0), next.n - 1);
    frameLampY = next.lampY[lastFrameIndex] ?? 0;
    followedLampY = frameLampY;
  };

  const poseToCamera = (pose: CameraPose): void => {
    // three 相机沿 -Z 看：基向量 (right, up, -forward)
    const fwd = new THREE.Vector3(pose.forward[0]!, pose.forward[1]!, pose.forward[2]!);
    const right = new THREE.Vector3(pose.right[0]!, pose.right[1]!, pose.right[2]!);
    const up = new THREE.Vector3(pose.up[0]!, pose.up[1]!, pose.up[2]!);
    const back = fwd.clone().negate();
    const m = new THREE.Matrix4().makeBasis(right, up, back);
    camera.quaternion.setFromRotationMatrix(m);
    camera.position.set(pose.position[0]!, pose.position[1]!, pose.position[2]!);
  };

  /**
   * 灯平移 → 视角跟随：只按**增量**平移相机与 target（保持用户鼠标平移/缩放的相对关系）。
   * 过渡期间冻结（不跟随），过渡完成后再跟随。
   */
  const followLamp = (): void => {
    const spec = presetSpec(preset);
    if (!spec || preset === 'firstPerson' || transitioning || transition) return;
    const dy = (frameLampY - followedLampY) * spec.targetLampFactor;
    if (Math.abs(dy) > 1e-12) {
      controls.target.y += dy;
      camera.position.y += dy;
      camera.lookAt(controls.target);
    }
  };

  const updateFrame = (res: SimResult, frameIndex: number): void => {
    const i = Math.min(Math.max(frameIndex, 0), res.n - 1);
    lastFrameIndex = i;
    const pose = res.poses[i]!;
    const lamp = lampAt(res, i);
    frameLampY = lamp[1]!;

    if (preset === 'firstPerson') {
      poseToCamera(pose);
    } else {
      followLamp();
    }
    followedLampY = frameLampY;

    // 相机标记：位置 + 朝向（圆锥默认沿 +Y）
    parts.cameraMarker.position.set(pose.position[0]!, pose.position[1]!, pose.position[2]!);
    const fwd = new THREE.Vector3(pose.forward[0]!, pose.forward[1]!, pose.forward[2]!);
    parts.cameraMarker.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), fwd);

    // 已飞过段
    parts.flownTrajectory.geometry.setDrawRange(0, i + 1);

    // 拖尾：最近 60 帧
    const from = Math.max(0, i - (parts.trailPositions.length / 3 - 1));
    const count = i - from + 1;
    for (let k = 0; k < count; k += 1) {
      const src = from + k;
      parts.trailPositions[k * 3] = res.Pd[src * 3]!;
      parts.trailPositions[k * 3 + 1] = res.Pd[src * 3 + 1]!;
      parts.trailPositions[k * 3 + 2] = res.Pd[src * 3 + 2]!;
    }
    (parts.trail.geometry.getAttribute('position') as THREE.BufferAttribute).needsUpdate = true;
    parts.trail.geometry.setDrawRange(0, count);

    // 灯球 / 灯位标记：灯在 Y 方向平移，逐帧跟随
    parts.lampGroup.position.set(lamp[0]!, lamp[1]!, lamp[2]!);
    parts.lampMarker.position.set(lamp[0]!, lamp[1]!, lamp[2]!);

    // 无人机：站位随灯平移（逐帧读 uavPos）
    const uav = uavAt(res, i);
    parts.uavBox.position.set(uav[0]!, uav[1]!, uav[2]!);
    parts.uavEdges.position.copy(parts.uavBox.position);

    // 阴影多边形（局部平面基逐帧由 losBasis(P_i, L_i) 给出，与仿真一致）
    const sc = res.shadowCount[i]!;
    const [lampU, lampV] = losBasis(pdAt(res, i), lamp);
    for (let k = 0; k < sc; k += 1) {
      const local = [res.shadow2D[i * 16 + k * 2]!, res.shadow2D[i * 16 + k * 2 + 1]!];
      const w = planeLocalToWorld(local, lamp, lampU, lampV);
      parts.shadowPositions[k * 3] = w[0]!;
      parts.shadowPositions[k * 3 + 1] = w[1]!;
      parts.shadowPositions[k * 3 + 2] = w[2]!;
    }
    (parts.shadowGeometry.getAttribute('position') as THREE.BufferAttribute).needsUpdate = true;
    // 索引几何的 drawRange 计的是索引数量：凸多边形三角扇 = (sc − 2) 个三角形
    parts.shadowGeometry.setDrawRange(0, sc >= 3 ? (sc - 2) * 3 : 0);

    // 顶点投影线：无人机 8 顶点 → 平面投影点
    for (let k = 0; k < 8; k += 1) {
      const cornerIndexBase = k * 3;
      const p3 = [
        res.shadow3D[i * 24 + cornerIndexBase]!,
        res.shadow3D[i * 24 + cornerIndexBase + 1]!,
        res.shadow3D[i * 24 + cornerIndexBase + 2]!,
      ];
      const uavCorner = uavCornerWorld(res, i, k);
      parts.shadowProjectionPositions[k * 6] = uavCorner[0]!;
      parts.shadowProjectionPositions[k * 6 + 1] = uavCorner[1]!;
      parts.shadowProjectionPositions[k * 6 + 2] = uavCorner[2]!;
      parts.shadowProjectionPositions[k * 6 + 3] = p3[0]!;
      parts.shadowProjectionPositions[k * 6 + 4] = p3[1]!;
      parts.shadowProjectionPositions[k * 6 + 5] = p3[2]!;
    }
    (
      parts.shadowProjectionLines.geometry.getAttribute('position') as THREE.BufferAttribute
    ).needsUpdate = true;

    // LOS：相机 → 当前灯心
    parts.losPositions[0] = pose.position[0]!;
    parts.losPositions[1] = pose.position[1]!;
    parts.losPositions[2] = pose.position[2]!;
    parts.losPositions[3] = lamp[0]!;
    parts.losPositions[4] = lamp[1]!;
    parts.losPositions[5] = lamp[2]!;
    (parts.losLine.geometry.getAttribute('position') as THREE.BufferAttribute).needsUpdate = true;
    parts.losLine.geometry.computeBoundingSphere();
    parts.losLine.computeLineDistances();

    // 光轴
    parts.axisPositions[0] = pose.position[0]!;
    parts.axisPositions[1] = pose.position[1]!;
    parts.axisPositions[2] = pose.position[2]!;
    parts.axisPositions[3] = pose.position[0]! + pose.forward[0]! * FRUSTUM_LENGTH_M;
    parts.axisPositions[4] = pose.position[1]! + pose.forward[1]! * FRUSTUM_LENGTH_M;
    parts.axisPositions[5] = pose.position[2]! + pose.forward[2]! * FRUSTUM_LENGTH_M;
    (parts.axisLine.geometry.getAttribute('position') as THREE.BufferAttribute).needsUpdate = true;
    parts.axisLine.geometry.computeBoundingSphere();

    // 视锥（4 条角射线 + 顶点环）
    const corners = frustumCornerRays(pose, res.camera, FRUSTUM_LENGTH_M);
    for (let k = 0; k < 4; k += 1) {
      const c = corners[k]!;
      parts.frustumPositions[k * 6] = pose.position[0]!;
      parts.frustumPositions[k * 6 + 1] = pose.position[1]!;
      parts.frustumPositions[k * 6 + 2] = pose.position[2]!;
      parts.frustumPositions[k * 6 + 3] = c[0]!;
      parts.frustumPositions[k * 6 + 4] = c[1]!;
      parts.frustumPositions[k * 6 + 5] = c[2]!;
    }
    for (let k = 0; k < 4; k += 1) {
      const a = corners[k]!;
      const b = corners[(k + 1) % 4]!;
      const off = (4 + k) * 6;
      parts.frustumPositions[off] = a[0]!;
      parts.frustumPositions[off + 1] = a[1]!;
      parts.frustumPositions[off + 2] = a[2]!;
      parts.frustumPositions[off + 3] = b[0]!;
      parts.frustumPositions[off + 4] = b[1]!;
      parts.frustumPositions[off + 5] = b[2]!;
    }
    (
      parts.frustumRays.geometry.getAttribute('position') as THREE.BufferAttribute
    ).needsUpdate = true;
    parts.frustumRays.geometry.computeBoundingSphere();
  };

  const updateTransition = (now: number): void => {
    if (transition) {
      const x = Math.min(1, (now - transition.start) / transition.duration);
      const e = easeInOutCubic(x);
      camera.position.lerpVectors(transition.fromPos, transition.toPos, e);
      controls.target.lerpVectors(transition.fromTarget, transition.toTarget, e);
      camera.lookAt(controls.target);
      if (x >= 1) {
        transition = null;
        transitioning = false;
        followedLampY = frameLampY;
        controls.enabled = preset !== 'firstPerson';
      }
    } else if (preset !== 'firstPerson') {
      controls.enabled = true;
    }
    if (controls.enabled) controls.update();
  };

  const setDisplay = (d: DisplayOptions): void => {
    parts.trail.visible = d.trail;
    parts.shadowMesh.visible = d.shadow;
    parts.shadowProjectionLines.visible = d.shadow;
    parts.frustumRays.visible = d.frustum;
    parts.axisLine.visible = d.frustum;
    parts.combLines.visible = d.comb;
    parts.lampTrackGroup.visible = d.lampTrack;
    // 网格只影响显示：不触发任何仿真重算
    grid.apply({
      gridPlane: d.gridPlane,
      showCoarse: d.showCoarse,
      showFine: d.showFine,
      showAxes: d.showAxes,
    });
    grid.setStyle({
      gridCoarseColor: d.gridCoarseColor,
      gridFineColor: d.gridFineColor,
      gridCoarseWidth: d.gridCoarseWidth,
      gridFineWidth: d.gridFineWidth,
    });
  };
  setDisplay(display);

  const resize = (width: number, height: number): void => {
    if (width <= 0 || height <= 0) return;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
    grid.setResolution(width, height);
  };

  return {
    scene,
    camera,
    renderer,
    controls,
    parts,
    setPreset,
    getPreset: () => preset,
    isTransitioning: () => transitioning,
    resetView: () => setPreset('closeup', true),
    setDisplay,
    setGridResolution: (width: number, height: number) => grid.setResolution(width, height),
    syncResult,
    updateFrame,
    updateTransition,
    resize,
    dispose: () => {
      controls.dispose();
      parts.dispose();
      grid.dispose();
      renderer.dispose();
    },
  };
}

/**
 * 无人机第 k 个顶点在第 i 帧的世界坐标（与 Python `uav_corners` 索引一致）。
 *
 * v3：机体三轴独立，半尺寸必须**逐轴**取（早先版本误用 `uavSizeM[0]` 当三轴半边，
 * 三轴不同尺寸时 8 条顶点投影线会整体错位）。
 */
function uavCornerWorld(res: SimResult, i: number, k: number): number[] {
  const uav = uavAt(res, i);
  const half = [res.uavSizeM[0]! / 2, res.uavSizeM[1]! / 2, res.uavSizeM[2]! / 2];
  const sx = k & 4 ? 1 : -1;
  const sy = k & 2 ? 1 : -1;
  const sz = k & 1 ? 1 : -1;
  return [uav[0]! + sx * half[0]!, uav[1]! + sy * half[1]!, uav[2]! + sz * half[2]!];
}
