/**
 * 主场景静态元素构建（弹道 / 灯球 / 灯轨道 / 无人机 / 相机标记 / 梳齿）。
 * 全部使用 Three.js 几何，1 m = 1 unit，不做缩放。
 *
 * 当前架构（M2 + v3）：
 *  - 灯是**球体**（半径 27.5 mm）；场景只有一条弹道（`SimResult.Pd` 的橙色实线）；
 *  - 灯移动轨道（±0.24 m 虚线 + 两端限位 + 原点十字 + 当前灯位标记）由
 *    `display.lampTrack` 控制显隐；
 *  - 机体是**三轴独立长方体**（`result.uavSizeM`）；
 *  - 每帧变化的量（阴影平面基 / LOS 端点 / 无人机位置 / 灯心位置）由
 *    mainScene.updateFrame 写入，本文件只负责建几何。
 */

import * as THREE from 'three';

import { LAMP_Y_RANGE_MM } from '../config';
import { lampPolygon } from '../core/geometry';
import type { SimResult } from '../core/simulate';

export interface SceneParts {
  root: THREE.Group;
  flownTrajectory: THREE.Line;
  trail: THREE.Line;
  trailPositions: Float32Array;
  cameraMarker: THREE.Mesh;
  uavBox: THREE.Mesh;
  uavEdges: THREE.LineSegments;
  /** 灯球组（球体 + 赤道点环 + 灯心标记）；position = 当前灯心 */
  lampGroup: THREE.Group;
  lampSphere: THREE.Mesh;
  lampRing: THREE.Points;
  lampCenter: THREE.Mesh;
  /** 灯移动轨道组（虚线 + 限位 + 原点十字 + 当前灯位标记） */
  lampTrackGroup: THREE.Group;
  /** 轨道上的当前灯位标记（橙色小球，每帧改 position.y） */
  lampMarker: THREE.Mesh;
  shadowMesh: THREE.Mesh;
  shadowGeometry: THREE.BufferGeometry;
  shadowPositions: Float32Array;
  shadowProjectionLines: THREE.LineSegments;
  shadowProjectionPositions: Float32Array;
  losLine: THREE.Line;
  losPositions: Float32Array;
  axisLine: THREE.Line;
  axisPositions: Float32Array;
  frustumRays: THREE.LineSegments;
  frustumPositions: Float32Array;
  combLines: THREE.LineSegments;
  reset(result: SimResult): void;
  dispose(): void;
}

const TRAIL_LENGTH = 60;
/** 灯轨道半长 [m]（物理限位 ±240 mm）。 */
export const LAMP_TRACK_HALF_M =
  Math.max(Math.abs(LAMP_Y_RANGE_MM[0]), Math.abs(LAMP_Y_RANGE_MM[1])) / 1000;

function makeLineMaterial(color: number, opacity = 1): THREE.LineBasicMaterial {
  return new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity });
}

function dashedMaterial(color: number): THREE.LineDashedMaterial {
  return new THREE.LineDashedMaterial({ color, dashSize: 0.25, gapSize: 0.15 });
}

/** 灯球赤道：局部坐标下 XZ 平面（法线 = 灯平移轴 Y）的 40 点圆环。 */
function equatorPoints(radiusM: number, segments: number): THREE.Vector3[] {
  const out: THREE.Vector3[] = [];
  for (const p of lampPolygon(radiusM, segments)) out.push(new THREE.Vector3(p[0]!, 0, p[1]!));
  return out;
}

/** 沿弹道每 1.2 m 弧长放一条短切线，构造"光轴梳齿"。 */
function buildCombPoints(path: readonly number[][], spacingM: number, tickM: number): number[] {
  const positions: number[] = [];
  if (path.length < 2) return positions;
  let travelled = 0;
  let nextAt = 0;
  for (let i = 1; i < path.length; i += 1) {
    const a = path[i - 1]!;
    const b = path[i]!;
    const segLen = Math.hypot(b[0]! - a[0]!, b[1]! - a[1]!, b[2]! - a[2]!);
    if (segLen <= 0) continue;
    while (travelled + segLen >= nextAt) {
      const w = segLen > 0 ? (nextAt - travelled) / segLen : 0;
      const p = [
        a[0]! + (b[0]! - a[0]!) * w,
        a[1]! + (b[1]! - a[1]!) * w,
        a[2]! + (b[2]! - a[2]!) * w,
      ];
      const dir = [
        (b[0]! - a[0]!) / segLen,
        (b[1]! - a[1]!) / segLen,
        (b[2]! - a[2]!) / segLen,
      ];
      positions.push(
        p[0]! - (dir[0]! * tickM) / 2,
        p[1]! - (dir[1]! * tickM) / 2,
        p[2]! - (dir[2]! * tickM) / 2,
        p[0]! + (dir[0]! * tickM) / 2,
        p[1]! + (dir[1]! * tickM) / 2,
        p[2]! + (dir[2]! * tickM) / 2,
      );
      nextAt += spacingM;
    }
    travelled += segLen;
  }
  return positions;
}

/** M2 弹道折线（逐帧 Pd）。 */
function trajectoryPoints(result: SimResult): number[][] {
  const out: number[][] = [];
  for (let i = 0; i < result.n; i += 1) {
    out.push([result.Pd[i * 3]!, result.Pd[i * 3 + 1]!, result.Pd[i * 3 + 2]!]);
  }
  return out;
}

export function buildScene(result: SimResult): SceneParts {
  const root = new THREE.Group();
  root.name = 'scene-root';

  // ---- 弹道（M2 积分轨迹，橙色实线；逐帧用 drawRange 揭示） ----------------
  const flownPositions = new Float32Array(result.n * 3);
  for (let i = 0; i < result.n; i += 1) {
    flownPositions[i * 3] = result.Pd[i * 3]!;
    flownPositions[i * 3 + 1] = result.Pd[i * 3 + 1]!;
    flownPositions[i * 3 + 2] = result.Pd[i * 3 + 2]!;
  }
  const flownGeometry = new THREE.BufferGeometry();
  flownGeometry.setAttribute('position', new THREE.BufferAttribute(flownPositions, 3));
  const flownTrajectory = new THREE.Line(flownGeometry, makeLineMaterial(0xf97316));
  flownTrajectory.geometry.setDrawRange(0, 1);
  root.add(flownTrajectory);

  // ---- 拖尾（固定 60 帧循环缓冲 + 顶点色渐变） ------------------------------
  const trailPositions = new Float32Array(TRAIL_LENGTH * 3);
  const trailColors = new Float32Array(TRAIL_LENGTH * 3);
  for (let i = 0; i < TRAIL_LENGTH; i += 1) {
    const f = i / (TRAIL_LENGTH - 1);
    trailColors[i * 3] = 1.0 * f;
    trailColors[i * 3 + 1] = 0.65 * f;
    trailColors[i * 3 + 2] = 0.15 * f;
  }
  const trailGeometry = new THREE.BufferGeometry();
  trailGeometry.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
  trailGeometry.setAttribute('color', new THREE.BufferAttribute(trailColors, 3));
  trailGeometry.setDrawRange(0, 0);
  const trail = new THREE.Line(
    trailGeometry,
    new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.9 }),
  );
  root.add(trail);

  // ---- 飞镖相机标记 --------------------------------------------------------
  const cameraMarker = new THREE.Mesh(
    new THREE.ConeGeometry(0.03, 0.09, 16),
    new THREE.MeshBasicMaterial({ color: 0xef4444 }),
  );
  cameraMarker.name = 'dart-camera';
  root.add(cameraMarker);

  // ---- 无人机（长方体 + 蓝色线框；v3 三轴独立 → BoxGeometry(sx, sy, sz)） ----
  const [sx, sy, sz] = [result.uavSizeM[0]!, result.uavSizeM[1]!, result.uavSizeM[2]!];
  const uavBox = new THREE.Mesh(
    new THREE.BoxGeometry(sx, sy, sz),
    new THREE.MeshBasicMaterial({ color: 0x1e40af, transparent: true, opacity: 0.25 }),
  );
  const uavEdges = new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.BoxGeometry(sx, sy, sz)),
    makeLineMaterial(0x3b82f6),
  );
  uavBox.position.set(result.uavPos[0]!, result.uavPos[1]!, result.uavPos[2]!);
  uavEdges.position.copy(uavBox.position);
  root.add(uavBox, uavEdges);

  // ---- 灯球（球体 + 赤道点环 + 灯心标记） -----------------------------------
  const lampRadius = result.lampRadiusM;
  const lampGroup = new THREE.Group();
  lampGroup.name = 'lamp-sphere';
  const lampSphere = new THREE.Mesh(
    new THREE.SphereGeometry(lampRadius, 32, 24),
    new THREE.MeshBasicMaterial({ color: 0x22c55e, transparent: true, opacity: 0.45 }),
  );
  const lampRing = new THREE.Points(
    new THREE.BufferGeometry().setFromPoints(equatorPoints(lampRadius, 40)),
    new THREE.PointsMaterial({ color: 0x86efac, size: 0.01, sizeAttenuation: false }),
  );
  const lampCenter = new THREE.Mesh(
    new THREE.SphereGeometry(lampRadius * 0.22, 12, 8),
    new THREE.MeshBasicMaterial({ color: 0xbbf7d0 }),
  );
  lampGroup.add(lampSphere, lampRing, lampCenter);
  lampGroup.position.set(result.lampPos[0]!, result.lampPos[1]!, result.lampPos[2]!);
  root.add(lampGroup);

  // ---- 灯移动轨道（±240 mm 虚线 + 限位球 + 原点十字 + 当前灯位标记） ---------
  const lampTrackGroup = new THREE.Group();
  lampTrackGroup.name = 'lamp-track';
  const trackHalf = LAMP_TRACK_HALF_M;
  const trackLine = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, -trackHalf, 0),
      new THREE.Vector3(0, trackHalf, 0),
    ]),
    dashedMaterial(0x94a3b8),
  );
  trackLine.computeLineDistances();
  lampTrackGroup.add(trackLine);
  for (const sign of [-1, 1]) {
    const stop = new THREE.Mesh(
      new THREE.SphereGeometry(0.012, 12, 8),
      new THREE.MeshBasicMaterial({ color: 0x94a3b8 }),
    );
    stop.position.set(0, sign * trackHalf, 0);
    lampTrackGroup.add(stop);
  }
  // 原点十字（X/Y 两条短线段，标出灯 Y=0 的基准位）
  const crossArm = lampRadius * 3;
  const cross = new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(-crossArm, 0, 0),
      new THREE.Vector3(crossArm, 0, 0),
      new THREE.Vector3(0, -crossArm, 0),
      new THREE.Vector3(0, crossArm, 0),
    ]),
    makeLineMaterial(0x86efac, 0.8),
  );
  lampTrackGroup.add(cross);
  const lampMarker = new THREE.Mesh(
    new THREE.SphereGeometry(Math.max(lampRadius * 1.6, 0.02), 16, 12),
    new THREE.MeshBasicMaterial({ color: 0xf59e0b }),
  );
  lampTrackGroup.add(lampMarker);
  root.add(lampTrackGroup);

  // ---- 阴影多边形（每帧更新顶点，容量 8） -----------------------------------
  const shadowPositions = new Float32Array(8 * 3);
  const shadowGeometry = new THREE.BufferGeometry();
  shadowGeometry.setAttribute('position', new THREE.BufferAttribute(shadowPositions, 3));
  shadowGeometry.setIndex([0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5, 0, 5, 6, 0, 6, 7]);
  const shadowMesh = new THREE.Mesh(
    shadowGeometry,
    new THREE.MeshBasicMaterial({
      color: 0xef4444,
      transparent: true,
      opacity: 0.5,
      side: THREE.DoubleSide,
    }),
  );
  shadowMesh.frustumCulled = false;
  root.add(shadowMesh);

  const shadowProjectionPositions = new Float32Array(8 * 2 * 3);
  const shadowProjectionGeometry = new THREE.BufferGeometry();
  shadowProjectionGeometry.setAttribute(
    'position',
    new THREE.BufferAttribute(shadowProjectionPositions, 3),
  );
  const shadowProjectionLines = new THREE.LineSegments(
    shadowProjectionGeometry,
    makeLineMaterial(0x94a3b8, 0.55),
  );
  shadowProjectionLines.frustumCulled = false;
  root.add(shadowProjectionLines);

  // ---- LOS / 光轴 / 视锥 ---------------------------------------------------
  const losPositions = new Float32Array(6);
  const losGeometry = new THREE.BufferGeometry();
  losGeometry.setAttribute('position', new THREE.BufferAttribute(losPositions, 3));
  const losLine = new THREE.Line(losGeometry, dashedMaterial(0xf59e0b));
  losLine.frustumCulled = false;
  root.add(losLine);

  const axisPositions = new Float32Array(6);
  const axisGeometry = new THREE.BufferGeometry();
  axisGeometry.setAttribute('position', new THREE.BufferAttribute(axisPositions, 3));
  const axisLine = new THREE.Line(axisGeometry, makeLineMaterial(0xef4444));
  axisLine.frustumCulled = false;
  root.add(axisLine);

  const frustumPositions = new Float32Array(8 * 2 * 3);
  const frustumGeometry = new THREE.BufferGeometry();
  frustumGeometry.setAttribute('position', new THREE.BufferAttribute(frustumPositions, 3));
  const frustumRays = new THREE.LineSegments(frustumGeometry, makeLineMaterial(0xf87171, 0.85));
  frustumRays.frustumCulled = false;
  root.add(frustumRays);

  // ---- 梳齿（沿 M2 弹道每 1.2 m 弧长一条切线） ------------------------------
  const combPositions = buildCombPoints(trajectoryPoints(result), 1.2, 0.08);
  const combGeometry = new THREE.BufferGeometry();
  combGeometry.setAttribute('position', new THREE.Float32BufferAttribute(combPositions, 3));
  const combLines = new THREE.LineSegments(combGeometry, makeLineMaterial(0x64748b, 0.7));
  root.add(combLines);

  const parts: SceneParts = {
    root,
    flownTrajectory,
    trail,
    trailPositions,
    cameraMarker,
    uavBox,
    uavEdges,
    lampGroup,
    lampSphere,
    lampRing,
    lampCenter,
    lampTrackGroup,
    lampMarker,
    shadowMesh,
    shadowGeometry,
    shadowPositions,
    shadowProjectionLines,
    shadowProjectionPositions,
    losLine,
    losPositions,
    axisLine,
    axisPositions,
    frustumRays,
    frustumPositions,
    combLines,
    reset(next: SimResult) {
      resetForResult(parts, next);
    },
    dispose() {
      root.traverse((obj) => {
        const anyObj = obj as unknown as {
          geometry?: THREE.BufferGeometry;
          material?: THREE.Material | THREE.Material[];
        };
        anyObj.geometry?.dispose();
        const mat = anyObj.material;
        if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
        else mat?.dispose();
      });
    },
  };
  return parts;
}

/** 参数改变后重建与结果规模相关的几何（弹道、梳齿、灯球半径、灯标记、机体尺寸）。 */
function resetForResult(parts: SceneParts, result: SimResult): void {
  // 已飞过段（M2 轨迹全程 = Pd）
  const flownPositions = new Float32Array(result.n * 3);
  for (let i = 0; i < result.n; i += 1) {
    flownPositions[i * 3] = result.Pd[i * 3]!;
    flownPositions[i * 3 + 1] = result.Pd[i * 3 + 1]!;
    flownPositions[i * 3 + 2] = result.Pd[i * 3 + 2]!;
  }
  parts.flownTrajectory.geometry.dispose();
  const flownGeometry = new THREE.BufferGeometry();
  flownGeometry.setAttribute('position', new THREE.BufferAttribute(flownPositions, 3));
  parts.flownTrajectory.geometry = flownGeometry;
  parts.flownTrajectory.geometry.setDrawRange(0, 1);

  // 梳齿
  const combPositions = buildCombPoints(trajectoryPoints(result), 1.2, 0.08);
  parts.combLines.geometry.dispose();
  const combGeometry = new THREE.BufferGeometry();
  combGeometry.setAttribute('position', new THREE.Float32BufferAttribute(combPositions, 3));
  parts.combLines.geometry = combGeometry;

  // 灯球半径（灯直径参数可能变化）→ 重建几何
  const lampRadius = result.lampRadiusM;
  parts.lampSphere.geometry.dispose();
  parts.lampSphere.geometry = new THREE.SphereGeometry(lampRadius, 32, 24);
  parts.lampRing.geometry.dispose();
  parts.lampRing.geometry = new THREE.BufferGeometry().setFromPoints(equatorPoints(lampRadius, 40));
  parts.lampCenter.geometry.dispose();
  parts.lampCenter.geometry = new THREE.SphereGeometry(lampRadius * 0.22, 12, 8);
  parts.lampMarker.geometry.dispose();
  parts.lampMarker.geometry = new THREE.SphereGeometry(Math.max(lampRadius * 1.6, 0.02), 16, 12);
  parts.lampGroup.position.set(result.lampPos[0]!, result.lampPos[1]!, result.lampPos[2]!);
  parts.lampMarker.position.set(result.lampPos[0]!, result.lampPos[1]!, result.lampPos[2]!);

  // 无人机尺寸
  const [sx, sy, sz] = [result.uavSizeM[0]!, result.uavSizeM[1]!, result.uavSizeM[2]!];
  parts.uavBox.geometry.dispose();
  parts.uavBox.geometry = new THREE.BoxGeometry(sx, sy, sz);
  parts.uavEdges.geometry.dispose();
  parts.uavEdges.geometry = new THREE.EdgesGeometry(new THREE.BoxGeometry(sx, sy, sz));
  parts.uavBox.position.set(result.uavPos[0]!, result.uavPos[1]!, result.uavPos[2]!);
  parts.uavEdges.position.copy(parts.uavBox.position);
}
