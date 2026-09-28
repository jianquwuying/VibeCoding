/**
 * M2 运动学：灯横向平移 / J2 锁定判据 / 飞镖六步积分 / 线段-点距离 / 无人机一维规划。
 *
 * 逐行对应 Python ``tools/python_ref/motion.py``；**分支顺序与表达式顺序必须一致**，
 * 否则跨语言 parity（≤1e-12）无法对齐（见 tests/golden/tools_parity.json）。
 */

import {
  CAMERA_UP_REF,
  LAMP_MOVE_END_S,
  LAMP_MOVE_START_S,
  LAMP_SPHERE_RADIUS_M,
} from '../config';
import { focalPx, poseFromPositionAndAxis, type CameraConfig, type CameraPose } from './camera';
import { cross, dot, normalize } from './geometry';

// ---------------------------------------------------------------------------
// 灯横向平移（分段线性）
// ---------------------------------------------------------------------------

/** t 时刻灯的 Y 坐标 [m]：t<1.2 → 0；1.2≤t<1.8 → 匀速；t≥1.8 → yTarget。 */
export function lampYAt(t: number, yTarget: number): number {
  if (t < LAMP_MOVE_START_S) return 0;
  if (t >= LAMP_MOVE_END_S) return yTarget;
  return (yTarget * (t - LAMP_MOVE_START_S)) / (LAMP_MOVE_END_S - LAMP_MOVE_START_S);
}

/** t 时刻灯的 Y 方向速度 [m/s]（移动区间内为常数，其余为 0）。 */
export function lampVyAt(t: number, yTarget: number): number {
  if (t < LAMP_MOVE_START_S || t >= LAMP_MOVE_END_S) return 0;
  return yTarget / (LAMP_MOVE_END_S - LAMP_MOVE_START_S);
}

/** t 时刻灯心世界坐标 = (0, y(t), 0)。 */
export function lampCenterAt(t: number, yTarget: number): number[] {
  return [0, lampYAt(t, yTarget), 0];
}

// ---------------------------------------------------------------------------
// J2 锁定判据
// ---------------------------------------------------------------------------

/**
 * 整颗 55 mm 灯球是否完整落在像面内（§2.5）。
 *
 * 球投影半径用 ``r_px = f · r / c.z``（小孔近似，与 Python 端同式）。
 */
export function isFullyVisible(
  P: readonly number[],
  d: readonly number[],
  L: readonly number[],
  camera: CameraConfig,
): boolean {
  const pose: CameraPose = poseFromPositionAndAxis(P, d, CAMERA_UP_REF);
  const c = toCameraFrameLocal(pose, L);
  if (c[2]! <= 1e-9) return false;
  const f = focalPx(camera);
  const u0 = camera.width / 2 + (f * c[0]!) / c[2]!;
  const v0 = camera.height / 2 - (f * c[1]!) / c[2]!;
  const rPx = (f * LAMP_SPHERE_RADIUS_M) / c[2]!;
  return (
    u0 - rPx >= 0 && u0 + rPx <= camera.width && v0 - rPx >= 0 && v0 + rPx <= camera.height
  );
}

/** 世界点 → 相机系 (x_c, y_c, z_c)（与 camera.ts 的 toCameraFrame 同式）。 */
function toCameraFrameLocal(pose: CameraPose, point: readonly number[]): number[] {
  const dx = point[0]! - pose.position[0]!;
  const dy = point[1]! - pose.position[1]!;
  const dz = point[2]! - pose.position[2]!;
  return [
    dx * pose.right[0]! + dy * pose.right[1]! + dz * pose.right[2]!,
    dx * pose.up[0]! + dy * pose.up[1]! + dz * pose.up[2]!,
    dx * pose.forward[0]! + dy * pose.forward[1]! + dz * pose.forward[2]!,
  ];
}

// ---------------------------------------------------------------------------
// M2 六步积分
// ---------------------------------------------------------------------------

export interface StepDartM2Input {
  P: readonly number[];
  V: readonly number[];
  L: readonly number[];
  dt: number;
  k: number;
  g: number;
  omegaMaxRad: number;
  camera: CameraConfig;
}

export interface StepDartM2Output {
  P_next: number[];
  V_next: number[];
  locked: boolean;
}

/** M2 六步积分一步（§2.3）。 */
export function stepDartM2(input: StepDartM2Input): StepDartM2Output {
  const { P, V, L, dt, k, g, omegaMaxRad, camera } = input;

  // 1) 头部朝向 = 速度方向（速度退化时用 (1,0,0) 兜底）
  const vNorm = Math.hypot(V[0]!, V[1]!, V[2]!);
  const d = vNorm >= 1e-12 ? [V[0]! / vNorm, V[1]! / vNorm, V[2]! / vNorm] : [1, 0, 0];

  // 2) 锁定判定（每帧独立）
  const locked = isFullyVisible(P, d, L, camera);

  // 3) 重力 + 线性阻力
  const aPhys = [
    -k * V[0]!,
    -k * V[1]!,
    -g - k * V[2]!,
  ];

  // 4) 无约束速度
  const vPhys = [V[0]! + aPhys[0]! * dt, V[1]! + aPhys[1]! * dt, V[2]! + aPhys[2]! * dt];

  // 5) 追踪段：保持速率、方向以 ω_max·dt 为步长转向灯心
  let vNew: number[];
  if (locked) {
    const vMag = Math.hypot(vPhys[0]!, vPhys[1]!, vPhys[2]!);
    const dAim = normalize([L[0]! - P[0]!, L[1]! - P[1]!, L[2]! - P[2]!]);
    const dThetaMax = omegaMaxRad * dt;
    const cosTheta = Math.min(1, Math.max(-1, dot(d, dAim)));
    const theta = Math.acos(cosTheta);
    let dNew: number[];
    if (theta <= dThetaMax) {
      dNew = dAim;
    } else {
      const axis = cross(d, dAim);
      const axisNorm = Math.hypot(axis[0]!, axis[1]!, axis[2]!);
      if (axisNorm < 1e-12) {
        dNew = d;
      } else {
        const kHat = [axis[0]! / axisNorm, axis[1]! / axisNorm, axis[2]! / axisNorm];
        const rot = cross(kHat, d);
        dNew = [
          d[0]! * Math.cos(dThetaMax) + rot[0]! * Math.sin(dThetaMax),
          d[1]! * Math.cos(dThetaMax) + rot[1]! * Math.sin(dThetaMax),
          d[2]! * Math.cos(dThetaMax) + rot[2]! * Math.sin(dThetaMax),
        ];
      }
    }
    vNew = [vMag * dNew[0]!, vMag * dNew[1]!, vMag * dNew[2]!];
  } else {
    vNew = vPhys;
  }

  // 6) 位置积分
  const pNext = [P[0]! + vNew[0]! * dt, P[1]! + vNew[1]! * dt, P[2]! + vNew[2]! * dt];
  return { P_next: pNext, V_next: vNew, locked };
}

// ---------------------------------------------------------------------------
// 线段-点最近距离（命中判据）
// ---------------------------------------------------------------------------

/** 线段 AB 到点 Q 的最近距离 [m]（退化线段直接取 |Q-A|）。 */
export function segmentToPointDistance(
  A: readonly number[],
  B: readonly number[],
  Q: readonly number[],
): number {
  const dx = B[0]! - A[0]!;
  const dy = B[1]! - A[1]!;
  const dz = B[2]! - A[2]!;
  const ddot = dx * dx + dy * dy + dz * dz;
  if (ddot < 1e-24) {
    return Math.hypot(Q[0]! - A[0]!, Q[1]! - A[1]!, Q[2]! - A[2]!);
  }
  const qx = Q[0]! - A[0]!;
  const qy = Q[1]! - A[1]!;
  const qz = Q[2]! - A[2]!;
  let tParam = (qx * dx + qy * dy + qz * dz) / ddot;
  tParam = Math.min(Math.max(tParam, 0), 1);
  const cx = A[0]! + tParam * dx;
  const cy = A[1]! + tParam * dy;
  const cz = A[2]! + tParam * dz;
  return Math.hypot(cx - Q[0]!, cy - Q[1]!, cz - Q[2]!);
}

// ---------------------------------------------------------------------------
// 无人机一维时间最优规划
// ---------------------------------------------------------------------------

/**
 * 一维带初速度的时间最优规划：从 y0（初速度 v0）到 y1（末速度 0），步进 dt。
 *
 * ⚠️ 分支顺序与表达式顺序必须与 Python ``uav_step`` 逐行一致。
 */
export function uavStep(
  y0: number,
  v0: number,
  y1: number,
  vMax: number,
  aMax: number,
  dt: number,
): { y: number; v: number } {
  const d = y1 - y0;
  const sgn = d > 0 ? 1 : d < 0 ? -1 : 0;
  const xt = Math.abs(d);

  // 已在目标位置
  if (xt < 1e-12) {
    if (Math.abs(v0) <= aMax * dt) return { y: y1, v: 0 };
    const sV = v0 > 0 ? 1 : -1;
    let vNew = v0 - sV * aMax * dt;
    if (vNew * sV < 0) vNew = 0;
    let yNew = y0 + v0 * dt - 0.5 * sV * aMax * dt * dt;
    const lo = Math.min(y0, y1);
    const hi = Math.max(y0, y1);
    yNew = Math.min(Math.max(yNew, lo), hi);
    return { y: yNew, v: vNew };
  }

  let v = sgn * v0;
  let x = 0;
  let tRem: number;

  // 正在远离目标：先满减速到 0
  if (v < 0) {
    const tStop = -v / aMax;
    if (tStop >= dt) {
      const xStep = v * dt + 0.5 * aMax * dt * dt;
      const vNew = v + aMax * dt;
      return { y: y0 + sgn * xStep, v: sgn * vNew };
    }
    x += v * tStop + 0.5 * aMax * tStop * tStop;
    tRem = dt - tStop;
    v = 0;
  } else {
    tRem = dt;
  }

  // 目标峰值速度（三角解，梯形时封顶 vMax）
  const vp2 = aMax * (xt - x) + (v * v) / 2;
  const vp = Math.min(Math.sqrt(Math.max(vp2, 0)), vMax);
  const tA = (vp - v) / aMax;
  const xA = (vp * vp - v * v) / (2 * aMax);

  if (tRem <= tA) {
    // 仍在加速段
    x += v * tRem + 0.5 * aMax * tRem * tRem;
    v += aMax * tRem;
    return { y: y0 + sgn * x, v: sgn * v };
  }

  tRem -= tA;
  x += xA;
  v = vp;

  const denom = Math.max(vp, 1e-12);
  const xC = Math.max(0, (xt - x - (vp * vp) / (2 * aMax)) / denom); // 匀速段距离
  if (tRem <= xC / denom) {
    x += vp * tRem;
    return { y: y0 + sgn * x, v: sgn * vp };
  }

  tRem -= xC / denom;
  x += xC;

  // 减速段：解 0.5·a·t² − vp·t + 剩余距离 = 0
  const xRem = xt - x;
  let disc = vp * vp - 2 * aMax * xRem;
  if (disc < 0) disc = 0;
  let t3 = (vp - Math.sqrt(disc)) / aMax;
  if (t3 > tRem) t3 = tRem;
  x += vp * t3 - 0.5 * aMax * t3 * t3;
  v = vp - aMax * t3;
  if (v < 0) v = 0;
  if (x > xt) {
    x = xt;
    v = 0;
  }
  return { y: y0 + sgn * x, v: sgn * v };
}
