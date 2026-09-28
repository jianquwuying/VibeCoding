/**
 * M2 主仿真：一次算完整个时间序列，供渲染层/曲线/时间轴/导出共用。
 *
 * 与 Python ``tools/python_ref/experiment.py::run_m2`` 逐帧对应：
 *   1. t=0：P_0 = 发射点，V_0 = v0·(cosθ0·ĥ0 + sinθ0·ẑ)，ĥ0 = 水平指向灯心
 *   2. 每帧：先按 uavStep 推进无人机（t≥1.2 s 起追踪灯的当前位置），
 *      再记录本帧状态（飞镖由 stepDartM2 推进到下一帧）
 *   3. 命中：线段 P_i → P_{i+1} 到灯心的最近距离 ≤ HIT_DISTANCE_M → 截断（命中帧不含）
 */

import {
  CAMERA_HEIGHT,
  CAMERA_UP_REF,
  CAMERA_WIDTH,
  DEFAULT_UAV_BOUNDS,
  GRAVITY_MPS2,
  HIT_DISTANCE_M,
  IMAGE_SPACE_SEGMENTS,
  LAMP_MOVE_START_S,
  LAUNCH_REL_LAMP_M,
  MAX_TURN_RATE_DPS,
  MIN_FRAMES,
  SIM_T_END_S,
  UAV_MAX_ACCEL_MPS2,
  UAV_MAX_SPEED_MPS,
  LAMP_CENTER,
  LAMP_NORMAL,
  type SimParams,
} from '../config';
import {
  fovMarginDeg,
  imageSpaceOcclusion,
  poseFromPositionAndAxis,
  projectCirclePolygon,
  projectPixel,
  type CameraConfig,
  type CameraPose,
} from './camera';
import {
  allVerticesBetweenCameraAndPlane,
  add,
  lampPolygon,
  makeConvexPolygon,
  normalize,
  planeBasis,
  scale,
  sub,
  uavCorners,
} from './geometry';
import {
  isFullyVisible,
  lampCenterAt,
  lampYAt,
  segmentToPointDistance,
  stepDartM2,
  uavStep,
} from './motion';
import { computeOcclusion, occlusionMargin, type LampFrame } from './occlusion';

export interface SimResult {
  n: number;
  dt: number;
  /** 截断后最后一帧的仿真时刻 [s] */
  tLast: number;
  t: Float64Array;
  /** R_occ；无效帧（valid=0）为 NaN */
  R: Float64Array;
  /** 飞镖位置（M2 积分），扁平 3n */
  Pd: Float64Array;
  /** 飞镖速度（M2 积分），扁平 3n */
  Vd: Float64Array;
  /** 无人机中心位置，扁平 3n（逐帧变化） */
  uavPos: Float64Array;
  /** 无人机 Y 方向速度 */
  uavVy: Float64Array;
  /** 灯心 Y 坐标 */
  lampY: Float64Array;
  /** 灯心世界坐标，扁平 3n */
  lampPos: Float64Array;
  /** 逐帧是否处于追踪段（0/1） */
  locked: Uint8Array;
  /** 逐帧几何前提是否成立（0/1） */
  valid: Uint8Array;
  /** 首次锁定时刻 [s]（-1 = 从未锁定） */
  tLock: number;
  hitLamp: boolean;
  /** 命中时刻 [s]（未命中为 null） */
  tHit: number | null;
  /** 逐帧灯状态（渲染/精度演示共用） */
  lampFrames: LampFrame[];
  /** FPV：灯球投影像素点，128×2×n（NaN 表示该帧不可见） */
  lampPx: Float64Array;
  /** FPV：无人机轮廓投影像素点，8×2×n（NaN 表示该帧不可见） */
  uavPx: Float64Array;
  /** 像面遮挡比 */
  imageR: Float64Array;
  /** 视场余量 [deg] */
  fovMargin: Float64Array;
  /** 每帧阴影凸包（平面局部二维坐标），容量 8 顶点/帧 */
  shadow2D: Float64Array;
  /** 每帧阴影凸包顶点数 */
  shadowCount: Uint8Array;
  /** 每帧 8 个投影点的世界坐标（3×8/帧） */
  shadow3D: Float64Array;
  /** 供渲染层直接取用的位姿（每帧） */
  poses: CameraPose[];
  /** 灯球几何（标量，整段不变） */
  lampRadiusM: number;
  lampAreaM2: number;
  /** 无人机几何 */
  uavSizeM: number[];
  /** 有效遮挡分析阈值（快照，供曲线/导出使用） */
  rTh: number;
  camera: CameraConfig;
  /** 遮挡余量最小值与其时刻（诊断"R 是否恒为 1"） */
  marginMin: number;
  marginMinT: number;
  /** 本次仿真的墙钟耗时 [ms] */
  elapsedMs: number;
}

export function makeCameraConfig(params: SimParams): CameraConfig {
  return { width: CAMERA_WIDTH, height: CAMERA_HEIGHT, fovHDeg: params.hfovDeg };
}

/** 机身中心安全边界：center ∈ [lo + half, hi − half]。 */
export function bodySafeBounds(sizeM: readonly number[]): { [axis: string]: [number, number] } {
  const axes: Array<'x' | 'y' | 'z'> = ['x', 'y', 'z'];
  const out: { [axis: string]: [number, number] } = {};
  for (let i = 0; i < 3; i += 1) {
    const axis = axes[i]!;
    const half = sizeM[i]! / 2;
    const [lo, hi] = DEFAULT_UAV_BOUNDS[axis];
    let cLo = lo + half;
    let cHi = hi - half;
    if (cLo > cHi) {
      const mid = 0.5 * (lo + hi);
      cLo = mid;
      cHi = mid;
    }
    out[axis] = [cLo, cHi];
  }
  return out;
}

/**
 * 灯位于 ``(0, lampY, 0)`` 时的无人机悬停站位，并裁剪到机身安全边界。
 *
 * ``站位 = L + standoff·n̂ + offset_u·û + offset_v·v̂``；对本场景（n̂=(-1,0,0)、
 * û=(0,-1,0)、v̂=(0,0,1)）等价于 ``(-standoff, lampY − offset_u, offset_v)``。
 */
export function hoverStationPositionAt(params: SimParams, lampY: number): number[] {
  const uavSizeM = [
    params.uavSizeXMm / 1000,
    params.uavSizeYMm / 1000,
    params.uavSizeZMm / 1000,
  ];
  const center: number[] = [LAMP_CENTER[0], LAMP_CENTER[1] + lampY, LAMP_CENTER[2]];
  const nHat = normalize(LAMP_NORMAL);
  const [uHat, vHat] = planeBasis(LAMP_NORMAL);
  let p = add(
    add(add(center, scale(nHat, params.standoffM)), scale(uHat, params.offsetUM)),
    scale(vHat, params.offsetVM),
  );
  const bounds = bodySafeBounds(uavSizeM);
  p = [
    Math.min(Math.max(p[0]!, bounds.x![0]), bounds.x![1]),
    Math.min(Math.max(p[1]!, bounds.y![0]), bounds.y![1]),
    Math.min(Math.max(p[2]!, bounds.z![0]), bounds.z![1]),
  ];
  return p;
}

export function runSimulation(params: SimParams): SimResult {
  const startedAt = performance.now();
  const camera = makeCameraConfig(params);
  const lampRadiusM = params.lampDiameterMm / 2000;
  const lampAreaM2 = Math.PI * lampRadiusM * lampRadiusM;
  const lampPoly2D = makeConvexPolygon(lampPolygon(lampRadiusM, params.lampSegments));
  const uavSizeM = [
    params.uavSizeXMm / 1000,
    params.uavSizeYMm / 1000,
    params.uavSizeZMm / 1000,
  ];
  const omegaMaxRad = (MAX_TURN_RATE_DPS * Math.PI) / 180;
  const g = GRAVITY_MPS2;

  const dt = params.dt;
  const nFull = Math.round(SIM_T_END_S / dt) + 1;
  const yTarget = params.lampTargetYM;

  // --- 初始状态 -----------------------------------------------------------
  let P: number[] = [...LAUNCH_REL_LAMP_M];
  const L0 = lampCenterAt(0, yTarget);
  const hHat0 = normalize([L0[0]! - P[0]!, L0[1]! - P[1]!, 0]);
  const th0 = (params.theta0Deg * Math.PI) / 180;
  let V: number[] = [
    params.v0 * (Math.cos(th0) * hHat0[0]!),
    params.v0 * (Math.cos(th0) * hHat0[1]!),
    params.v0 * Math.sin(th0),
  ];

  let uavY = -params.offsetUM;
  let uavV = 0;

  const t = new Float64Array(nFull);
  const R = new Float64Array(nFull);
  const Pd = new Float64Array(nFull * 3);
  const Vd = new Float64Array(nFull * 3);
  const uavPos = new Float64Array(nFull * 3);
  const uavVy = new Float64Array(nFull);
  const lampY = new Float64Array(nFull);
  const lampPos = new Float64Array(nFull * 3);
  const locked = new Uint8Array(nFull);
  const valid = new Uint8Array(nFull);
  const shadow2D = new Float64Array(nFull * 8 * 2);
  const shadowCount = new Uint8Array(nFull);
  const shadow3D = new Float64Array(nFull * 8 * 3);
  const lampPx = new Float64Array(IMAGE_SPACE_SEGMENTS * 2 * nFull);
  const uavPx = new Float64Array(8 * 2 * nFull);
  const imageR = new Float64Array(nFull);
  const fovMargin = new Float64Array(nFull);
  const lampFrames: LampFrame[] = [];
  const poses: CameraPose[] = [];

  let hitIndex = -1;
  let tLock = -1;
  let marginMin = Number.POSITIVE_INFINITY;
  let marginMinT = -1;

  for (let i = 0; i < nFull; i += 1) {
    const ti = i * dt;
    const L = lampCenterAt(ti, yTarget);

    // --- 无人机：t ≥ 1.2 s 起追踪灯的当前位置 ------------------------------
    if (ti >= LAMP_MOVE_START_S && i > 0) {
      const targetY = lampYAt(ti, yTarget) - params.offsetUM;
      const st = uavStep(uavY, uavV, targetY, UAV_MAX_SPEED_MPS, UAV_MAX_ACCEL_MPS2, dt);
      uavY = st.y;
      uavV = st.v;
    }
    const uavCenter = hoverStationPositionAt(params, uavY + params.offsetUM);

    // --- 记录帧 i ----------------------------------------------------------
    t[i] = ti;
    Pd[i * 3] = P[0]!;
    Pd[i * 3 + 1] = P[1]!;
    Pd[i * 3 + 2] = P[2]!;
    Vd[i * 3] = V[0]!;
    Vd[i * 3 + 1] = V[1]!;
    Vd[i * 3 + 2] = V[2]!;
    lampPos[i * 3] = L[0]!;
    lampPos[i * 3 + 1] = L[1]!;
    lampPos[i * 3 + 2] = L[2]!;
    lampY[i] = L[1]!;
    uavPos[i * 3] = uavCenter[0]!;
    uavPos[i * 3 + 1] = uavCenter[1]!;
    uavPos[i * 3 + 2] = uavCenter[2]!;
    uavY = uavY; // （保持语义清晰：uavY 即无人机中心的 Y）
    uavVy[i] = uavV;

    const losHat = normalize(sub(L, P));
    const lampFrame: LampFrame = {
      center: L,
      losHat,
      radiusM: lampRadiusM,
      areaM2: lampAreaM2,
      polygon2D: lampPoly2D,
    };
    lampFrames.push(lampFrame);

    // --- 遮挡与"相机飞到无人机之后"的有效性判据 -----------------------------
    const occ = computeOcclusion(P, uavCenter, uavSizeM, lampFrame);
    const frameValid = allVerticesBetweenCameraAndPlane(
      P,
      uavCorners(uavCenter, uavSizeM),
      L,
      losHat,
    );
    valid[i] = frameValid ? 1 : 0;
    R[i] = frameValid ? occ.Rocc : Number.NaN;
    if (occ.shadowPolygon.length >= 3) {
      const m = occlusionMargin(occ.shadowPolygon, lampRadiusM);
      if (m < marginMin) {
        marginMin = m;
        marginMinT = ti;
      }
      const count = Math.min(occ.shadowPolygon.length, 8);
      shadowCount[i] = count;
      for (let k = 0; k < count; k += 1) {
        shadow2D[i * 16 + k * 2] = occ.shadowPolygon[k]![0]!;
        shadow2D[i * 16 + k * 2 + 1] = occ.shadowPolygon[k]![1]!;
      }
      for (let k = 0; k < occ.shadowPoints3D.length; k += 1) {
        const p3 = occ.shadowPoints3D[k]!;
        shadow3D[i * 24 + k * 3] = p3[0]!;
        shadow3D[i * 24 + k * 3 + 1] = p3[1]!;
        shadow3D[i * 24 + k * 3 + 2] = p3[2]!;
      }
    }

    // --- 相机位姿与 FPV 预计算 ---------------------------------------------
    const vNorm = Math.hypot(V[0]!, V[1]!, V[2]!);
    const dHat = vNorm > 1e-12 ? [V[0]! / vNorm, V[1]! / vNorm, V[2]! / vNorm] : [1, 0, 0];
    const pose = poseFromPositionAndAxis(P, vNorm > 1e-12 ? V : dHat, CAMERA_UP_REF);
    poses.push(pose);
    fovMargin[i] = fovMarginDeg(pose, L, camera);

    const [uHat, vHat] = planeBasis(losHat);
    const lampProj = projectCirclePolygon(
      pose,
      L,
      lampRadiusM,
      uHat,
      vHat,
      camera,
      IMAGE_SPACE_SEGMENTS,
    );
    const base = i * IMAGE_SPACE_SEGMENTS * 2;
    if (lampProj === null) {
      for (let k = 0; k < IMAGE_SPACE_SEGMENTS * 2; k += 1) lampPx[base + k] = Number.NaN;
    } else {
      for (let k = 0; k < IMAGE_SPACE_SEGMENTS; k += 1) {
        lampPx[base + k * 2] = lampProj[k]![0]!;
        lampPx[base + k * 2 + 1] = lampProj[k]![1]!;
      }
    }

    const corners = uavCorners(uavCenter, uavSizeM);
    const uBase = i * 8 * 2;
    let uavVisible = true;
    for (let k = 0; k < 8; k += 1) {
      const q = projectPixel(pose, corners[k]!, camera);
      if (q === null) {
        uavVisible = false;
        break;
      }
      uavPx[uBase + k * 2] = q[0];
      uavPx[uBase + k * 2 + 1] = q[1];
    }
    if (!uavVisible) {
      for (let k = 0; k < 8 * 2; k += 1) uavPx[uBase + k] = Number.NaN;
    }

    const img = imageSpaceOcclusion(
      pose,
      { center: L, radiusM: lampRadiusM },
      uHat,
      vHat,
      uavCenter,
      uavSizeM,
      camera,
      IMAGE_SPACE_SEGMENTS,
    );
    imageR[i] = img.ratio;

    // --- 飞镖推进 + 命中判定 -----------------------------------------------
    if (i < nFull - 1) {
      const out = stepDartM2({
        P,
        V,
        L,
        dt,
        k: params.speedDecayPerS,
        g,
        omegaMaxRad,
        camera,
      });
      locked[i] = out.locked ? 1 : 0;
      if (out.locked && tLock < 0) tLock = ti;
      const dMin = segmentToPointDistance(P, out.P_next, L);
      if (hitIndex < 0 && dMin <= HIT_DISTANCE_M) hitIndex = i;
      P = out.P_next;
      V = out.V_next;
    } else {
      locked[i] = isFullyVisible(P, vNorm > 1e-12 ? [V[0]! / vNorm, V[1]! / vNorm, V[2]! / vNorm] : [1, 0, 0], L, camera)
        ? 1
        : 0;
    }
  }

  const n = hitIndex >= 0 ? Math.max(MIN_FRAMES, hitIndex) : nFull;
  const hitLamp = hitIndex >= 0;
  const slice1 = (a: Float64Array): Float64Array => a.slice(0, n);
  const slice3 = (a: Float64Array): Float64Array => a.slice(0, n * 3);
  const slicePx = (a: Float64Array, stride: number): Float64Array => a.slice(0, n * stride);

  return {
    n,
    dt,
    tLast: t[n - 1]!,
    t: slice1(t),
    R: slice1(R),
    Pd: slice3(Pd),
    Vd: slice3(Vd),
    uavPos: slice3(uavPos),
    uavVy: slice1(uavVy),
    lampY: slice1(lampY),
    lampPos: slice3(lampPos),
    locked: locked.slice(0, n),
    valid: valid.slice(0, n),
    tLock,
    hitLamp,
    tHit: hitLamp ? hitIndex * dt : null,
    lampFrames: lampFrames.slice(0, n),
    lampPx: slicePx(lampPx, IMAGE_SPACE_SEGMENTS * 2),
    uavPx: slicePx(uavPx, 16),
    imageR: slice1(imageR),
    fovMargin: slice1(fovMargin),
    shadow2D: slicePx(shadow2D, 16),
    shadowCount: shadowCount.slice(0, n),
    shadow3D: slicePx(shadow3D, 24),
    poses: poses.slice(0, n),
    lampRadiusM,
    lampAreaM2,
    uavSizeM,
    rTh: params.rTh,
    camera,
    marginMin,
    marginMinT,
    elapsedMs: performance.now() - startedAt,
  };
}

/** 时间 → 帧索引（夹取到 [0, n-1]）。 */
export function frameIndexAt(result: SimResult, t: number): number {
  const idx = Math.floor(t / result.dt);
  return Math.min(Math.max(idx, 0), result.n - 1);
}

/** 取某帧的相机位置。 */
export function pdAt(result: SimResult, i: number): number[] {
  return [result.Pd[i * 3]!, result.Pd[i * 3 + 1]!, result.Pd[i * 3 + 2]!];
}

/** 取某帧的无人机中心。 */
export function uavAt(result: SimResult, i: number): number[] {
  return [result.uavPos[i * 3]!, result.uavPos[i * 3 + 1]!, result.uavPos[i * 3 + 2]!];
}

/** 取某帧的灯心。 */
export function lampAt(result: SimResult, i: number): number[] {
  return [result.lampPos[i * 3]!, result.lampPos[i * 3 + 1]!, result.lampPos[i * 3 + 2]!];
}
