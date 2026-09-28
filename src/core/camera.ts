/**
 * 飞镖相机姿态与成像。
 *
 * 光轴**严格取弹道切线** `forward = dC/dt`；right/up 由 CAMERA_UP_REF（默认世界 +Z）构造。
 * 针孔成像：f = (W/2)/tan(HFOV/2)，主点取画面中心，
 * 像素坐标 u = cx + f·x_c/z_c、v = cy − f·y_c/z_c（**y 向下**）。
 * 几何遮挡率 R_occ 只取决于光心位置 C；相机朝向只影响视场判定与 FPV 画面。
 */

import {
  add,
  asVector,
  convexHull2D,
  cross,
  dot,
  intersectConvexPolygons,
  makeConvexPolygon,
  normalize,
  scale,
  sub,
  uavCorners,
} from './geometry';

const EPS = 1e-9;

export interface CameraConfig {
  width: number;
  height: number;
  fovHDeg: number;
}

export interface CameraPose {
  position: number[];
  forward: number[];
  right: number[];
  up: number[];
}

function fovHRad(camera: CameraConfig): number {
  return (camera.fovHDeg * Math.PI) / 180;
}

function cameraAspect(camera: CameraConfig): number {
  return camera.width / camera.height;
}

/** 由水平 FOV 与画幅比例推导的垂直 FOV [deg]。 */
function fovVDeg(camera: CameraConfig): number {
  return (2 * Math.atan(Math.tan(fovHRad(camera) / 2) / cameraAspect(camera)) * 180) / Math.PI;
}

/** 等效像素焦距 f_x = (W/2) / tan(HFOV/2)。 */
export function focalPx(camera: CameraConfig): number {
  return camera.width / 2 / Math.tan(fovHRad(camera) / 2);
}

/** 由光心与光轴构造位姿；upRef 仅用于确定滚转。 */
export function poseFromPositionAndAxis(
  position: readonly number[],
  axis: readonly number[],
  upRef: readonly number[],
): CameraPose {
  const pos = asVector(position);
  const forward = normalize(axis);
  const upHint = asVector(upRef);
  let right = cross(forward, upHint);
  if (Math.hypot(right[0]!, right[1]!, right[2]!) <= EPS) {
    right = cross(forward, [0.0, 1.0, 0.0]);
    if (Math.hypot(right[0]!, right[1]!, right[2]!) <= EPS) {
      right = cross(forward, [1.0, 0.0, 0.0]);
    }
  }
  right = normalize(right);
  const up = normalize(cross(right, forward));
  return { position: pos, forward, right, up };
}

/** 世界点 → 相机系 (x_c, y_c, z_c)。 */
export function toCameraFrame(pose: CameraPose, point: readonly number[]): number[] {
  const d = sub(point, pose.position);
  return [dot(d, pose.right), dot(d, pose.up), dot(d, pose.forward)];
}

/** 目标点相对视场边缘的余量 [deg]：>0 在视场内。 */
export function fovMarginDeg(pose: CameraPose, point: readonly number[], camera: CameraConfig): number {
  const c = toCameraFrame(pose, point);
  if (c[2]! <= EPS) return -90;
  const halfH = camera.fovHDeg / 2;
  const halfV = fovVDeg(camera) / 2;
  const az = (Math.atan2(c[0]!, c[2]!) * 180) / Math.PI;
  const el = (Math.atan2(c[1]!, c[2]!) * 180) / Math.PI;
  return Math.min(halfH - Math.abs(az), halfV - Math.abs(el));
}

/** 世界点 → 像素坐标 (u, v)；目标在相机后方（z_c <= EPS）时返回 null。 */
export function projectPixel(
  pose: CameraPose,
  point: readonly number[],
  camera: CameraConfig,
): [number, number] | null {
  const c = toCameraFrame(pose, point);
  if (c[2]! <= EPS) return null;
  const f = focalPx(camera);
  const cx = camera.width / 2;
  const cy = camera.height / 2;
  return [cx + (f * c[0]!) / c[2]!, cy - (f * c[1]!) / c[2]!];
}

/** 圆盘投影到像面（任意一点在相机后方则返回 null）。 */
export function projectCirclePolygon(
  pose: CameraPose,
  center: readonly number[],
  radiusM: number,
  basisU: readonly number[],
  basisV: readonly number[],
  camera: CameraConfig,
  nSegments: number,
): number[][] | null {
  const c = asVector(center);
  const u = asVector(basisU);
  const v = asVector(basisV);
  const out: number[][] = [];
  for (let i = 0; i < nSegments; i += 1) {
    const theta = (2 * Math.PI * i) / nSegments;
    const world = add(c, add(scale(u, radiusM * Math.cos(theta)), scale(v, radiusM * Math.sin(theta))));
    const px = projectPixel(pose, world, camera);
    if (px === null) return null;
    out.push([px[0], px[1]]);
  }
  return out;
}

/** 一组世界点投影到像面并取凸包（任一点不可见返回 null）。 */
function projectPointsHull(
  pose: CameraPose,
  points: readonly number[][],
  camera: CameraConfig,
): number[][] | null {
  const px: number[][] = [];
  for (const p of points) {
    const q = projectPixel(pose, p, camera);
    if (q === null) return null;
    px.push([q[0], q[1]]);
  }
  return convexHull2D(px);
}

/** 像面遮挡比结果（M2 只产出标量比值；多边形由 FPV 场景自行重建）。 */
export interface ImageSpaceResult {
  ratio: number;
}

/**
 * 像面遮挡比：无人机轮廓与**灯球**像面投影的交集面积 / 灯球投影面积。
 * 这是成像平面上的**补充指标**，主指标仍是灯平面上的几何遮挡率 R_occ。
 */
export function imageSpaceOcclusion(
  pose: CameraPose,
  lamp: { center: readonly number[]; radiusM: number },
  basisU: readonly number[],
  basisV: readonly number[],
  uavCenter: readonly number[],
  uavSize: readonly number[],
  camera: CameraConfig,
  nSegments: number,
): ImageSpaceResult {
  const empty: ImageSpaceResult = { ratio: 0 };
  const lampPx = projectCirclePolygon(pose, lamp.center, lamp.radiusM, basisU, basisV, camera, nSegments);
  const uavPoly = projectPointsHull(pose, uavCorners(uavCenter, uavSize), camera);
  if (lampPx === null || uavPoly === null || uavPoly.length < 3) return empty;

  const lampPoly = makeConvexPolygon(convexHull2D(lampPx));
  if (lampPoly.points.length < 3 || lampPoly.area <= 0) return empty;
  const uavConvex = makeConvexPolygon(uavPoly);
  const { area } = intersectConvexPolygons(lampPoly, uavConvex);
  const ratio = Math.min(1, Math.max(0, area / lampPoly.area));
  return { ratio };
}

/** 视锥四条角射线在 lengthM 处的端点（用于 3D 绘制）。 */
export function frustumCornerRays(
  pose: CameraPose,
  camera: CameraConfig,
  lengthM: number,
): number[][] {
  const hHalf = Math.tan(fovHRad(camera) / 2) * lengthM;
  const vHalf = hHalf / cameraAspect(camera);
  const corners: number[][] = [];
  for (const [sx, sy] of [
    [1, 1],
    [1, -1],
    [-1, -1],
    [-1, 1],
  ]) {
    corners.push(
      add(
        add(add(pose.position, scale(pose.forward, lengthM)), scale(pose.right, sx! * hHalf)),
        scale(pose.up, sy! * vHalf),
      ),
    );
  }
  return corners;
}
