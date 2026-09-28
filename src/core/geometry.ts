/**
 * 几何核心：射线-平面求交、平面正交基、无人机顶点/线框、阴影投影、凸包与凸多边形交集。
 *
 * 与 Python `rm_uav_occlusion/geometry.py` 一一对应；全部计算使用 float64。
 */

export const EPS = 1e-12;

export class GeometryError extends Error {}

/** 把长度 3 的序列转成 number[]（对应 Python `as_vector`）。 */
export function asVector(v: readonly number[]): number[] {
  if (v.length !== 3) throw new GeometryError(`需要 3 维向量，收到 ${v.length} 维`);
  return [v[0]!, v[1]!, v[2]!];
}

export function norm(v: readonly number[]): number {
  return Math.hypot(v[0]!, v[1]!, v[2]!);
}

export function scale(v: readonly number[], s: number): number[] {
  return [v[0]! * s, v[1]! * s, v[2]! * s];
}

export function add(a: readonly number[], b: readonly number[]): number[] {
  return [a[0]! + b[0]!, a[1]! + b[1]!, a[2]! + b[2]!];
}

export function sub(a: readonly number[], b: readonly number[]): number[] {
  return [a[0]! - b[0]!, a[1]! - b[1]!, a[2]! - b[2]!];
}

export function dot(a: readonly number[], b: readonly number[]): number {
  return a[0]! * b[0]! + a[1]! * b[1]! + a[2]! * b[2]!;
}

export function cross(a: readonly number[], b: readonly number[]): number[] {
  return [
    a[1]! * b[2]! - a[2]! * b[1]!,
    a[2]! * b[0]! - a[0]! * b[2]!,
    a[0]! * b[1]! - a[1]! * b[0]!,
  ];
}

/** 单位向量；零向量视为几何异常。 */
export function normalize(v: readonly number[]): number[] {
  const arr = asVector(v);
  const n = norm(arr);
  if (n <= EPS) throw new GeometryError('零向量无法归一化');
  return [arr[0]! / n, arr[1]! / n, arr[2]! / n];
}

/**
 * 求从 C 出发、经过 P 的射线与平面的交点。
 * 射线与平面平行（|投影分量| < EPS）或交点在相机后方（t < 0）时返回 null。
 */
export function projectPointToPlane(
  C: readonly number[],
  P: readonly number[],
  planeCenter: readonly number[],
  planeNormal: readonly number[],
): number[] | null {
  const c = asVector(C);
  const p = asVector(P);
  const pc = asVector(planeCenter);
  const n = normalize(planeNormal);
  const direction = sub(p, c);
  const denom = dot(direction, n);
  if (Math.abs(denom) < EPS) return null;
  const t = dot(sub(pc, c), n) / denom;
  if (t < 0) return null;
  return add(c, scale(direction, t));
}

/** 返回平面内一组右手正交单位基 (u, v)，与 Python `plane_basis` 完全一致。 */
export function planeBasis(planeNormal: readonly number[]): [number[], number[]] {
  const n = normalize(planeNormal);
  const ref: number[] = Math.abs(n[2]!) < 0.9 ? [0.0, 0.0, 1.0] : [1.0, 0.0, 0.0];
  const u = normalize(cross(ref, n));
  const v = normalize(cross(n, u));
  return [u, v];
}

/** 平面上三维点 → 以 planeCenter 为原点的平面局部二维坐标。 */
export function planeLocalCoords(
  point: readonly number[],
  planeCenter: readonly number[],
  u: readonly number[],
  v: readonly number[],
): [number, number] {
  const d = sub(point, planeCenter);
  return [dot(d, u), dot(d, v)];
}

/** 平面局部二维坐标 → 世界三维坐标。 */
export function planeLocalToWorld(
  coords2d: readonly number[],
  planeCenter: readonly number[],
  u: readonly number[],
  v: readonly number[],
): number[] {
  if (coords2d.length !== 2) throw new GeometryError('平面局部坐标必须是 2 维');
  const c = asVector(planeCenter);
  const uu = asVector(u);
  const vv = asVector(v);
  return [
    c[0]! + coords2d[0]! * uu[0]! + coords2d[1]! * vv[0]!,
    c[1]! + coords2d[0]! * uu[1]! + coords2d[1]! * vv[1]!,
    c[2]! + coords2d[0]! * uu[2]! + coords2d[1]! * vv[2]!,
  ];
}

/**
 * 轴对齐长方体 8 个顶点，形状 (8, 3)。
 * v3 起 `size` 三轴可以不同（长=X / 宽=Y / 高=Z），半尺寸逐轴取 `|size[i]|/2`。
 * 索引 = 4*a + 2*b + c（a/b/c 对应 x/y/z 的正负），因此 idx^1、idx^2、idx^4 一定是相邻顶点。
 */
export function uavCorners(center: readonly number[], size: readonly number[]): number[][] {
  const c = asVector(center);
  const s = asVector(size);
  const half = [Math.abs(s[0]!) / 2, Math.abs(s[1]!) / 2, Math.abs(s[2]!) / 2];
  const out: number[][] = [];
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      for (const sz of [-1, 1]) {
        out.push([c[0]! + sx * half[0]!, c[1]! + sy * half[1]!, c[2]! + sz * half[2]!]);
      }
    }
  }
  return out;
}

/** 长方体线框的 12 条棱（顶点索引对）。 */
export function uavEdges(): Array<[number, number]> {
  const edges: Array<[number, number]> = [];
  for (let i = 0; i < 8; i += 1) {
    for (const bit of [1, 2, 4]) {
      const j = i ^ bit;
      if (i < j) edges.push([i, j]);
    }
  }
  return edges;
}

/** 判断全部顶点是否整体位于相机与灯平面之间。 */
export function allVerticesBetweenCameraAndPlane(
  C: readonly number[],
  corners: readonly number[][],
  planeCenter: readonly number[],
  planeNormal: readonly number[],
  tol = 1e-9,
): boolean {
  const n = normalize(planeNormal);
  const pc = asVector(planeCenter);
  const dCam = dot(sub(C, pc), n);
  if (Math.abs(dCam) <= tol) return false;
  if (corners.length === 0) throw new GeometryError('顶点数组为空');
  for (const corner of corners) {
    const d = dot(sub(corner, pc), n);
    if (d * dCam <= 0) return false;
    if (Math.abs(d) > Math.abs(dCam) + tol) return false;
  }
  return true;
}

/** 单调链凸包（逆时针，去掉共线点）。点数不足以构成面积多边形时返回退化点集。 */
export function convexHull2D(points: readonly number[][]): number[][] {
  const pts = points
    .map((p) => [p[0]!, p[1]!] as [number, number])
    .sort((a, b) => (a[0] - b[0]) || (a[1] - b[1]));
  if (pts.length <= 2) return pts.map((p) => [p[0], p[1]]);

  const crossZ = (o: [number, number], a: [number, number], b: [number, number]) =>
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);

  const lower: Array<[number, number]> = [];
  for (const p of pts) {
    while (lower.length >= 2 && crossZ(lower[lower.length - 2]!, lower[lower.length - 1]!, p) <= 0) {
      lower.pop();
    }
    lower.push(p);
  }
  const upper: Array<[number, number]> = [];
  for (let i = pts.length - 1; i >= 0; i -= 1) {
    const p = pts[i]!;
    while (upper.length >= 2 && crossZ(upper[upper.length - 2]!, upper[upper.length - 1]!, p) <= 0) {
      upper.pop();
    }
    upper.push(p);
  }
  lower.pop();
  upper.pop();
  const hull = lower.concat(upper);
  return hull.map((p) => [p[0], p[1]]);
}

/**
 * 把无人机 8 个顶点从相机投到灯平面，得到阴影凸包。
 *
 * 返回 { polygon2D, points3D }：
 *   - polygon2D：灯平面局部坐标系下的凸包顶点（退化时点数 < 3）
 *   - points3D ：8 个投影点的世界坐标
 */
export function projectUavShadow(
  C: readonly number[],
  uavCenter: readonly number[],
  uavSize: readonly number[],
  planeCenter: readonly number[],
  planeNormal: readonly number[],
): { polygon2D: number[][]; points3D: number[][] } {
  const corners = uavCorners(uavCenter, uavSize);
  if (!allVerticesBetweenCameraAndPlane(C, corners, planeCenter, planeNormal)) {
    throw new GeometryError('无人机未整体位于相机与灯平面之间（可能已穿过灯平面或落后于相机）');
  }
  const projected: number[][] = [];
  for (const corner of corners) {
    const p = projectPointToPlane(C, corner, planeCenter, planeNormal);
    if (p === null) throw new GeometryError('射线与灯平面平行或交点位于相机后方，投影退化');
    projected.push(p);
  }
  const [u, v] = planeBasis(planeNormal);
  const points2d = projected.map((p) => planeLocalCoords(p, planeCenter, u, v) as number[]);
  const hull = convexHull2D(points2d);
  return { polygon2D: hull, points3D: projected };
}

/**
 * 灯球截面多边形（平面局部二维坐标）：过灯心、垂直于视线法线的圆离散成正 n 边形。
 *
 * 顶点约定与 shapely `Point(0,0).buffer(r, quad_segs)` 一致：
 * 从 (r, 0) 起、按**顺时针** 2π/segments 步进（quad_segs = segments / 4）。
 */
export function lampPolygon(radiusM: number, segments: number): number[][] {
  const n = Math.max(8, Math.floor(segments));
  const out: number[][] = [];
  for (let i = 0; i < n; i += 1) {
    const theta = (-2 * Math.PI * i) / n;
    out.push([radiusM * Math.cos(theta), radiusM * Math.sin(theta)]);
  }
  return out;
}

/** 多边形有向面积（鞋带公式），逆时针为正。 */
export function signedArea(polygon: readonly number[][]): number {
  let acc = 0;
  for (let i = 0; i < polygon.length; i += 1) {
    const a = polygon[i]!;
    const b = polygon[(i + 1) % polygon.length]!;
    acc += a[0]! * b[1]! - b[0]! * a[1]!;
  }
  return acc / 2;
}

export function polygonArea(polygon: readonly number[][]): number {
  return Math.abs(signedArea(polygon));
}

export interface Aabb {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
}

export function boundsOf(polygon: readonly number[][]): Aabb {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const p of polygon) {
    if (p[0]! < minX) minX = p[0]!;
    if (p[1]! < minY) minY = p[1]!;
    if (p[0]! > maxX) maxX = p[0]!;
    if (p[1]! > maxY) maxY = p[1]!;
  }
  return { minX, minY, maxX, maxY };
}

function aabbOverlap(a: Aabb, b: Aabb): boolean {
  return !(a.maxX < b.minX || b.maxX < a.minX || a.maxY < b.minY || b.maxY < a.minY);
}

/** 预展开的凸多边形：顶点（逆时针）、包围盒、包围盒四角、面积。 */
export interface ConvexPolygon {
  points: number[][];
  bounds: Aabb;
  /** 包围盒四角（逆时针）。若四角都在另一个凸多边形内，则整个多边形都在其内。 */
  aabbCorners: number[][];
  area: number;
}

/**
 * 预处理凸多边形：一次算好逆时针顶点、包围盒、包围盒四角与面积。
 *
 * 灯球截面是 1024 边形，逐帧重复做 O(n) 的 bounds/CCW 扫描会明显拖慢仿真，
 * 因此缓存这些派生量后，常见"灯球被完全遮挡"的帧只需几十次运算即可判定。
 */
export function makeConvexPolygon(polygon: readonly number[][]): ConvexPolygon {
  const points = ensureCCW(polygon);
  const bounds = boundsOf(points);
  const aabbCorners: number[][] = [
    [bounds.minX, bounds.minY],
    [bounds.maxX, bounds.minY],
    [bounds.maxX, bounds.maxY],
    [bounds.minX, bounds.maxY],
  ];
  return { points, bounds, aabbCorners, area: Math.abs(signedArea(points)) };
}

function pointInConvex(p: readonly number[], polygon: readonly number[][]): boolean {
  let hasPos = false;
  let hasNeg = false;
  for (let i = 0; i < polygon.length; i += 1) {
    const a = polygon[i]!;
    const b = polygon[(i + 1) % polygon.length]!;
    const z = (b[0]! - a[0]!) * (p[1]! - a[1]!) - (b[1]! - a[1]!) * (p[0]! - a[0]!);
    if (z > 1e-14) hasPos = true;
    else if (z < -1e-14) hasNeg = true;
    if (hasPos && hasNeg) return false;
  }
  return true;
}

function ensureCCW(polygon: readonly number[][]): number[][] {
  const copy = polygon.map((p) => [p[0]!, p[1]!]);
  return signedArea(copy) < 0 ? copy.reverse() : copy;
}

/** Sutherland–Hodgman：用凸裁剪窗口 clip 裁剪凸/任意 subject。 */
function clipByConvex(subject: readonly number[][], clip: number[][]): number[][] {
  const inside = (p: readonly number[], a: readonly number[], b: readonly number[]) =>
    (b[0]! - a[0]!) * (p[1]! - a[1]!) - (b[1]! - a[1]!) * (p[0]! - a[0]!) >= -1e-15;

  const intersect = (
    p1: readonly number[],
    p2: readonly number[],
    a: readonly number[],
    b: readonly number[],
  ): number[] => {
    const d1x = p2[0]! - p1[0]!;
    const d1y = p2[1]! - p1[1]!;
    const d2x = b[0]! - a[0]!;
    const d2y = b[1]! - a[1]!;
    const denom = d1x * d2y - d1y * d2x;
    if (Math.abs(denom) < 1e-18) return [p2[0]!, p2[1]!];
    const t = ((a[0]! - p1[0]!) * d2y - (a[1]! - p1[1]!) * d2x) / denom;
    return [p1[0]! + t * d1x, p1[1]! + t * d1y];
  };

  let output: number[][] = subject.map((p) => [p[0]!, p[1]!]);
  for (let i = 0; i < clip.length; i += 1) {
    if (output.length === 0) break;
    const a = clip[i]!;
    const b = clip[(i + 1) % clip.length]!;
    const input = output;
    output = [];
    let prev = input[input.length - 1]!;
    let prevInside = inside(prev, a, b);
    for (const cur of input) {
      const curInside = inside(cur, a, b);
      if (curInside) {
        if (!prevInside) output.push(intersect(prev, cur, a, b));
        output.push([cur[0]!, cur[1]!]);
      } else if (prevInside) {
        output.push(intersect(prev, cur, a, b));
      }
      prev = cur;
      prevInside = curInside;
    }
  }
  return output;
}

/**
 * 两个凸多边形的交集面积与交集多边形（等价于 shapely `a.intersection(b)` 的面积）。
 *
 * 本场景中两个多边形都必然是凸的（无人机阴影是 8 点凸包；灯球截面是正多边形），
 * 因此用 Sutherland–Hodgman 裁剪即可，且带 AABB 快速排除与包含快速路径。
 */
/** 两个**已预处理**的凸多边形求交集面积与交集多边形。 */
export function intersectConvexPolygons(
  a: ConvexPolygon,
  b: ConvexPolygon,
): { area: number; polygon: number[][] } {
  if (a.points.length < 3 || b.points.length < 3) return { area: 0, polygon: [] };
  if (!aabbOverlap(a.bounds, b.bounds)) return { area: 0, polygon: [] };

  // 快速接受 1：a 的包围盒四角都在 b 内 ⇒ 整个 a ⊆ b（两者都凸）。
  // 这是默认工况（机身阴影远大于灯球截面）最常见的路径，通常只需 4×O(|b|)。
  if (a.aabbCorners.every((p) => pointInConvex(p, b.points))) {
    return { area: a.area, polygon: a.points.map((p) => [p[0]!, p[1]!]) };
  }
  // 快速接受 2：b 的包围盒完全落在 a 的包围盒内且四角都在 a 内 ⇒ b ⊆ a。
  if (
    b.bounds.minX >= a.bounds.minX &&
    b.bounds.maxX <= a.bounds.maxX &&
    b.bounds.minY >= a.bounds.minY &&
    b.bounds.maxY <= a.bounds.maxY &&
    b.aabbCorners.every((p) => pointInConvex(p, a.points))
  ) {
    return { area: b.area, polygon: b.points.map((p) => [p[0]!, p[1]!]) };
  }

  // 裁剪代价 = |裁剪窗口边数| × |被裁顶点数|，因此让顶点少的一方当窗口。
  const clipped =
    a.points.length >= b.points.length
      ? clipByConvex(a.points, b.points)
      : clipByConvex(b.points, a.points);
  if (clipped.length < 3) return { area: 0, polygon: [] };
  return { area: polygonArea(clipped), polygon: clipped };
}

/** 便捷入口：直接对两个凸多边形顶点数组求交集（内部会做预处理）。 */
/**
 * 视线方向的平面基：法线 = ``normalize(L − C)``（M2：投影平面垂直于当前视线）。
 */
export function losBasis(
  C: readonly number[],
  L: readonly number[],
): [number[], number[]] {
  return planeBasis(normalize(sub(L, C)));
}
