/**
 * FPV 小窗：像平面 2D 场景（不用 3D 渲染）。
 *
 * 相机用 `OrthographicCamera(0, 640, 0, 480, -1, 1)`，与 `projectPixel` 的
 * [0,640]×[0,480]（y 向下）像素坐标完全一致，因此无需再做 (320, 240) 平移。
 * 由于 top=0 < bottom=480 会翻转投影矩阵手性、默认 FrontSide 会被背面剔除，
 * 本场景所有 Mesh 材质统一使用 THREE.DoubleSide。
 *
 * 注意：无人机 8 个投影角点按 Python 的 `idx = 4a+2b+c` 顺序存放，**不是**边界顺序，
 * 必须先取凸包再交给裁剪器，否则会得到自交多边形、交集为空。
 */

import * as THREE from 'three';

import { CAMERA_HEIGHT, CAMERA_WIDTH, IMAGE_SPACE_SEGMENTS } from '../config';
import { convexHull2D, intersectConvexPolygons, makeConvexPolygon } from '../core/geometry';
import type { SimResult } from '../core/simulate';

export interface FpvScene {
  renderer: THREE.WebGLRenderer;
  scene: THREE.Scene;
  camera: THREE.OrthographicCamera;
  updateFrame(result: SimResult, frameIndex: number): void;
  render(): void;
  setSize(width: number, height: number): void;
  dispose(): void;
}

function polygonGeometry(points: readonly number[][]): THREE.BufferGeometry {
  const shape = new THREE.Shape();
  shape.moveTo(points[0]![0]!, points[0]![1]!);
  for (let i = 1; i < points.length; i += 1) shape.lineTo(points[i]![0]!, points[i]![1]!);
  shape.closePath();
  return new THREE.ShapeGeometry(shape);
}

function outlineGeometry(points: readonly number[][]): THREE.BufferGeometry {
  const vecs = points.map((p) => new THREE.Vector3(p[0]!, p[1]!, 0));
  if (vecs.length > 0) vecs.push(vecs[0]!.clone());
  return new THREE.BufferGeometry().setFromPoints(vecs);
}

export function createFpvScene(canvas: HTMLCanvasElement): FpvScene {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setClearColor(0x020617, 1);
  renderer.setSize(canvas.clientWidth || 200, canvas.clientHeight || 150, false);

  const scene = new THREE.Scene();
  const camera = new THREE.OrthographicCamera(0, CAMERA_WIDTH, 0, CAMERA_HEIGHT, -1, 1);

  const lampMat = new THREE.MeshBasicMaterial({
    color: 0x22c55e,
    transparent: true,
    opacity: 0.45,
    side: THREE.DoubleSide,
  });
  const uavMat = new THREE.MeshBasicMaterial({
    color: 0x3b82f6,
    transparent: true,
    opacity: 0.4,
    side: THREE.DoubleSide,
  });
  const interMat = new THREE.MeshBasicMaterial({
    color: 0xef4444,
    transparent: true,
    opacity: 0.65,
    side: THREE.DoubleSide,
  });

  const lampMesh = new THREE.Mesh(new THREE.BufferGeometry(), lampMat);
  const uavMesh = new THREE.Mesh(new THREE.BufferGeometry(), uavMat);
  const interMesh = new THREE.Mesh(new THREE.BufferGeometry(), interMat);
  const lampOutline = new THREE.Line(
    outlineGeometry([[0, 0]]),
    new THREE.LineBasicMaterial({ color: 0x4ade80 }),
  );
  const uavOutline = new THREE.Line(
    outlineGeometry([[0, 0]]),
    new THREE.LineBasicMaterial({ color: 0x60a5fa }),
  );
  lampMesh.visible = false;
  uavMesh.visible = false;
  interMesh.visible = false;
  scene.add(lampMesh, uavMesh, interMesh, lampOutline, uavOutline);

  // 交集多边形按帧索引缓存，避免同一帧重复计算
  let cachedFrame = -1;
  let cachedIntersection: number[][] = [];

  const updateFrame = (result: SimResult, frameIndex: number): void => {
    const i = Math.min(Math.max(frameIndex, 0), result.n - 1);
    const lampBase = i * IMAGE_SPACE_SEGMENTS * 2;
    const uavBase = i * 8 * 2;

    const lampPts: number[][] = [];
    for (let k = 0; k < IMAGE_SPACE_SEGMENTS; k += 1) {
      const x = result.lampPx[lampBase + k * 2]!;
      const y = result.lampPx[lampBase + k * 2 + 1]!;
      if (!Number.isFinite(x) || !Number.isFinite(y)) {
        lampPts.length = 0;
        break;
      }
      lampPts.push([x, y]);
    }
    const uavRaw: number[][] = [];
    for (let k = 0; k < 8; k += 1) {
      const x = result.uavPx[uavBase + k * 2]!;
      const y = result.uavPx[uavBase + k * 2 + 1]!;
      if (!Number.isFinite(x) || !Number.isFinite(y)) {
        uavRaw.length = 0;
        break;
      }
      uavRaw.push([x, y]);
    }

    const hasLamp = lampPts.length >= 3;
    const hasUav = uavRaw.length >= 3;
    lampMesh.visible = hasLamp;
    lampOutline.visible = hasLamp;
    uavMesh.visible = hasUav;
    uavOutline.visible = hasUav;

    const lampConvex = hasLamp ? makeConvexPolygon(lampPts) : null;
    // 角点是索引顺序，必须先取凸包得到边界顺序
    const uavConvex = hasUav ? makeConvexPolygon(convexHull2D(uavRaw)) : null;

    if (lampConvex && lampConvex.points.length >= 3) {
      lampMesh.geometry.dispose();
      lampMesh.geometry = polygonGeometry(lampConvex.points);
      lampOutline.geometry.dispose();
      lampOutline.geometry = outlineGeometry(lampConvex.points);
    }
    if (uavConvex && uavConvex.points.length >= 3) {
      uavMesh.geometry.dispose();
      uavMesh.geometry = polygonGeometry(uavConvex.points);
      uavOutline.geometry.dispose();
      uavOutline.geometry = outlineGeometry(uavConvex.points);
    }

    if (cachedFrame !== i) {
      cachedFrame = i;
      cachedIntersection =
        lampConvex && uavConvex && lampConvex.points.length >= 3 && uavConvex.points.length >= 3
          ? intersectConvexPolygons(lampConvex, uavConvex).polygon
          : [];
    }
    const showInter = cachedIntersection.length >= 3;
    interMesh.visible = showInter;
    if (showInter) {
      interMesh.geometry.dispose();
      interMesh.geometry = polygonGeometry(cachedIntersection);
    }
  };

  const setSize = (width: number, height: number): void => {
    if (width <= 0 || height <= 0) return;
    renderer.setSize(Math.round(width), Math.round(height), false);
  };

  return {
    renderer,
    scene,
    camera,
    updateFrame,
    render: () => renderer.render(scene, camera),
    setSize,
    dispose: () => {
      lampMat.dispose();
      uavMat.dispose();
      interMat.dispose();
      lampMesh.geometry.dispose();
      uavMesh.geometry.dispose();
      interMesh.geometry.dispose();
      lampOutline.geometry.dispose();
      uavOutline.geometry.dispose();
      renderer.dispose();
    },
  };
}
