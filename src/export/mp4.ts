/**
 * MP4 导出：ffmpeg.wasm（H.264 + yuv420p）。
 *
 * 导出与 UI 当前播放速度**解耦**：固定 EXPORT_SPEED = 0.5（慢放 2 倍）。
 * 帧数 N = round(tLast × EXPORT_FPS / EXPORT_SPEED)，采样 t_i = tLast·i/(N−1)，
 * 保证首帧 t=0、末帧 t=tLast 全覆盖。
 */

import * as THREE from 'three';

import {
  EXPORT_BITRATE_KBPS,
  EXPORT_FPS,
  EXPORT_HEIGHT,
  EXPORT_MEMORY_BUDGET_BYTES,
  EXPORT_SPEED,
  EXPORT_WIDTH,
} from '../config';
import type { SimResult } from '../core/simulate';
import type { FpvScene } from '../render/fpvScene';
import { drawHudText } from './hud';
import { DEFAULT_HUD_LAYOUT, fpvInsetPixels, toPixels } from '../ui/hudLayout';

export interface ExportDeps {
  result: SimResult;
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  /** FPV 场景（共享 scene/camera，导出时用独立离屏渲染器出图） */
  fpvScene: FpvScene;
  uplotCanvas: HTMLCanvasElement;
  effectsCanvas: HTMLCanvasElement;
  /** 更新到指定帧（主场景 + FPV 场景 + 曲线游标） */
  updateFrame(frameIndex: number): void;
  /**
   * 离屏渲染分辨率变化通知（CSS 像素口径）。
   * Line2 坐标轴的线宽依赖 resolution：导出开始时会以 (1920,1080) 调用，
   * 调用方负责在导出结束后恢复屏幕分辨率。
   */
  onRenderResolutionChange?: (width: number, height: number) => void;
  onProgress(phase: string, frac: number): void;
}

export interface ExportResult {
  blob: Blob;
  frames: number;
  format: 'png' | 'jpeg';
  bytes: number;
  elapsedMs: number;
}

export class ExportError extends Error {
  constructor(message: string, cause?: unknown) {
    super(message, { cause });
    this.name = 'ExportError';
  }
}

/** 导出视频左上角的标题文字（网页预览与导出共用同一文案）。 */
export const EXPORT_TITLE = '无人机遮挡绿色引导灯 · Web 仿真器';

function frameCount(tLast: number): number {
  return Math.max(2, Math.round((tLast * EXPORT_FPS) / EXPORT_SPEED));
}

/** 导出采样时刻（首帧 0、末帧 tLast）。 */
export function exportTimes(tLast: number): number[] {
  const n = frameCount(tLast);
  const out: number[] = [];
  for (let i = 0; i < n; i += 1) out.push((tLast * i) / (n - 1));
  return out;
}

async function canvasBytes(
  canvas: HTMLCanvasElement,
  format: 'png' | 'jpeg',
): Promise<Uint8Array> {
  const blob = await new Promise<Blob | null>((resolve) => {
    if (format === 'png') canvas.toBlob(resolve, 'image/png');
    else canvas.toBlob(resolve, 'image/jpeg', 0.95);
  });
  if (!blob) throw new ExportError('画布编码失败（toBlob 返回空）');
  return new Uint8Array(await blob.arrayBuffer());
}

export async function exportMP4(deps: ExportDeps): Promise<ExportResult> {
  const startedAt = performance.now();
  const { result } = deps;
  const layout = DEFAULT_HUD_LAYOUT;
  const viewport = { width: EXPORT_WIDTH, height: EXPORT_HEIGHT };

  if (typeof self !== 'undefined' && 'crossOriginIsolated' in self && !self.crossOriginIsolated) {
    throw new ExportError(
      '当前页面未处于跨源隔离（crossOriginIsolated=false）。ffmpeg.wasm 需要 COOP/COEP 响应头，' +
        '请用 npm run dev / npm run preview 启动，或为静态托管补上 Cross-Origin-Embedder-Policy 与 Cross-Origin-Opener-Policy。',
    );
  }

  const times = exportTimes(result.tLast);
  const frames = times.length;
  deps.onProgress('加载编码器', 0);
  deps.onRenderResolutionChange?.(EXPORT_WIDTH, EXPORT_HEIGHT);

  // ---- 离屏渲染器 ---------------------------------------------------------
  const exportRenderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
  exportRenderer.setPixelRatio(1);
  exportRenderer.setSize(EXPORT_WIDTH, EXPORT_HEIGHT, false);
  const exportCamera = deps.camera.clone();
  exportCamera.aspect = EXPORT_WIDTH / EXPORT_HEIGHT;
  exportCamera.updateProjectionMatrix();

  const fpvRect = fpvInsetPixels(layout, viewport);
  const fpvRenderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
  fpvRenderer.setPixelRatio(1);
  fpvRenderer.setSize(Math.round(fpvRect.w), Math.round(fpvRect.h), false);

  const composite = document.createElement('canvas');
  composite.width = EXPORT_WIDTH;
  composite.height = EXPORT_HEIGHT;
  const ctx = composite.getContext('2d');
  if (!ctx) throw new ExportError('无法创建 2D 合成画布上下文');

  const curveRect = toPixels(layout.curveArea, viewport);

  /**
   * 渲染一帧到合成画布。
   *
   * 注意：导出帧数（如 224）与仿真采样帧数（如 188）不是一一对应，
   * 必须先把视频时刻映射回仿真帧索引，否则 (如 R[188]) 会越界成为 undefined。
   */
  const renderFrameToComposite = (time: number): void => {
    const simIndex = Math.min(
      Math.max(Math.round(time / result.dt), 0),
      result.n - 1,
    );
    deps.updateFrame(simIndex);
    exportCamera.position.copy(deps.camera.position);
    exportCamera.quaternion.copy(deps.camera.quaternion);
    exportRenderer.render(deps.scene, exportCamera);
    fpvRenderer.render(deps.fpvScene.scene, deps.fpvScene.camera);

    ctx.clearRect(0, 0, EXPORT_WIDTH, EXPORT_HEIGHT);
    ctx.drawImage(exportRenderer.domElement, 0, 0, EXPORT_WIDTH, EXPORT_HEIGHT);
    ctx.drawImage(
      fpvRenderer.domElement,
      fpvRect.x,
      fpvRect.y,
      fpvRect.w,
      fpvRect.h,
    );
    ctx.drawImage(deps.uplotCanvas, curveRect.x, curveRect.y, curveRect.w, curveRect.h);
    ctx.drawImage(deps.effectsCanvas, curveRect.x, curveRect.y, curveRect.w, curveRect.h);
    drawHudText(
      ctx,
      {
        rOcc: result.R[simIndex]!,
        rTh: result.rTh,
        t: time,
        tLast: result.tLast,
        speed: EXPORT_SPEED,
        imageRatio: result.imageR[simIndex]!,
        fovMarginDeg: result.fovMargin[simIndex]!,
        title: EXPORT_TITLE,
      },
      layout,
      viewport,
    );
  };

  let ffmpeg: import('@ffmpeg/ffmpeg').FFmpeg | null = null;
  const writtenFiles: string[] = [];
  let format: 'png' | 'jpeg' = 'png';

  try {
    // ---- ffmpeg.wasm（全部资源随 npm 本地提供，不走 CDN） -------------------
    const [{ FFmpeg }, coreModule, wasmModule, workerModule] = await Promise.all([
      import('@ffmpeg/ffmpeg'),
      // 走包 exports 暴露的入口（@ffmpeg/core 只导出 . 与 ./wasm）
      import('@ffmpeg/core?url'),
      import('@ffmpeg/core/wasm?url'),
      import('@ffmpeg/ffmpeg/worker?url'),
    ]);
    ffmpeg = new FFmpeg();
    ffmpeg.on('log', ({ message }) => {
      if (message.toLowerCase().includes('error')) console.warn('[ffmpeg]', message);
    });
    let encodeFrac = 0;
    ffmpeg.on('progress', ({ progress }) => {
      if (Number.isFinite(progress)) encodeFrac = Math.min(Math.max(progress, 0), 1);
      deps.onProgress('编码中', 0.85 + 0.15 * encodeFrac);
    });
    await ffmpeg.load({
      coreURL: coreModule.default,
      wasmURL: wasmModule.default,
      workerURL: workerModule.default,
    });

    // ---- 逐帧渲染 ---------------------------------------------------------
    const ext = (f: 'png' | 'jpeg'): string => (f === 'png' ? 'png' : 'jpg');
    const nameFor = (i: number, f: 'png' | 'jpeg'): string =>
      `frame_${String(i).padStart(4, '0')}.${ext(f)}`;

    let firstBytes: Uint8Array;
    {
      const idx = 0;
      renderFrameToComposite(times[idx]!);
      firstBytes = await canvasBytes(composite, 'png');
      const projected = firstBytes.length * frames;
      if (projected > EXPORT_MEMORY_BUDGET_BYTES) {
        format = 'jpeg';
        firstBytes = await canvasBytes(composite, 'jpeg');
        // eslint-disable-next-line no-console
        console.info(
          `[export] PNG 序列预计 ${(projected / 1024 / 1024).toFixed(0)} MB，超过预算，改用 JPEG(q=0.95)`,
        );
      }
      await ffmpeg.writeFile(nameFor(idx, format), firstBytes);
      writtenFiles.push(nameFor(idx, format));
      deps.onProgress('渲染帧', (0.85 * 1) / frames);
    }

    for (let i = 1; i < frames; i += 1) {
      renderFrameToComposite(times[i]!);
      const bytes = await canvasBytes(composite, format);
      const name = nameFor(i, format);
      await ffmpeg.writeFile(name, bytes);
      writtenFiles.push(name);
      deps.onProgress('渲染帧', (0.85 * (i + 1)) / frames);
    }

    // ---- 编码 -------------------------------------------------------------
    deps.onProgress('编码中', 0.85);
    const input = `frame_%04d.${ext(format)}`;
    const args = [
      '-framerate',
      String(EXPORT_FPS),
      '-i',
      input,
      '-c:v',
      'libx264',
      '-pix_fmt',
      'yuv420p',
      '-b:v',
      `${EXPORT_BITRATE_KBPS}k`,
      '-y',
      'out.mp4',
    ];
    await ffmpeg.exec(args);

    const data = await ffmpeg.readFile('out.mp4');
    const bytes =
      typeof data === 'string' ? new TextEncoder().encode(data) : new Uint8Array(data as Uint8Array);
    if (bytes.byteLength === 0) throw new ExportError('ffmpeg 输出为空，编码失败');
    deps.onProgress('完成', 1);

    return {
      blob: new Blob([bytes], { type: 'video/mp4' }),
      frames,
      format,
      bytes: bytes.byteLength,
      elapsedMs: performance.now() - startedAt,
    };
  } finally {
    if (ffmpeg) {
      for (const name of writtenFiles) {
        try {
          await ffmpeg.deleteFile(name);
        } catch {
          /* 清理失败不影响导出结果 */
        }
      }
      try {
        await ffmpeg.deleteFile('out.mp4');
      } catch {
        /* out.mp4 可能不存在 */
      }
    }
    exportRenderer.dispose();
    fpvRenderer.dispose();
    // 恢复预览到当前帧由调用方负责
  }
}

export { frameCount as exportFrameCount };
