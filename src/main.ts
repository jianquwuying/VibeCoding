/**
 * 入口：串联仿真核心 / 3D 主视口 / FPV 小窗 / 三层 HUD / 参数面板 / MP4 导出。
 *
 * 约定：
 *  - 参数改变才重算仿真（每动画帧最多一次），动画循环只读 SimResult；
 *  - 主视口与 FPV 小窗共享同一份 SimResult，不重复做几何运算；
 *  - 布局像素全部由 ui/hudLayout 推导，渲染代码不出现硬编码像素。
 */

import './style.css';

import {
  DEFAULT_HUD_LAYOUT,
  fpvInsetPixels,
  toPixels,
  type Viewport,
} from './ui/hudLayout';
import {
  DEFAULT_DISPLAY,
  DEFAULT_PARAMS,
  EXPORT_FPS,
  EXPORT_SPEED,
  LAMP_SEGMENTS_DEFAULT,
  PLAYBACK_SPEED_DEFAULT,
  type DisplayOptions,
  type SimParams,
} from './config';
import { lampPolygon, makeConvexPolygon } from './core/geometry';
import { computeOcclusion, type LampFrame } from './core/occlusion';
import { frameIndexAt, runSimulation, type SimResult } from './core/simulate';
import { ExportError, exportMP4 } from './export/mp4';
import { createFpvScene } from './render/fpvScene';
import { createMainScene, VIEW_PRESETS, type ViewPreset } from './render/mainScene';
import { createCurveView } from './ui/curve';
import { createFpvOverlay } from './ui/fpvOverlay';
import { clearPersisted, createPanel, loadPersisted, savePersisted } from './ui/panel';
import { createTimelineView } from './ui/timeline';

if (new URLSearchParams(window.location.search).get('selftest') === 'ffmpeg') {
  void import('./selftest').then((m) => m.runFfmpegSelftest());
}

function need<T extends HTMLElement>(id: string): T {
  const el = document.getElementById(id);
  if (!el) throw new Error(`缺少 DOM 节点 #${id}`);
  return el as T;
}

const mainCanvas = need<HTMLCanvasElement>('main-canvas');
const curveHost = need<HTMLDivElement>('curve-host');
const uplotHost = need<HTMLDivElement>('curve-uplot');
const effectsCanvas = need<HTMLCanvasElement>('curve-effects');
const fpvHost = need<HTMLDivElement>('fpv-host');
const fpvCanvas = need<HTMLCanvasElement>('fpv-canvas');
const viewBar = need<HTMLDivElement>('view-bar');
const exportOverlay = need<HTMLDivElement>('export-overlay');
const exportStage = need<HTMLParagraphElement>('export-stage');
const exportPercent = need<HTMLParagraphElement>('export-percent');
const exportFill = need<HTMLDivElement>('export-progress-fill');
const toastHost = need<HTMLDivElement>('toast-host');
const noHitWarning = need<HTMLDivElement>('no-hit-warning');

function showModal(opts: {
  title: string;
  body: string;
  confirmText?: string;
  cancelText?: string;
}): Promise<boolean> {
  return new Promise((resolve) => {
    const card = document.createElement('div');
    card.className = 'modal-card';
    const title = document.createElement('h3');
    title.textContent = opts.title;
    const body = document.createElement('p');
    body.textContent = opts.body;
    const actions = document.createElement('div');
    actions.className = 'modal-actions';
    const ok = document.createElement('button');
    ok.className = 'primary';
    ok.textContent = opts.confirmText ?? '确定';
    const cancel = document.createElement('button');
    cancel.textContent = opts.cancelText ?? '取消';
    const close = (value: boolean): void => {
      toastHost.replaceChildren();
      toastHost.classList.remove('visible');
      resolve(value);
    };
    ok.addEventListener('click', () => close(true));
    cancel.addEventListener('click', () => close(false));
    actions.append(cancel, ok);
    card.append(title, body, actions);
    toastHost.replaceChildren(card);
    toastHost.classList.add('visible');
  });
}

const persisted = loadPersisted();
let params: SimParams = { ...persisted.params };
let display: DisplayOptions = { ...persisted.display };
let speed = persisted.speed > 0 ? persisted.speed : PLAYBACK_SPEED_DEFAULT;
let result: SimResult = runSimulation(params);
let playing = true;
let playbackTime = 0;
let exporting = false;
let lastNow = performance.now();
let recomputeQueued = false;

const viewport = (): Viewport => ({ width: window.innerWidth, height: window.innerHeight });

const mainScene = createMainScene(mainCanvas, result, display);
const fpvScene = createFpvScene(fpvCanvas);
const curveView = createCurveView(curveHost, uplotHost, effectsCanvas);
const fpvOverlay = createFpvOverlay(
  fpvHost,
  fpvCanvas,
  need<HTMLSpanElement>('fpv-roc'),
  need<HTMLSpanElement>('fpv-image'),
  need<HTMLSpanElement>('fpv-margin'),
);

const timeline = createTimelineView(
  {
    bar: need<HTMLDivElement>('timeline-bar'),
    controls: need<HTMLDivElement>('timeline-controls'),
    trackHost: need<HTMLDivElement>('timeline-track-host'),
    track: need<HTMLDivElement>('timeline-track'),
    progress: need<HTMLDivElement>('timeline-progress'),
    thumb: need<HTMLDivElement>('timeline-thumb'),
    playButton: need<HTMLButtonElement>('btn-play'),
    resetButton: need<HTMLButtonElement>('btn-reset'),
    speedSlider: need<HTMLInputElement>('speed-slider'),
    speedValue: need<HTMLSpanElement>('speed-value'),
    speedHint: need<HTMLSpanElement>('speed-hint'),
    hudRocc: need<HTMLSpanElement>('hud-roc'),
    hudTime: need<HTMLSpanElement>('hud-time'),
    hudSpeed: need<HTMLSpanElement>('hud-speed'),
  },
  {
    onPlayToggle: () => {
      if (exporting) return;
      if (!playing && playbackTime >= result.tLast - 1e-9) playbackTime = 0;
      playing = !playing;
      timeline.setPlaying(playing);
    },
    onReset: () => {
      if (exporting) return;
      playbackTime = 0;
      playing = false;
      timeline.setPlaying(false);
    },
    onSeekFraction: (frac) => {
      if (exporting) return;
      playbackTime = frac * result.tLast;
      playing = false;
      timeline.setPlaying(false);
    },
    onSpeedChange: (value) => {
      speed = value;
      schedulePersist();
    },
  },
  speed,
);

const panel = createPanel(
  need<HTMLDivElement>('panel-host'),
  { params, display, speed },
  {
    onParamsChange: (next) => {
      params = next;
      scheduleRecompute();
      schedulePersist();
    },
    onDisplayChange: (next) => {
      display = next;
      mainScene.setDisplay(display);
      curveView.setVisible(display.curve);
      fpvOverlay.setVisible(display.fpv);
      schedulePersist();
    },
    onPrecisionDemo: (segments) => {
      console.info(`[lamp] 精度演示：LampSegments = ${segments}`);
    },
    onResetAll: () => {
      params = { ...DEFAULT_PARAMS };
      display = { ...DEFAULT_DISPLAY };
      speed = PLAYBACK_SPEED_DEFAULT;
      timeline.setSpeed(speed);
      clearPersisted();
      scheduleRecompute();
    },
  },
);

const presetButtons = new Map<ViewPreset, HTMLButtonElement>();
function markActivePreset(id: ViewPreset): void {
  for (const [key, button] of presetButtons) button.classList.toggle('active', key === id);
}
for (const preset of VIEW_PRESETS) {
  const button = document.createElement('button');
  button.textContent = preset.label;
  button.addEventListener('click', () => {
    if (exporting) return;
    mainScene.setPreset(preset.id, true);
    markActivePreset(preset.id);
  });
  presetButtons.set(preset.id, button);
  viewBar.append(button);
}
const resetViewButton = document.createElement('button');
resetViewButton.textContent = '重置视角';
resetViewButton.className = 'accent';
resetViewButton.addEventListener('click', () => {
  if (exporting) return;
  mainScene.resetView();
  markActivePreset('closeup');
});
viewBar.append(resetViewButton);
markActivePreset('closeup');

function recompute(): void {
  result = runSimulation(params);
  playbackTime = Math.min(playbackTime, result.tLast);
  mainScene.syncResult(result);
  curveView.setData(result.t, result.R, result.rTh);
  panel.setPerf(result.elapsedMs, 1000 / 60);
  panel.setPrecisionDelta(precisionDeltaText());
}

function scheduleRecompute(): void {
  if (recomputeQueued) return;
  recomputeQueued = true;
  window.requestAnimationFrame(() => {
    recomputeQueued = false;
    recompute();
  });
}

function precisionDeltaText(): string {
  if (params.lampSegments === LAMP_SEGMENTS_DEFAULT) return '0.000000（当前即 1024）';
  const i = frameIndexAt(result, playbackTime);
  const frame = result.lampFrames[i]!;
  const C = [result.Pd[i * 3]!, result.Pd[i * 3 + 1]!, result.Pd[i * 3 + 2]!];
  const uavCenter = [result.uavPos[i * 3]!, result.uavPos[i * 3 + 1]!, result.uavPos[i * 3 + 2]!];
  const current = computeOcclusion(
    C,
    uavCenter,
    result.uavSizeM,
    {
      ...frame,
      polygon2D: makeConvexPolygon(lampPolygon(frame.radiusM, params.lampSegments)),
    } satisfies LampFrame,
  ).Rocc;
  const reference = computeOcclusion(
    C,
    uavCenter,
    result.uavSizeM,
    {
      ...frame,
      polygon2D: makeConvexPolygon(lampPolygon(frame.radiusM, LAMP_SEGMENTS_DEFAULT)),
    } satisfies LampFrame,
  ).Rocc;
  return `${current.toFixed(6)} vs ${reference.toFixed(6)}（Δ=${Math.abs(current - reference).toFixed(
    6,
  )}）`;
}

function applyFrame(index: number, now: number): void {
  const i = Math.min(Math.max(index, 0), result.n - 1);
  mainScene.updateFrame(result, i);
  fpvScene.updateFrame(result, i);
  const t = result.t[i]!;
  curveView.setCursor(t, result.R[i]!, result.rTh);
  timeline.setProgress(t / Math.max(result.tLast, 1e-9), t, result.tLast);
  timeline.setReadouts(result.R[i]!, result.rTh, t, result.tLast);
  fpvOverlay.setReadouts(result.R[i]!, result.imageR[i]!, result.fovMargin[i]!);
  // 无命中告警：仅在末帧叠加黄字（播放到结尾时才提示，不打断过程观察）
  noHitWarning.hidden = result.hitLamp || i !== result.n - 1;
  mainScene.updateTransition(now);
}

function renderAll(): void {
  mainScene.renderer.render(mainScene.scene, mainScene.camera);
  fpvScene.render();
}

function applyLayout(): void {
  const vp = viewport();
  mainScene.resize(vp.width, vp.height);
  curveView.layoutTo(toPixels(DEFAULT_HUD_LAYOUT.curveArea, vp));
  timeline.layout(DEFAULT_HUD_LAYOUT, vp);
  fpvOverlay.layout(DEFAULT_HUD_LAYOUT, vp);
  const inset = fpvInsetPixels(DEFAULT_HUD_LAYOUT, vp);
  fpvScene.setSize(inset.w, inset.h);
  // 首次布局完成后再开启尺寸过渡，避免初始化时 left/width 被动画"追赶"
  window.setTimeout(() => fpvHost.classList.add('animate'), 400);
}

window.addEventListener('resize', () => applyLayout());
new ResizeObserver(() => fpvScene.setSize(fpvHost.clientWidth, fpvHost.clientHeight)).observe(
  fpvHost,
);

let persistTimer = 0;
function schedulePersist(): void {
  window.clearTimeout(persistTimer);
  persistTimer = window.setTimeout(() => savePersisted({ params, display, speed }), 500);
}

const exportButton = need<HTMLButtonElement>('btn-export');
exportButton.addEventListener('click', () => {
  void runExport();
});

async function runExport(): Promise<void> {
  if (exporting) return;
  const frames = Math.max(2, Math.round((result.tLast * EXPORT_FPS) / EXPORT_SPEED));
  const confirmed = await showModal({
    title: '导出 MP4',
    body:
      '首次导出需要初始化本地编码器资源（约 30 MB，由本地服务提供，无 CDN）。' +
      `导出视频为 ${EXPORT_FPS} fps、慢放 2×（${(frames / EXPORT_FPS).toFixed(2)} s），` +
      `完整覆盖 ${result.tLast.toFixed(2)} s 仿真内容。是否继续？`,
    confirmText: '开始导出',
  });
  if (!confirmed) return;

  exporting = true;
  playing = false;
  timeline.setPlaying(false);
  exportOverlay.hidden = false;
  const setProgress = (phase: string, frac: number): void => {
    exportStage.textContent = phase;
    exportPercent.textContent = `${Math.round(frac * 100)}%`;
    exportFill.style.width = `${Math.min(Math.max(frac, 0), 1) * 100}%`;
  };
  setProgress('准备中…', 0);

  try {
    const outcome = await exportMP4({
      result,
      scene: mainScene.scene,
      camera: mainScene.camera,
      fpvScene,
      uplotCanvas: curveView.uplotCanvas,
      effectsCanvas: curveView.effectsCanvas,
      updateFrame: (frameIndex) => {
        mainScene.updateFrame(result, frameIndex);
        fpvScene.updateFrame(result, frameIndex);
        curveView.setCursor(result.t[frameIndex]!, result.R[frameIndex]!, result.rTh);
      },
      // Line2 坐标轴按导出分辨率（1920×1080）计算线宽
      onRenderResolutionChange: (width, height) =>
        mainScene.setGridResolution(width, height),
      onProgress: setProgress,
    });

    const url = URL.createObjectURL(outcome.blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = `uav-occlusion-${outcome.frames}f.mp4`;
    document.body.append(link);
    link.click();
    link.remove();
    window.setTimeout(() => URL.revokeObjectURL(url), 60_000);
    console.info(
      `[export] 完成：${outcome.frames} 帧 / ${(outcome.bytes / 1024 / 1024).toFixed(1)} MB / ` +
        `${outcome.format} / ${(outcome.elapsedMs / 1000).toFixed(1)} s`,
    );
  } catch (error) {
    console.error('[export] 视频编码失败', error);
    const detail = error instanceof ExportError ? error.message : String(error);
    await showModal({
      title: '视频编码失败，请重试或降低分辨率',
      body: detail,
      confirmText: '知道了',
      cancelText: '关闭',
    });
  } finally {
    exporting = false;
    exportOverlay.hidden = true;
    // 恢复坐标轴线宽到屏幕分辨率
    const vp = viewport();
    mainScene.setGridResolution(vp.width, vp.height);
  }
}

function loop(now: number): void {
  const dtReal = (now - lastNow) / 1000;
  lastNow = now;

  if (playing && !exporting) {
    playbackTime += dtReal * speed;
    if (playbackTime >= result.tLast) {
      playbackTime = result.tLast;
      playing = false;
      timeline.setPlaying(false);
    }
  }

  if (!exporting) {
    applyFrame(frameIndexAt(result, playbackTime), now);
    renderAll();
  }
  window.requestAnimationFrame(loop);
}

applyLayout();
recompute();
timeline.setSpeed(speed);
timeline.setPlaying(playing);
mainScene.setPreset('closeup', false);
window.requestAnimationFrame(loop);

(window as unknown as { __uav?: unknown }).__uav = {
  get result() {
    return result;
  },
  get params() {
    return params;
  },
  get display() {
    return display;
  },
  /** 调试用：改显示选项（走面板 → 主场景 → 落盘的完整链路） */
  setDisplay: (partial: Partial<DisplayOptions>) => panel.setDisplay(partial),
};
