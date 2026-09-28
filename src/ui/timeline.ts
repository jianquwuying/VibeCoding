/**
 * 底部控制条：播放/暂停/重置 + 线性速度滑块（5 档磁吸）+ 进度条 + 时间读数。
 * 以及 HUD 第 3 层（HTML overlay 文字）的定位。
 */

import {
  COLORS,
  PLAYBACK_SPEED_MAX,
  PLAYBACK_SPEED_MIN,
  PLAYBACK_SPEED_SNAPS,
  SNAP_EPS,
} from '../config';
import {
  hudFontSizes,
  pointToPixels,
  toPixels,
  type HudLayout,
  type Viewport,
} from './hudLayout';

export interface TimelineElements {
  bar: HTMLDivElement;
  controls: HTMLDivElement;
  trackHost: HTMLDivElement;
  track: HTMLDivElement;
  progress: HTMLDivElement;
  thumb: HTMLDivElement;
  playButton: HTMLButtonElement;
  resetButton: HTMLButtonElement;
  speedSlider: HTMLInputElement;
  speedValue: HTMLSpanElement;
  speedHint: HTMLSpanElement;
  hudRocc: HTMLSpanElement;
  hudTime: HTMLSpanElement;
  hudSpeed: HTMLSpanElement;
}

export interface TimelineCallbacks {
  onPlayToggle(): void;
  onReset(): void;
  onSeekFraction(frac: number): void;
  onSpeedChange(speed: number): void;
}

export interface TimelineView {
  setPlaying(playing: boolean): void;
  setProgress(frac: number, t: number, tLast: number): void;
  setSpeed(speed: number): void;
  getSpeed(): number;
  /** 第 3 层文字：R_occ 数值平滑过渡（~150 ms lerp） */
  setReadouts(rOcc: number, rTh: number, t: number, tLast: number): void;
  layout(layout: HudLayout, viewport: Viewport): void;
  dispose(): void;
}

/** 线性滑块 + 5 档磁吸（距档位 < SNAP_EPS 时吸附）。 */
export function snapSpeed(value: number): number {
  for (const s of PLAYBACK_SPEED_SNAPS) {
    if (Math.abs(value - s) < SNAP_EPS) return s;
  }
  return value;
}

function lerp(a: number, b: number, w: number): number {
  return a + (b - a) * w;
}

function hexToRgb(hex: string): [number, number, number] {
  const v = hex.replace('#', '');
  return [
    parseInt(v.slice(0, 2), 16),
    parseInt(v.slice(2, 4), 16),
    parseInt(v.slice(4, 6), 16),
  ];
}

function mixColor(from: string, to: string, w: number): string {
  const a = hexToRgb(from);
  const b = hexToRgb(to);
  return `rgb(${Math.round(lerp(a[0], b[0], w))}, ${Math.round(lerp(a[1], b[1], w))}, ${Math.round(
    lerp(a[2], b[2], w),
  )})`;
}

export function createTimelineView(
  elements: TimelineElements,
  callbacks: TimelineCallbacks,
  initialSpeed: number,
): TimelineView {
  const el = elements;
  let speed = initialSpeed;
  let dragging = false;
  let displayedR = 0;
  let targetR = 0;
  let colorW = 1;
  let colorTarget = 1;
  /** 当前帧为无效帧（valid=0，R = NaN）→ HUD 显示 "—" */
  let rInvalid = false;

  el.speedSlider.min = String(PLAYBACK_SPEED_MIN);
  el.speedSlider.max = String(PLAYBACK_SPEED_MAX);
  el.speedSlider.step = '0.01';
  el.speedSlider.value = String(speed);

  const renderSpeed = (): void => {
    el.speedSlider.value = String(speed);
    el.speedValue.textContent = `${speed.toFixed(2)}×`;
    el.hudSpeed.textContent = `${speed.toFixed(2)}×`;
    el.speedHint.textContent = `当前预览 ${speed.toFixed(2)}×，导出视频固定 0.50×`;
  };

  el.playButton.addEventListener('click', () => callbacks.onPlayToggle());
  el.resetButton.addEventListener('click', () => callbacks.onReset());

  el.speedSlider.addEventListener('input', () => {
    const raw = Number.parseFloat(el.speedSlider.value);
    const snapped = snapSpeed(raw);
    speed = Math.min(Math.max(snapped, PLAYBACK_SPEED_MIN), PLAYBACK_SPEED_MAX);
    if (snapped !== raw) {
      // 吸附瞬间的轻微弹性动画（100 ms）
      el.speedSlider.classList.add('snap-pop');
      window.setTimeout(() => el.speedSlider.classList.remove('snap-pop'), 100);
    }
    renderSpeed();
    callbacks.onSpeedChange(speed);
  });

  const seekFromPointer = (clientX: number): void => {
    const rect = el.track.getBoundingClientRect();
    const frac = Math.min(Math.max((clientX - rect.left) / Math.max(rect.width, 1), 0), 1);
    callbacks.onSeekFraction(frac);
  };

  const onPointerDown = (event: PointerEvent): void => {
    dragging = true;
    el.track.setPointerCapture(event.pointerId);
    seekFromPointer(event.clientX);
  };
  const onPointerMove = (event: PointerEvent): void => {
    if (dragging) seekFromPointer(event.clientX);
  };
  const onPointerUp = (event: PointerEvent): void => {
    dragging = false;
    if (el.track.hasPointerCapture(event.pointerId)) el.track.releasePointerCapture(event.pointerId);
  };
  el.track.addEventListener('pointerdown', onPointerDown);
  el.track.addEventListener('pointermove', onPointerMove);
  el.track.addEventListener('pointerup', onPointerUp);
  el.track.addEventListener('pointercancel', onPointerUp);

  renderSpeed();

  const layout = (hudLayout: HudLayout, viewport: Viewport): void => {
    const band = toPixels(hudLayout.timeline, viewport);
    const fonts = hudFontSizes(hudLayout, viewport);
    el.bar.style.left = `${band.x}px`;
    el.bar.style.top = `${band.y}px`;
    el.bar.style.width = `${band.w}px`;
    el.bar.style.height = `${band.h}px`;

    // 带内三行：控制条 / 读数行（层 3）/ 进度条
    el.controls.style.left = '0';
    el.controls.style.top = '0';
    el.controls.style.width = '100%';
    el.controls.style.height = `${band.h * 0.3}px`;
    el.trackHost.style.left = '0';
    el.trackHost.style.top = `${band.h * 0.66}px`;
    el.trackHost.style.width = '100%';
    el.trackHost.style.height = `${band.h * 0.34}px`;

    el.controls.style.fontSize = `${fonts.meta}px`;
    el.speedValue.style.fontSize = `${fonts.readout}px`;
    el.speedHint.style.fontSize = `${fonts.meta * 0.62}px`;

    const anchors = [hudLayout.textLeft, hudLayout.textCenter, hudLayout.textRight];
    const spans = [el.hudRocc, el.hudTime, el.hudSpeed];
    for (let i = 0; i < spans.length; i += 1) {
      const p = pointToPixels(anchors[i]!, viewport);
      const span = spans[i]!;
      span.style.top = `${p.y}px`;
      span.style.fontSize = `${fonts.readout * (i === 0 ? 1 : 0.75)}px`;
      if (i === 0) {
        span.style.left = `${p.x}px`;
        span.style.transform = 'translateY(-100%)';
      } else if (i === 1) {
        span.style.left = `${p.x}px`;
        span.style.transform = 'translate(-50%, -100%)';
      } else {
        span.style.left = `${p.x}px`;
        span.style.transform = 'translate(-100%, -100%)';
      }
    }
    el.hudSpeed.style.color = COLORS.accent;
  };

  const setReadouts = (rOcc: number, rTh: number, t: number, tLast: number): void => {
    rInvalid = !Number.isFinite(rOcc);
    if (!rInvalid) {
      targetR = rOcc;
      colorTarget = rOcc >= rTh ? 1 : 0;
    }
    el.hudTime.textContent = `t = ${t.toFixed(2)} / ${tLast.toFixed(2)} s`;
  };

  const tickSmoothing = (): void => {
    if (rInvalid) {
      el.hudRocc.textContent = 'R_occ = —';
      el.hudRocc.style.color = COLORS.text;
      window.requestAnimationFrame(tickSmoothing);
      return;
    }
    if (Math.abs(displayedR - targetR) > 1e-6 || Math.abs(colorW - colorTarget) > 1e-4) {
      displayedR = lerp(displayedR, targetR, 0.22);
      colorW = lerp(colorW, colorTarget, 0.22);
      el.hudRocc.textContent = `R_occ = ${displayedR.toFixed(3)}`;
      el.hudRocc.style.color = mixColor(COLORS.threshold, COLORS.effective, colorW);
    }
    window.requestAnimationFrame(tickSmoothing);
  };
  window.requestAnimationFrame(tickSmoothing);

  return {
    setPlaying: (playing: boolean) => {
      el.playButton.textContent = playing ? '⏸' : '▶';
    },
    setProgress: (frac, t, tLast) => {
      const f = Math.min(Math.max(frac, 0), 1);
      el.progress.style.width = `${f * 100}%`;
      el.thumb.style.left = `${f * 100}%`;
      el.hudTime.textContent = `t = ${t.toFixed(2)} / ${tLast.toFixed(2)} s`;
    },
    setSpeed: (value: number) => {
      speed = value;
      renderSpeed();
    },
    getSpeed: () => speed,
    setReadouts,
    layout,
    dispose: () => {
      el.track.removeEventListener('pointerdown', onPointerDown);
      el.track.removeEventListener('pointermove', onPointerMove);
      el.track.removeEventListener('pointerup', onPointerUp);
      el.track.removeEventListener('pointercancel', onPointerUp);
    },
  };
}
