/**
 * R_occ 曲线（方案 C 三层 HUD 的第 1、2 层）。
 *
 * 层 1 = uPlot 画布：曲线 + 坐标轴 + 网格 + R_th 虚线（仅数据/参数变化时重绘）。
 * 层 2 = Canvas2D 特效层：曲线发光、有效遮挡区间渐变填充、当前帧游标 + 脉冲圆环
 *        （每帧重绘，尺寸/位置与 uPlot 画布及 DPR 完全一致）。
 *
 * 游标刻意不交给 uPlot 的内部 cursor 状态绘制，这样导出时只需
 * drawImage(uplotCanvas) + drawImage(effectsCanvas) 即可逐帧还原预览。
 */

import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';

import { COLORS } from '../config';
import type { HudRegion } from './hudLayout';

export interface CurveView {
  root: HTMLDivElement;
  uplotCanvas: HTMLCanvasElement;
  effectsCanvas: HTMLCanvasElement;
  setData(t: Float64Array, R: Float64Array, rTh: number): void;
  setCursor(t: number, rOcc: number, rTh: number): void;
  /** 用 hudLayout 换算后的像素矩形定位（不做硬编码像素） */
  layoutTo(rect: HudRegion): void;
  setVisible(visible: boolean): void;
  onSeek(cb: (t: number) => void): void;
  dispose(): void;
}

export function createCurveView(
  host: HTMLDivElement,
  uplotHost: HTMLDivElement,
  effectsCanvas: HTMLCanvasElement,
): CurveView {
  let tLast = 1;
  let plot: uPlot | null = null;
  let seekHandler: ((t: number) => void) | null = null;
  let currentT = 0;
  let currentR = 0;
  let currentTh = 0.7;
  let dpr = Math.min(window.devicePixelRatio || 1, 2);

  const buildPlot = (t: Float64Array, R: Float64Array, rTh: number): void => {
    tLast = Math.max(t[t.length - 1] ?? 1, 1e-6);
    const threshold = new Float64Array(R.length).fill(rTh);
    const opts: uPlot.Options = {
      width: Math.max(host.clientWidth, 64),
      height: Math.max(host.clientHeight, 48),
      padding: [6, 10, 0, 0],
      legend: { show: false },
      cursor: { show: false },
      scales: {
        x: { time: false, range: [0, tLast] },
        y: { range: [0, 1] },
      },
      axes: [
        {
          stroke: COLORS.text,
          grid: { stroke: COLORS.grid, dash: [4, 4] },
          ticks: { stroke: COLORS.grid },
          font: '11px Inter, system-ui, sans-serif',
          values: (_u, vals) => vals.map((v) => v.toFixed(1)),
        },
        {
          stroke: COLORS.text,
          grid: { stroke: COLORS.grid, dash: [4, 4] },
          ticks: { stroke: COLORS.grid },
          font: '11px Inter, system-ui, sans-serif',
          values: (_u, vals) => vals.map((v) => v.toFixed(1)),
        },
      ],
      series: [
        {},
        { stroke: COLORS.primary, width: 1.5, points: { show: false } },
        { stroke: COLORS.threshold, width: 1, dash: [4, 4], points: { show: false } },
      ],
    };
    // 无效帧（valid=0）的 R 为 NaN：转成 null，uPlot 会在该点断线
    const rDisplay: Array<number | null> = Array.from(R, (v) => (Number.isNaN(v) ? null : v));
    const data: uPlot.AlignedData = [Array.from(t), rDisplay, Array.from(threshold)];
    if (plot) {
      plot.destroy();
      plot = null;
    }
    uplotHost.replaceChildren();
    plot = new uPlot(opts, data, uplotHost);
    attachSeek();
    resizeEffects();
  };

  const attachSeek = (): void => {
    if (!plot) return;
    const over = uplotHost.querySelector('.u-over') as HTMLElement | null;
    if (!over) return;
    over.style.cursor = 'crosshair';
    over.addEventListener('click', (event) => {
      if (!plot || !seekHandler) return;
      const rect = over.getBoundingClientRect();
      const localX = (event as MouseEvent).clientX - rect.left;
      const t = plot.posToVal(localX, 'x');
      seekHandler(Math.min(Math.max(t, 0), tLast));
    });
  };

  const resizeEffects = (): void => {
    dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.max(host.clientWidth, 1);
    const h = Math.max(host.clientHeight, 1);
    effectsCanvas.style.width = `${w}px`;
    effectsCanvas.style.height = `${h}px`;
    effectsCanvas.width = Math.round(w * dpr);
    effectsCanvas.height = Math.round(h * dpr);
  };

  const effectsCtx = (): CanvasRenderingContext2D | null => effectsCanvas.getContext('2d');

  const drawEffects = (): void => {
    const ctx = effectsCtx();
    if (!ctx || !plot) return;
    const w = effectsCanvas.width;
    const h = effectsCanvas.height;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const data = plot.data;
    const xs = data[0] as number[];
    // 无效帧为 null（断线）
    const ys = data[1] as Array<number | null>;
    if (!xs || xs.length === 0) return;

    const px = (t: number): number => plot!.valToPos(t, 'x', true);
    const py = (v: number): number => plot!.valToPos(v, 'y', true);

    // uPlot 首次 setData 后 scale 尚未就绪时 valToPos 会返回 NaN，
    // 直接用来建渐变会抛异常并中断整条初始化链，故先做有限性检查。
    const yBottom = py(0);
    const yTop = py(1);
    if (!Number.isFinite(yBottom) || !Number.isFinite(yTop)) return;

    // 有效遮挡区间：曲线下方渐变填充
    const bottom = yBottom;
    let runStart: number | null = null;
    const flushRun = (endIdx: number): void => {
      if (runStart === null) return;
      const x0 = px(xs[runStart]!);
      const x1 = px(xs[endIdx]!);
      const grad = ctx.createLinearGradient(0, yTop, 0, bottom);
      grad.addColorStop(0, COLORS.effectiveFill);
      grad.addColorStop(1, COLORS.effectiveFillEnd);
      ctx.beginPath();
      ctx.moveTo(x0, bottom);
      for (let k = runStart; k <= endIdx; k += 1) ctx.lineTo(px(xs[k]!), py(ys[k]!));
      ctx.lineTo(x1, bottom);
      ctx.closePath();
      ctx.fillStyle = grad;
      ctx.fill();
      runStart = null;
    };
    for (let k = 0; k < xs.length; k += 1) {
      const v = ys[k];
      const effective = v !== null && v !== undefined && v >= currentTh;
      if (effective && runStart === null) runStart = k;
      if (!effective && runStart !== null) flushRun(k - 1);
    }
    if (runStart !== null) flushRun(xs.length - 1);

    // 曲线发光（沿曲线描一遍更宽的半透明线；null 处断开）
    ctx.beginPath();
    let started = false;
    for (let k = 0; k < xs.length; k += 1) {
      const v = ys[k];
      if (v === null || v === undefined) {
        started = false;
        continue;
      }
      const x = px(xs[k]!);
      const y = py(v);
      if (!started) {
        ctx.moveTo(x, y);
        started = true;
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.lineWidth = 5.5;
    ctx.strokeStyle = COLORS.primaryGlow;
    ctx.lineJoin = 'round';
    ctx.stroke();

    // 当前帧游标 + 脉冲圆环
    const cx = px(currentT);
    if (!Number.isFinite(cx)) return;
    const top = yTop;
    ctx.beginPath();
    ctx.moveTo(cx, top);
    ctx.lineTo(cx, bottom);
    ctx.lineWidth = 1;
    ctx.strokeStyle = COLORS.accent;
    ctx.stroke();

    // 无效帧（R = NaN）不画脉冲圆环
    const cursorY = py(currentR);
    if (!Number.isFinite(cursorY)) return;
    const phase = (performance.now() % 1200) / 1200;
    const radius = 3 + phase * 7;
    ctx.beginPath();
    ctx.arc(cx, cursorY, radius, 0, Math.PI * 2);
    ctx.strokeStyle = `rgba(245, 158, 11, ${(0.6 * (1 - phase)).toFixed(3)})`;
    ctx.lineWidth = 1.5;
    ctx.stroke();
  };

  const setData = (t: Float64Array, R: Float64Array, rTh: number): void => {
    currentTh = rTh;
    buildPlot(t, R, rTh);
    // 等 uPlot 完成一次布局后再画特效层
    window.requestAnimationFrame(drawEffects);
  };

  const setCursor = (t: number, rOcc: number, rTh: number): void => {
    currentT = Math.min(Math.max(t, 0), tLast);
    currentR = rOcc;
    currentTh = rTh;
    drawEffects();
  };

  const layoutTo = (rect: HudRegion): void => {
    host.style.left = `${rect.x}px`;
    host.style.top = `${rect.y}px`;
    host.style.width = `${rect.w}px`;
    host.style.height = `${rect.h}px`;
    uplotHost.style.width = '100%';
    uplotHost.style.height = '100%';
    if (plot) plot.setSize({ width: Math.round(rect.w), height: Math.round(rect.h) });
    resizeEffects();
    window.requestAnimationFrame(drawEffects);
  };

  return {
    root: host,
    get uplotCanvas() {
      return (plot?.ctx.canvas ?? document.createElement('canvas')) as HTMLCanvasElement;
    },
    effectsCanvas,
    setData,
    setCursor,
    layoutTo,
    setVisible: (visible: boolean) => {
      host.style.display = visible ? 'block' : 'none';
    },
    onSeek: (cb) => {
      seekHandler = cb;
    },
    dispose: () => {
      plot?.destroy();
      plot = null;
    },
  };
}
