/**
 * HUD 第 3 层（文字）的 Canvas2D 重绘：导出时用它还原网页预览里的 HTML overlay，
 * 保证"所见即所得"。位置/字号一律来自 ui/hudLayout（不硬编码像素）。
 */

import { COLORS } from '../config';
import {
  fpvInsetPixels,
  hudFontSizes,
  pointToPixels,
  toPixels,
  type HudLayout,
  type Viewport,
} from '../ui/hudLayout';

export interface HudTextState {
  rOcc: number;
  rTh: number;
  t: number;
  tLast: number;
  speed: number;
  imageRatio: number;
  fovMarginDeg: number;
  title: string;
}

const FONT_STACK = 'Inter, system-ui, "Segoe UI", sans-serif';

export function drawHudText(
  ctx: CanvasRenderingContext2D,
  state: HudTextState,
  layout: HudLayout,
  viewport: Viewport,
): void {
  const fonts = hudFontSizes(layout, viewport);
  const effective = state.rOcc >= state.rTh;
  ctx.save();
  ctx.textBaseline = 'alphabetic';
  ctx.textAlign = 'left';

  const left = pointToPixels(layout.textLeft, viewport);
  ctx.font = `500 ${fonts.readout}px ${FONT_STACK}`;
  // 无效帧（valid=0，R = NaN）→ "—"（与网页预览的 HTML overlay 一致）
  const rValid = Number.isFinite(state.rOcc);
  ctx.fillStyle = rValid ? (effective ? COLORS.effective : COLORS.threshold) : COLORS.text;
  ctx.fillText(rValid ? `R_occ = ${state.rOcc.toFixed(3)}` : 'R_occ = —', left.x, left.y);

  const center = pointToPixels(layout.textCenter, viewport);
  ctx.textAlign = 'center';
  ctx.fillStyle = COLORS.text;
  ctx.fillText(
    `t = ${state.t.toFixed(2)} / ${state.tLast.toFixed(2)} s`,
    center.x,
    center.y - fonts.readout * 0.12,
  );

  const right = pointToPixels(layout.textRight, viewport);
  ctx.textAlign = 'right';
  ctx.fillStyle = COLORS.accent;
  ctx.fillText(`${state.speed.toFixed(2)}×`, right.x, right.y - fonts.readout * 0.12);

  // FPV 内嵌下方的三行读数
  const fpv = fpvInsetPixels(layout, viewport);
  const line = Math.max(10, fonts.meta * 0.72);
  ctx.textAlign = 'left';
  ctx.font = `500 ${line}px ${FONT_STACK}`;
  ctx.fillStyle = COLORS.text;
  ctx.fillText(
    rValid ? `R_occ ${state.rOcc.toFixed(3)}` : 'R_occ —',
    fpv.x,
    fpv.y + fpv.h + line * 1.15,
  );
  ctx.fillText(
    Number.isFinite(state.imageRatio) ? `像面遮挡比 ${state.imageRatio.toFixed(3)}` : '像面遮挡比 —',
    fpv.x,
    fpv.y + fpv.h + line * 2.3,
  );
  ctx.fillText(
    Number.isFinite(state.fovMarginDeg)
      ? `视场余量 ${state.fovMarginDeg.toFixed(2)}°`
      : '视场余量 —',
    fpv.x,
    fpv.y + fpv.h + line * 3.45,
  );

  // 左上角标题
  const titleX = toPixels(layout.timeline, viewport).x + fpv.x * 0 + 16;
  ctx.font = `500 ${line}px ${FONT_STACK}`;
  ctx.fillStyle = 'rgba(248, 250, 252, 0.75)';
  ctx.fillText(state.title, titleX, line * 1.6);
  ctx.restore();
}
