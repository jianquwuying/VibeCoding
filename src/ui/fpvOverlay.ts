/**
 * FPV 小窗定位与悬停放大（尺寸一律由 hudLayout 推导，不硬编码像素）。
 *
 * 锚点固定在**右上角**：用 `right` 而不是 `left`，因此放大时右上角不动、
 * 左边缘与下边缘向外扩展（向画面中心方向生长）。
 */

import {
  fpvHoverPixels,
  fpvInsetPixels,
  type HudLayout,
  type HudRegion,
  type Viewport,
} from './hudLayout';

export interface FpvOverlay {
  /** FPV 画布的 CSS 尺寸（像素） */
  size(): { width: number; height: number };
  layout(layout: HudLayout, viewport: Viewport): void;
  setVisible(visible: boolean): void;
  setReadouts(rOcc: number, imageRatio: number, marginDeg: number): void;
  isEnlarged(): boolean;
  dispose(): void;
}

export function createFpvOverlay(
  host: HTMLDivElement,
  canvas: HTMLCanvasElement,
  roc: HTMLSpanElement,
  imageRatio: HTMLSpanElement,
  margin: HTMLSpanElement,
): FpvOverlay {
  let baseWidth = 0;
  let baseHeight = 0;
  let enlarged = false;
  let hoverRect: HudRegion | null = null;

  const applySize = (): void => {
    if (baseWidth <= 0) return;
    const rect =
      enlarged && hoverRect ? hoverRect : { x: 0, y: 0, w: baseWidth, h: baseHeight };
    host.style.width = `${rect.w}px`;
    host.style.height = `${rect.h}px`;
    // 放大时改的是 CSS 尺寸，正交边界（0..640 / 0..480）保持不变
    host.classList.toggle('fpv-enlarged', enlarged);
  };

  const onEnter = (): void => {
    enlarged = true;
    applySize();
  };
  const onLeave = (): void => {
    enlarged = false;
    applySize();
  };
  host.addEventListener('mouseenter', onEnter);
  host.addEventListener('mouseleave', onLeave);

  return {
    size: () => ({ width: baseWidth, height: baseHeight }),
    layout: (layout, viewport) => {
      const rect = fpvInsetPixels(layout, viewport);
      baseWidth = rect.w;
      baseHeight = rect.h;
      hoverRect = fpvHoverPixels(layout, viewport);
      // 右上角锚定：right 值由 hudLayout 推导（恒等于 1.2% 视口宽），
      // 不留 left，否则 left 会压过 right 使放大向右下扩展。
      host.style.left = 'auto';
      host.style.right = `${viewport.width - (rect.x + rect.w)}px`;
      host.style.top = `${rect.y}px`;
      applySize();
    },
    setVisible: (visible: boolean) => {
      host.style.display = visible ? 'block' : 'none';
    },
    setReadouts: (rOcc, ratio, marginDeg) => {
      // 无效帧（valid=0）的 R 为 NaN → "—"
      roc.textContent = Number.isFinite(rOcc) ? `R_occ ${rOcc.toFixed(3)}` : 'R_occ —';
      imageRatio.textContent = Number.isFinite(ratio)
        ? `像面遮挡比 ${ratio.toFixed(3)}`
        : '像面遮挡比 —';
      margin.textContent = Number.isFinite(marginDeg)
        ? `视场余量 ${marginDeg.toFixed(2)}°`
        : '视场余量 —';
    },
    isEnlarged: () => enlarged,
    dispose: () => {
      host.removeEventListener('mouseenter', onEnter);
      host.removeEventListener('mouseleave', onLeave);
      void canvas;
    },
  };
}
