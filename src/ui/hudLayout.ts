import {
  FPV_HOVER_MAX_HEIGHT_FRACTION,
  FPV_HOVER_SCALE,
} from '../config';

/**
 * HUD 布局（钉死）：屏幕与导出**共用同一份**归一化布局描述。
 *
 * 坐标语义：归一化 0~1，x 向右、**y 向下**，相对于整个画布。
 * 任何渲染/导出代码都不得出现硬编码像素值，一律经 toPixels / pointToPixels 换算。
 */

export interface HudRegion {
  x: number;
  y: number;
  w: number;
  h: number;
}

export interface HudLayout {
  /** 曲线区（uPlot + 特效层共用） */
  curveArea: HudRegion;
  /** 时间轴 / 播放控制条区域（HTML overlay 用） */
  timeline: HudRegion;
  /** FPV 内嵌区（主视口右上角；h 由宽高比现算，忽略其 h 字段） */
  fpvInset: HudRegion;
  /** 三处文字锚点（HTML overlay 用，y 为文字基线所在比例位置） */
  textLeft: { x: number; y: number };
  textCenter: { x: number; y: number };
  textRight: { x: number; y: number };
}

export interface Viewport {
  width: number;
  height: number;
}

/** FPV 内嵌固定 4:3（与 640×480 成像一致）。 */
export const FPV_INSET_ASPECT = 4 / 3;

/** FPV 内嵌右上角留白（归一化，宽高各 1.2%）。 */
export const FPV_INSET_RATIO = 0.012;
/** FPV 内嵌宽度占视口宽的比例。 */
export const FPV_INSET_WIDTH_RATIO = 0.25;

export const DEFAULT_HUD_LAYOUT: HudLayout = {
  // 曲线区：底部控制条上方，占画布高 24%
  curveArea: { x: 0.0, y: 0.63, w: 1.0, h: 0.24 },
  // 时间轴：最底部控制条，占画布高 13%
  timeline: { x: 0.0, y: 0.87, w: 1.0, h: 0.13 },
  // FPV 内嵌：右上角，宽 25%，右上留 1.2% 边距（x 由留白与宽度推导，避免字面量漂移）；
  // h 由 fpvInsetPixels 现算
  fpvInset: {
    x: 1 - FPV_INSET_RATIO - FPV_INSET_WIDTH_RATIO,
    y: FPV_INSET_RATIO,
    w: FPV_INSET_WIDTH_RATIO,
    h: 0.0,
  },
  // 三处文字锚点：位于 timeline band 内（0.87 + 0.13/2 = 0.935），y 为基线
  textLeft: { x: 0.02, y: 0.935 },
  textCenter: { x: 0.5, y: 0.935 },
  textRight: { x: 0.98, y: 0.935 },
};

/** 归一化区域 → 像素区域。 */
export function toPixels(region: HudRegion, viewport: Viewport): HudRegion {
  return {
    x: region.x * viewport.width,
    y: region.y * viewport.height,
    w: region.w * viewport.width,
    h: region.h * viewport.height,
  };
}

/** 归一化锚点 → 像素锚点。 */
export function pointToPixels(
  point: { x: number; y: number },
  viewport: Viewport,
): { x: number; y: number } {
  return { x: point.x * viewport.width, y: point.y * viewport.height };
}

/**
 * FPV 内嵌的像素矩形：宽度取自 fpvInset.w，高度按 4:3 现算
 * （h = (w·viewport.width) / aspect / viewport.height）。
 */
export function fpvInsetPixels(layout: HudLayout, viewport: Viewport): HudRegion {
  const widthPx = layout.fpvInset.w * viewport.width;
  const heightPx = widthPx / FPV_INSET_ASPECT;
  return {
    x: layout.fpvInset.x * viewport.width,
    y: layout.fpvInset.y * viewport.height,
    w: widthPx,
    h: heightPx,
  };
}

/**
 * FPV 悬停放大后的像素矩形（右上角锚点不变，只改尺寸）。
 *
 * 宽度 = min(基准宽 × FPV_HOVER_SCALE, 视口高 × FPV_HOVER_MAX_HEIGHT_FRACTION)，
 * 高度按 4:3 现算。1080p → 648×486、1600×900 → 540×405（均 1.35×）。
 */
export function fpvHoverPixels(layout: HudLayout, viewport: Viewport): HudRegion {
  const base = fpvInsetPixels(layout, viewport);
  const capByHeight = viewport.height * FPV_HOVER_MAX_HEIGHT_FRACTION;
  const widthPx = Math.min(base.w * FPV_HOVER_SCALE, capByHeight);
  const heightPx = widthPx / FPV_INSET_ASPECT;
  // 锚点固定在右上角：右下角坐标不变，左上角随尺寸向左下扩展
  return {
    x: base.x + base.w - widthPx,
    y: base.y,
    w: widthPx,
    h: heightPx,
  };
}

/** 依据 viewport 与布局推导的 HUD 字号（避免硬编码像素值）。 */
export function hudFontSizes(layout: HudLayout, viewport: Viewport): {
  readout: number;
  meta: number;
} {
  const band = layout.timeline.h * viewport.height;
  return {
    readout: Math.max(12, band * 0.17),
    meta: Math.max(10, band * 0.128),
  };
}
