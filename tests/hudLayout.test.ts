import { describe, expect, it } from 'vitest';

import {
  DEFAULT_HUD_LAYOUT,
  FPV_INSET_RATIO,
  FPV_INSET_WIDTH_RATIO,
  fpvHoverPixels,
  fpvInsetPixels,
  hudFontSizes,
  pointToPixels,
  toPixels,
} from '../src/ui/hudLayout';

const fhd = { width: 1920, height: 1080 };
const hd = { width: 1600, height: 900 };

describe('HUD 布局', () => {
  it('1920×1080 下 FPV 内嵌为 480×360（4:3，不拉伸）', () => {
    const r = fpvInsetPixels(DEFAULT_HUD_LAYOUT, fhd);
    expect(r.w).toBeCloseTo(480, 9);
    expect(r.h).toBeCloseTo(360, 9);
    // 留白与宽度都从具名常量推导，测试内不写字面量
    expect(r.x + r.w).toBeCloseTo((1 - FPV_INSET_RATIO) * fhd.width, 9);
    expect(r.y).toBeCloseTo(FPV_INSET_RATIO * fhd.height, 9);
    expect(r.w).toBeCloseTo(FPV_INSET_WIDTH_RATIO * fhd.width, 9);
  });

  it('1600×900 下 FPV 内嵌为 400×300', () => {
    const r = fpvInsetPixels(DEFAULT_HUD_LAYOUT, hd);
    expect(r.w).toBeCloseTo(400, 9);
    expect(r.h).toBeCloseTo(300, 9);
  });

  it('曲线区在时间轴正上方且不重叠（曲线在上、控制条在下）', () => {
    const curve = toPixels(DEFAULT_HUD_LAYOUT.curveArea, fhd);
    const timeline = toPixels(DEFAULT_HUD_LAYOUT.timeline, fhd);
    expect(curve.y + curve.h).toBeCloseTo(timeline.y, 9);
    expect(timeline.y + timeline.h).toBeCloseTo(fhd.height, 9);
    // 曲线必须位于画布下半部分（底部控制条上方），而不是顶部
    expect(curve.y).toBeGreaterThan(fhd.height * 0.5);
  });

  it('三个文字锚点都落在时间轴 band 内', () => {
    const timeline = DEFAULT_HUD_LAYOUT.timeline;
    for (const anchor of [
      DEFAULT_HUD_LAYOUT.textLeft,
      DEFAULT_HUD_LAYOUT.textCenter,
      DEFAULT_HUD_LAYOUT.textRight,
    ]) {
      expect(anchor.y).toBeGreaterThan(timeline.y);
      expect(anchor.y).toBeLessThan(timeline.y + timeline.h);
    }
  });

  it('FPV 内嵌不与文字锚点/时间轴 band 重叠', () => {
    for (const viewport of [fhd, hd]) {
      const fpv = fpvInsetPixels(DEFAULT_HUD_LAYOUT, viewport);
      const centers = [DEFAULT_HUD_LAYOUT.textLeft, DEFAULT_HUD_LAYOUT.textCenter];
      for (const c of centers) {
        const p = pointToPixels(c, viewport);
        const insideFpv =
          p.x >= fpv.x && p.x <= fpv.x + fpv.w && p.y >= fpv.y && p.y <= fpv.y + fpv.h;
        expect(insideFpv).toBe(false);
      }
      expect(fpv.y + fpv.h).toBeLessThan(DEFAULT_HUD_LAYOUT.curveArea.y * viewport.height);
    }
  });

  it('字号由 band 高度推导（无硬编码像素）', () => {
    const a = hudFontSizes(DEFAULT_HUD_LAYOUT, fhd);
    const b = hudFontSizes(DEFAULT_HUD_LAYOUT, hd);
    expect(a.readout).toBeGreaterThan(b.readout);
    expect(a.readout).toBeCloseTo(DEFAULT_HUD_LAYOUT.timeline.h * fhd.height * 0.17, 9);
  });
});

describe('FPV hover (anchored to the top-right corner)', () => {
  for (const viewport of [fhd, hd]) {
    const label = `${viewport.width}x${viewport.height}`;

    it(`${label}: base right edge = (1 - FPV_INSET_RATIO) * viewport.width`, () => {
      const base = fpvInsetPixels(DEFAULT_HUD_LAYOUT, viewport);
      expect(base.x + base.w).toBeCloseTo((1 - FPV_INSET_RATIO) * viewport.width, 9);
    });

    it(`${label}: hover keeps the top-right corner fixed and grows left+down`, () => {
      const base = fpvInsetPixels(DEFAULT_HUD_LAYOUT, viewport);
      const hover = fpvHoverPixels(DEFAULT_HUD_LAYOUT, viewport);
      expect(hover.x + hover.w).toBeCloseTo(base.x + base.w, 9); // right edge fixed
      expect(hover.y).toBeCloseTo(base.y, 9); // top edge fixed
      expect(hover.w).toBeGreaterThan(base.w); // grows left
      expect(hover.h).toBeGreaterThan(base.h); // grows down
    });
  }

  it('1080p hover = 648x486 (1.5x capped by 60% of viewport height)', () => {
    const hover = fpvHoverPixels(DEFAULT_HUD_LAYOUT, fhd);
    expect(hover.w).toBeCloseTo(648, 9);
    expect(hover.h).toBeCloseTo(486, 9);
  });

  it('1600x900 hover = 540x405', () => {
    const hover = fpvHoverPixels(DEFAULT_HUD_LAYOUT, hd);
    expect(hover.w).toBeCloseTo(540, 9);
    expect(hover.h).toBeCloseTo(405, 9);
  });
});
