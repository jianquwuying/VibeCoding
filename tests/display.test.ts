/**
 * 显示选项净化单测：保证新增字段（网格 4 样式项）不会让旧存档失效，
 * 且非法/损坏的值一律回落默认。
 */

import { describe, expect, it } from 'vitest';

import { DEFAULT_DISPLAY, GRID_WIDTH_RANGE, sanitizeDisplay } from '../src/config';

describe('sanitizeDisplay', () => {
  it('无参 / null / undefined → 等于默认值，且不是同一对象引用', () => {
    for (const input of [undefined, null]) {
      const out = sanitizeDisplay(input);
      expect(out).toEqual(DEFAULT_DISPLAY);
      expect(out).not.toBe(DEFAULT_DISPLAY);
    }
  });

  it('顶层非对象（字符串 / 数字 / 布尔 / 数组）→ 全默认', () => {
    for (const input of ['foo', 42, true, [] as unknown]) {
      expect(sanitizeDisplay(input)).toEqual(DEFAULT_DISPLAY);
    }
  });

  it('旧存档（只有旧字段）→ 保留旧值、新字段补默认', () => {
    const out = sanitizeDisplay({ trail: false, curve: false });
    expect(out.trail).toBe(false);
    expect(out.curve).toBe(false);
    expect(out.gridPlane).toBe(DEFAULT_DISPLAY.gridPlane);
    expect(out.gridCoarseColor).toBe(DEFAULT_DISPLAY.gridCoarseColor);
    expect(out.gridFineWidth).toBe(DEFAULT_DISPLAY.gridFineWidth);
  });

  it('非法布尔值回落默认', () => {
    for (const bad of ['yes', 1, null, {}]) {
      const out = sanitizeDisplay({ showCoarse: bad, showFine: bad, showAxes: bad });
      expect(out.showCoarse).toBe(DEFAULT_DISPLAY.showCoarse);
      expect(out.showFine).toBe(DEFAULT_DISPLAY.showFine);
      expect(out.showAxes).toBe(DEFAULT_DISPLAY.showAxes);
    }
  });

  it('gridPlane 三值白名单', () => {
    for (const good of ['lamp', 'horizontal', 'both'] as const) {
      expect(sanitizeDisplay({ gridPlane: good }).gridPlane).toBe(good);
    }
    for (const bad of ['foo', '', 3, null, {}]) {
      expect(sanitizeDisplay({ gridPlane: bad }).gridPlane).toBe(DEFAULT_DISPLAY.gridPlane);
    }
  });

  it('颜色必须是 6 位 hex 字符串', () => {
    expect(sanitizeDisplay({ gridCoarseColor: '#ff0000' }).gridCoarseColor).toBe('#ff0000');
    expect(sanitizeDisplay({ gridFineColor: '#00FF00' }).gridFineColor).toBe('#00FF00');
    for (const bad of ['red', '#fff', 'ff0000', 123, null]) {
      expect(sanitizeDisplay({ gridCoarseColor: bad }).gridCoarseColor).toBe(
        DEFAULT_DISPLAY.gridCoarseColor,
      );
    }
  });

  it('线宽必须是有限数并 clamp 到 [1,4]', () => {
    expect(sanitizeDisplay({ gridCoarseWidth: 2.5 }).gridCoarseWidth).toBe(2.5);
    expect(sanitizeDisplay({ gridCoarseWidth: 99 }).gridCoarseWidth).toBe(GRID_WIDTH_RANGE[1]);
    expect(sanitizeDisplay({ gridFineWidth: -5 }).gridFineWidth).toBe(GRID_WIDTH_RANGE[0]);
    for (const bad of ['x', NaN, Infinity, null]) {
      expect(sanitizeDisplay({ gridFineWidth: bad }).gridFineWidth).toBe(
        DEFAULT_DISPLAY.gridFineWidth,
      );
    }
  });

  it('默认值本身合法：DEFAULT_DISPLAY 的线宽在范围内、颜色是合法 hex', () => {
    expect(DEFAULT_DISPLAY.gridCoarseWidth).toBeGreaterThanOrEqual(GRID_WIDTH_RANGE[0]);
    expect(DEFAULT_DISPLAY.gridFineWidth).toBeLessThanOrEqual(GRID_WIDTH_RANGE[1]);
    expect(DEFAULT_DISPLAY.gridCoarseColor).toMatch(/^#[0-9a-f]{6}$/i);
    expect(DEFAULT_DISPLAY.gridFineColor).toMatch(/^#[0-9a-f]{6}$/i);
  });

  it('lampTrack（显示灯移动轨道）走布尔白名单，默认 true', () => {
    expect(DEFAULT_DISPLAY.lampTrack).toBe(true);
    expect(sanitizeDisplay({ lampTrack: false }).lampTrack).toBe(false);
    expect(sanitizeDisplay({ lampTrack: true }).lampTrack).toBe(true);
    for (const bad of ['yes', 1, null, {}]) {
      expect(sanitizeDisplay({ lampTrack: bad }).lampTrack).toBe(DEFAULT_DISPLAY.lampTrack);
    }
    // 旧存档（v1 字段集）自动补 lampTrack
    expect(sanitizeDisplay({ trail: false, comb: true }).lampTrack).toBe(
      DEFAULT_DISPLAY.lampTrack,
    );
  });
});
