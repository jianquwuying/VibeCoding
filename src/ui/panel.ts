/**
 * 参数面板（Tweakpane v4）+ localStorage 持久化（带 schema 版本号）+ 性能读数。
 */

import { Pane } from 'tweakpane';

import {
  DEFAULT_DISPLAY,
  DEFAULT_PARAMS,
  GRID_WIDTH_RANGE,
  GRID_WIDTH_STEP,
  LAMP_SEGMENTS_MAX,
  LAMP_SEGMENTS_MIN,
  LAMP_SEGMENTS_STEP,
  LAMP_Y_RANGE_MM,
  LAMP_Y_STEP_MM,
  PARAM_RANGES,
  PLAYBACK_SPEED_DEFAULT,
  STORAGE_KEY,
  STORAGE_SCHEMA_VERSION,
  UAV_MAX_ACCEL_MPS2,
  UAV_MAX_SPEED_MPS,
  sanitizeDisplay,
  type DisplayOptions,
  type SimParams,
} from '../config';

export interface PersistedState {
  params: SimParams;
  display: DisplayOptions;
  speed: number;
}

export interface PanelCallbacks {
  onParamsChange(params: SimParams): void;
  onDisplayChange(display: DisplayOptions): void;
  onPrecisionDemo(segments: number): void;
  onResetAll(): void;
}

export interface PanelView {
  pane: Pane;
  setPerf(simMs: number, frameMs: number): void;
  setPrecisionDelta(text: string): void;
  refresh(): void;
  setParams(params: SimParams): void;
  /** 以部分字段更新显示选项（会走完整联动：面板刷新 → 主场景应用 → 落盘） */
  setDisplay(partial: Partial<DisplayOptions>): void;
  getParams(): SimParams;
  getDisplay(): DisplayOptions;
  dispose(): void;
}

function clampToRange(key: keyof typeof PARAM_RANGES, value: number): number {
  const [lo, hi] = PARAM_RANGES[key];
  return Math.min(Math.max(value, lo), hi);
}

/** 参数净化：逐字段 clamp 到 `PARAM_RANGES`，非有限数回落默认（导出以便单测）。 */
export function sanitizeParams(raw: Partial<SimParams> | undefined): SimParams {
  const merged: SimParams = { ...DEFAULT_PARAMS, ...(raw ?? {}) };
  const out: SimParams = { ...merged };
  for (const key of Object.keys(PARAM_RANGES) as Array<keyof typeof PARAM_RANGES>) {
    const v = merged[key];
    out[key] = Number.isFinite(v) ? clampToRange(key, v) : DEFAULT_PARAMS[key];
  }
  out.lampSegments = Math.min(
    Math.max(
      Math.round(merged.lampSegments / LAMP_SEGMENTS_STEP) * LAMP_SEGMENTS_STEP,
      LAMP_SEGMENTS_MIN,
    ),
    LAMP_SEGMENTS_MAX,
  );
  return out;
}

/** 读取持久化参数；schema 版本不匹配则丢弃并回落默认值。 */
export function loadPersisted(): PersistedState {
  const fallback: PersistedState = {
    params: { ...DEFAULT_PARAMS },
    display: { ...DEFAULT_DISPLAY },
    speed: PLAYBACK_SPEED_DEFAULT,
  };
  try {
    const raw = window.localStorage.getItem(STORAGE_KEY);
    if (!raw) return fallback;
    const parsed = JSON.parse(raw) as {
      version?: number;
      params?: Partial<SimParams>;
      display?: Partial<DisplayOptions>;
      speed?: number;
    };
    if (parsed.version !== STORAGE_SCHEMA_VERSION) {
      // eslint-disable-next-line no-console
      console.warn(
        `[params] localStorage schema 版本不匹配（${String(parsed.version)} ≠ ${STORAGE_SCHEMA_VERSION}），已回落默认值`,
      );
      return fallback;
    }
    return {
      params: sanitizeParams(parsed.params),
      // 旧存档缺少新增显示字段（如网格 4 项）时自动补默认值，不丢参数
      display: sanitizeDisplay(parsed.display),
      speed:
        typeof parsed.speed === 'number' && Number.isFinite(parsed.speed)
          ? parsed.speed
          : PLAYBACK_SPEED_DEFAULT,
    };
  } catch (error) {
    // eslint-disable-next-line no-console
    console.warn('[params] 读取 localStorage 失败，使用默认值', error);
    return fallback;
  }
}

/** 写入持久化参数（带 schema 版本号）。 */
export function savePersisted(state: PersistedState): void {
  try {
    window.localStorage.setItem(
      STORAGE_KEY,
      JSON.stringify({ version: STORAGE_SCHEMA_VERSION, ...state }),
    );
  } catch (error) {
    // eslint-disable-next-line no-console
    console.warn('[params] 写入 localStorage 失败', error);
  }
}

export function clearPersisted(): void {
  try {
    window.localStorage.removeItem(STORAGE_KEY);
  } catch (error) {
    // eslint-disable-next-line no-console
    console.warn('[params] 清除 localStorage 失败', error);
  }
}

export function createPanel(
  container: HTMLElement,
  initial: PersistedState,
  callbacks: PanelCallbacks,
): PanelView {
  const params: SimParams = { ...initial.params };
  const display: DisplayOptions = { ...initial.display };
  const readonlyState = {
    maxSpeed: UAV_MAX_SPEED_MPS,
    maxAccel: UAV_MAX_ACCEL_MPS2,
    resolution: '640×480',
    simMs: '—',
    precisionDelta: '—',
  };

  const pane = new Pane({ container, title: '参数面板', expanded: true });

  const emitParams = (): void => callbacks.onParamsChange({ ...params });
  const emitDisplay = (): void => callbacks.onDisplayChange({ ...display });

  // ---- 机体高度警告（v3，panel 内部自足：不改 PanelCallbacks / main.ts） -----
  //
  // 元素在所有 Tweakpane binding 建完后 append 到 container（= #panel-host）末尾，
  // 因此它显示在面板**底部**。updateUavWarning() 只改 hidden，不碰 Tweakpane，
  // 所以**不调用** pane.refresh()。
  const UAV_HEIGHT_WARN_MM = 55;
  let warnEl: HTMLDivElement | null = null;
  const updateUavWarning = (): void => {
    if (!warnEl) return;
    warnEl.hidden = params.uavSizeZMm >= UAV_HEIGHT_WARN_MM;
  };

  /** 把 target 的指定字段恢复为 defaults 的对应值（供每组具名重置按钮使用）。 */
  const applyDefaults = <T extends object>(target: T, defaults: T, keys: Array<keyof T>): void => {
    for (const key of keys) target[key] = defaults[key];
  };
  const resetParams = (keys: Array<keyof SimParams>): void => {
    applyDefaults(params, DEFAULT_PARAMS, keys);
    pane.refresh();
    emitParams();
  };

  // ---- 弹道参数 ---------------------------------------------------------
  const dartFolder = pane.addFolder({ title: '弹道参数', expanded: true });
  dartFolder
    .addBinding(params, 'v0', { min: 15, max: 30, step: 0.1, label: '初始速度 [m/s]' })
    .on('change', emitParams);
  dartFolder
    .addBinding(params, 'theta0Deg', { min: 15, max: 45, step: 0.1, label: '初始仰角 [°]' })
    .on('change', emitParams);
  dartFolder
    .addBinding(params, 'speedDecayPerS', {
      min: 0,
      max: 1.5,
      step: 0.01,
      label: '速度衰减率 [1/s]',
    })
    .on('change', emitParams);
  dartFolder
    .addButton({ title: '重置弹道' })
    .on('click', () => resetParams(['v0', 'theta0Deg', 'speedDecayPerS']));

  // ---- 悬停站位 ----------------------------------------------------------
  const hoverFolder = pane.addFolder({ title: '悬停站位', expanded: true });
  hoverFolder
    .addBinding(params, 'standoffM', {
      min: 0.05,
      max: 0.3,
      step: 0.005,
      label: '距灯平面距离 [m]',
    })
    .on('change', emitParams);
  hoverFolder
    .addBinding(params, 'offsetVM', {
      min: -0.1,
      max: 0.1,
      step: 0.005,
      label: '竖直方向偏置 [m]',
    })
    .on('change', emitParams);
  hoverFolder
    .addBinding(params, 'offsetUM', {
      min: -0.1,
      max: 0.1,
      step: 0.005,
      label: '水平方向偏置 [m]',
    })
    .on('change', emitParams);
  hoverFolder
    .addButton({ title: '重置悬停站位' })
    .on('click', () => resetParams(['standoffM', 'offsetUM', 'offsetVM']));

  // ---- 目标灯位置 --------------------------------------------------------
  const lampPosFolder = pane.addFolder({ title: '目标灯位置', expanded: true });
  lampPosFolder
    .addBinding(params, 'lampTargetYM', {
      min: LAMP_Y_RANGE_MM[0] / 1000,
      max: LAMP_Y_RANGE_MM[1] / 1000,
      step: LAMP_Y_STEP_MM / 1000,
      label: '灯 Y 偏移 [m]',
    })
    .on('change', emitParams);
  lampPosFolder
    .addButton({ title: '重置灯位置' })
    .on('click', () => resetParams(['lampTargetYM']));

  // ---- 无人机 -----------------------------------------------------------
  const uavFolder = pane.addFolder({ title: '无人机', expanded: false });
  // v3：三轴独立（长=X / 宽=Y / 高=Z），三个滑块互不联动
  uavFolder
    .addBinding(params, 'uavSizeXMm', { min: 50, max: 200, step: 1, label: '机体长度 [mm]' })
    .on('change', emitParams);
  uavFolder
    .addBinding(params, 'uavSizeYMm', { min: 50, max: 200, step: 1, label: '机体宽度 [mm]' })
    .on('change', emitParams);
  uavFolder
    .addBinding(params, 'uavSizeZMm', { min: 50, max: 200, step: 1, label: '机体高度 [mm]' })
    .on('change', () => {
      updateUavWarning();
      emitParams();
    });
  uavFolder.addBinding(readonlyState, 'maxSpeed', { readonly: true, label: '最大速度 [m/s]' });
  uavFolder.addBinding(readonlyState, 'maxAccel', {
    readonly: true,
    label: '最大加速度 [m/s²]',
  });
  uavFolder.addButton({ title: '重置无人机' }).on('click', () => {
    resetParams(['uavSizeXMm', 'uavSizeYMm', 'uavSizeZMm']);
    updateUavWarning();
  });

  // ---- 灯球 -------------------------------------------------------------
  const lampFolder = pane.addFolder({ title: '灯球', expanded: false });
  lampFolder
    .addBinding(params, 'lampDiameterMm', { min: 30, max: 100, step: 1, label: '灯球直径 [mm]' })
    .on('change', emitParams);
  lampFolder
    .addBinding(params, 'lampSegments', {
      min: LAMP_SEGMENTS_MIN,
      max: LAMP_SEGMENTS_MAX,
      step: LAMP_SEGMENTS_STEP,
      label: '灯球多边形精度',
    })
    .on('change', emitParams);
  lampFolder.addButton({ title: '精度演示' }).on('click', () => {
    params.lampSegments =
      params.lampSegments >= LAMP_SEGMENTS_MAX ? LAMP_SEGMENTS_MIN : LAMP_SEGMENTS_MAX;
    pane.refresh();
    callbacks.onPrecisionDemo(params.lampSegments);
    emitParams();
  });
  lampFolder.addBinding(readonlyState, 'precisionDelta', {
    readonly: true,
    label: 'ΔR（对比 1024 边形）',
  });
  lampFolder
    .addButton({ title: '重置灯' })
    .on('click', () => resetParams(['lampDiameterMm', 'lampSegments']));

  // ---- 相机 -------------------------------------------------------------
  const cameraFolder = pane.addFolder({ title: '相机', expanded: false });
  cameraFolder
    .addBinding(params, 'hfovDeg', { min: 30, max: 120, step: 1, label: '水平视场角 [°]' })
    .on('change', emitParams);
  cameraFolder.addBinding(readonlyState, 'resolution', { readonly: true, label: '分辨率（只读）' });
  cameraFolder.addButton({ title: '重置相机' }).on('click', () => resetParams(['hfovDeg']));

  // ---- 仿真 -------------------------------------------------------------
  const simFolder = pane.addFolder({ title: '仿真', expanded: false });
  simFolder
    .addBinding(params, 'dt', { min: 0.005, max: 0.05, step: 0.005, label: '仿真步长 [s]' })
    .on('change', emitParams);
  simFolder
    .addBinding(params, 'rTh', { min: 0.3, max: 0.95, step: 0.01, label: '遮挡分析阈值' })
    .on('change', emitParams);
  simFolder
    .addButton({ title: '重置仿真' })
    .on('click', () => resetParams(['dt', 'rTh']));

  // ---- 性能 -------------------------------------------------------------
  const perfFolder = pane.addFolder({ title: '性能', expanded: false });
  perfFolder.addBinding(readonlyState, 'simMs', { readonly: true, label: '仿真耗时' });

  // ---- 显示选项 ---------------------------------------------------------
  const displayFolder = pane.addFolder({ title: '显示选项', expanded: false });
  for (const [key, label] of [
    ['trail', '显示弹道拖尾'],
    ['shadow', '显示阴影多边形'],
    ['frustum', '显示相机视锥'],
    ['comb', '显示光轴梳齿'],
    ['fpv', '显示 FPV 小窗'],
    ['curve', '显示遮挡率曲线'],
    ['lampTrack', '显示灯移动轨道'],
  ] as Array<[keyof DisplayOptions, string]>) {
    displayFolder.addBinding(display, key, { label }).on('change', emitDisplay);
  }
  displayFolder
    .addBinding(display, 'gridPlane', {
      label: '网格平面',
      options: {
        '灯球平面 (X=0)': 'lamp',
        '水平面 (Z=0)': 'horizontal',
        '两者都画': 'both',
      },
    })
    .on('change', emitDisplay);
  const showCoarseBinding = displayFolder.addBinding(display, 'showCoarse', {
    label: '显示粗网格',
  });
  const showFineBinding = displayFolder.addBinding(display, 'showFine', { label: '显示细网格' });
  displayFolder.addBinding(display, 'showAxes', { label: '显示坐标轴' }).on('change', emitDisplay);

  // ---- 网格样式（子分组） ------------------------------------------------
  const gridStyleFolder = displayFolder.addFolder({ title: '网格样式', expanded: false });
  // 颜色：绑定 hex 字符串即自动使用 Tweakpane 的颜色选择器
  // （本项目不使用 color:{type}，那是给 {r,g,b} 对象/数值分量绑定用的）
  gridStyleFolder
    .addBinding(display, 'gridCoarseColor', { label: '粗网格颜色', picker: 'popup' })
    .on('change', emitDisplay);
  const coarseWidthBinding = gridStyleFolder
    .addBinding(display, 'gridCoarseWidth', {
      label: '粗网格线宽 [px]',
      min: GRID_WIDTH_RANGE[0],
      max: GRID_WIDTH_RANGE[1],
      step: GRID_WIDTH_STEP,
    })
    .on('change', emitDisplay);
  gridStyleFolder
    .addBinding(display, 'gridFineColor', { label: '细网格颜色', picker: 'popup' })
    .on('change', emitDisplay);
  const fineWidthBinding = gridStyleFolder
    .addBinding(display, 'gridFineWidth', {
      label: '细网格线宽 [px]',
      min: GRID_WIDTH_RANGE[0],
      max: GRID_WIDTH_RANGE[1],
      step: GRID_WIDTH_STEP,
    })
    .on('change', emitDisplay);

  /** 线宽控件只在对应网格显示时可用（避免无效控件观感）。 */
  const syncWidthDisabled = (): void => {
    coarseWidthBinding.disabled = !display.showCoarse;
    fineWidthBinding.disabled = !display.showFine;
  };
  showCoarseBinding.on('change', () => {
    syncWidthDisabled();
    emitDisplay();
  });
  showFineBinding.on('change', () => {
    syncWidthDisabled();
    emitDisplay();
  });
  syncWidthDisabled();

  displayFolder.addButton({ title: '重置显示' }).on('click', () => {
    Object.assign(display, DEFAULT_DISPLAY);
    pane.refresh();
    syncWidthDisabled();
    emitDisplay();
  });

  pane.addButton({ title: '重置所有参数' }).on('click', () => {
    Object.assign(params, DEFAULT_PARAMS);
    Object.assign(display, DEFAULT_DISPLAY);
    pane.refresh();
    syncWidthDisabled();
    updateUavWarning();
    callbacks.onResetAll();
  });

  // 警告元素：放在面板底部（pane DOM 之后）
  warnEl = document.createElement('div');
  warnEl.id = 'uav-warning';
  warnEl.textContent = `⚠ 高度 < ${UAV_HEIGHT_WARN_MM} mm 时，遮挡策略可能失效`;
  warnEl.hidden = true;
  container.appendChild(warnEl);
  updateUavWarning();

  return {
    pane,
    setPerf: (simMs, frameMs) => {
      readonlyState.simMs = `${simMs.toFixed(1)} ms（60fps 预算 ${frameMs.toFixed(1)} ms）`;
      pane.refresh();
    },
    setPrecisionDelta: (text) => {
      readonlyState.precisionDelta = text;
      pane.refresh();
    },
    refresh: () => pane.refresh(),
    setParams: (next) => {
      Object.assign(params, next);
      pane.refresh();
      updateUavWarning();
    },
    setDisplay: (partial) => {
      Object.assign(display, partial);
      pane.refresh();
      syncWidthDisabled();
      emitDisplay();
    },
    getParams: () => ({ ...params }),
    getDisplay: () => ({ ...display }),
    dispose: () => {
      // pane.dispose() 不会移除手动 append 的元素，必须显式清理
      warnEl?.remove();
      warnEl = null;
      pane.dispose();
    },
  };
}
