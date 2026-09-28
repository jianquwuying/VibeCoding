# 无人机遮挡绿色引导灯 · Web 仿真器

把 Python 参考实现的几何遮挡仿真改写为**纯前端**应用：
实时调参、3D 动效、三层 HUD、R_occ 曲线、真 MP4 导出。
**弹道是 M2 数值积分模型**（重力 + 线性阻力 + 视线追击 + 最大转弯率限幅），
灯是**球体**（直径 55 mm）并沿 Y 在 ±240 mm 内平移。
**Python（`tools/python_ref/`，vendored 副本）是数值唯一真值来源**，
Web 端逐点复现并用黄金基准自动比对（`R_occ` ≤ 1e-6）。

> 本文档同时是**可复原规格**：只把它交给一个编码 AI，按 §4 → §12 的顺序即可重建整个项目。
> §5~§10 列出的常量、公式、接口与 DOM 契约是"必须逐字对齐"的部分；§11 给出可自证的测试与期望值。
> §13 附录额外给出 `index.html` 与 `src/style.css` 的**逐字内容**（这两处细节无法用契约完整描述）。

## 版本历史

- **v1**：飞镖 → 静止绿灯，无人机定点悬停，平面圆盘灯，解析抛物线弹道。
- **v2**：灯横向平移 + 制导飞镖 + 无人机实时追踪；**球体灯**；**M2 六步数值模型**
  （重力 + 线性阻力 + 视线追击 + `ω_max` 限幅）；灯移动轨道、`losBasis`、`LampFrame`。
- **v3**：**无人机三轴独立可调**（长=X / 宽=Y / 高=Z，默认 `(100, 100, 80)` mm，范围 [50,200]）；
  黄金基准扩为 8 组；持久化升 v3。**代价**：默认机体变小 → **默认不再完全遮挡**
  （`R_min = 0.974843`、`minMargin = −3.36 mm`，见 §11.3）。

---

## 0. 快速开始

环境：Node.js ≥ 18（实测 v24.19.0）、现代 Chrome / Edge。无后端、无 CDN。

```bash
npm install
npm run dev        # http://localhost:5173
```

| 命令 | 作用 |
|---|---|
| `npm run dev` | 开发服务器 |
| `npm run build` | `tsc --noEmit` + `vite build` |
| `npm run preview` | 预览生产构建（同样带 COOP/COEP 头） |
| `npm run test` | vitest，129 项：数值一致性 / 运动学 / 遮挡余量 / 三轴机体 / 网格 / 坐标轴 / 布局 / 裁剪 / 性能 |
| `npm run golden` | 重新生成黄金基准（需本机 Python + numpy，走 `tools/python_ref/`） |

**导出 MP4 的硬前置条件**：ffmpeg.wasm 依赖 `SharedArrayBuffer`，而它只在跨源隔离文档中可用。
`vite.config.ts` 已为 dev 与 preview 默认开启 `Cross-Origin-Embedder-Policy: require-corp` 与
`Cross-Origin-Opener-Policy: same-origin`；把 `dist/` 托管到别处时必须自行补这两个响应头。
导出前会自检 `self.crossOriginIsolated`，不满足直接报错。

---

## 1. 目标与硬性约束

**场景（M2）**：飞镖相机以 `v0 = 20 m/s`、`θ0 = 30°` 射出，弹道段受重力与线性阻力；
一旦"完整看到 55 mm 灯球"（J2 判据）就按 `ω_max = 60°/s` 的转弯率上限**逐帧**朝灯心追击。
灯在 `t = 1.2 ~ 1.8 s` 从 `Y = 0` 平移到 `lampTargetYM`（±240 mm），
无人机（**三轴独立长方体** 100(X) × 100(Y) × 80(Z) mm，长=X / 宽=Y / 高=Z；0.15 m 站位）
以最大 5 m/s / 2 m/s² 的时间最优规划追踪灯的位置。
全程计算几何遮挡率 `R_occ`（无人机阴影凸包 ∩ 灯球投影 ÷ 灯球面积）。

**必须遵守**

- 技术栈锁定：TypeScript + Vite + Three.js（**纯 WebGL，不用 WebGPU**）+ Tweakpane v4 + uPlot +
  ffmpeg.wasm；原生 TS + DOM，**不用 React/Vue**；全部依赖走 npm，**不用 CDN**；无后端。
- 数值核心（`src/core/**`）必须与 Python 逐点一致：`R_occ` 容差 **1e-6**、位置/速度容差 **1e-9**、
  帧数 / 末帧时刻 / `tLock` / `tHit` / `hitLamp` / `locked` / `valid` 严格一致。
- 单位：**1 m = 1 Three.js unit**（不缩放）。相机 `near 0.01` / `far 100`。
- 坐标系：以绿灯几何中心为原点；`X` 沿飞行方向（弹道沿 +X 前进）、`Y` 场地横向、`Z` 竖直向上。
  灯球在 **X = 0** 平面内沿 Y 平移；该平面的法线 **(-1, 0, 0)**（朝向迎面而来的飞镖）。
- 渲染层**只读** `SimResult`：动画循环内不做几何/仿真计算；参数变化才重算仿真（每动画帧最多一次）。

---

## 2. 技术栈与工程配置

### 2.1 `package.json`

```jsonc
{
  "name": "rm-uav-occlusion-web",
  "private": true,
  "type": "module",
  "scripts": {
    "dev": "vite",
    "build": "tsc --noEmit && vite build",
    "preview": "vite preview",
    "test": "vitest run",
    "test:watch": "vitest",
    "golden": "python tools/gen_golden.py"
  },
  "dependencies": {
    "@ffmpeg/core": "^0.12.6", "@ffmpeg/ffmpeg": "^0.12.10", "@ffmpeg/util": "^0.12.1",
    "three": "^0.169.0", "tweakpane": "^4.0.5", "uplot": "^1.6.31"
  },
  "devDependencies": {
    "@tweakpane/core": "^2.0.0", "@types/three": "^0.169.0",
    "polygon-clipping": "^0.15.7", "typescript": "^5.6.3", "vite": "^5.4.10", "vitest": "^2.1.8"
  }
}
```

> `@tweakpane/core` 是**类型依赖**：`tweakpane` 的 `.d.ts` 从它继承 `addFolder/addBinding/disabled` 等成员，
> 不装则 TS 报 "Property does not exist"。`three` 不自带类型，需 `@types/three`。

### 2.2 `tsconfig.json`

```jsonc
{
  "compilerOptions": {
    "target": "ES2022", "lib": ["ES2022", "DOM", "DOM.Iterable"],
    "module": "ESNext", "moduleResolution": "bundler", "types": ["vite/client"],
    "strict": true, "noUnusedLocals": true, "noUnusedParameters": true,
    "noImplicitOverride": true, "noFallthroughCasesInSwitch": true,
    "exactOptionalPropertyTypes": false, "isolatedModules": true, "esModuleInterop": true,
    "skipLibCheck": true, "resolveJsonModule": true, "verbatimModuleSyntax": true, "noEmit": true
  },
  "include": ["src", "tests", "vite.config.ts"]
}
```

### 2.3 `vite.config.ts`

```ts
import { defineConfig } from 'vitest/config';

const crossOriginIsolationHeaders = {
  'Cross-Origin-Embedder-Policy': 'require-corp',
  'Cross-Origin-Opener-Policy': 'same-origin',
};

export default defineConfig({
  base: './',
  // @ffmpeg/ffmpeg 内部用 new Worker(new URL(...)) 加载 worker，被 Vite 预打包会失效
  optimizeDeps: { exclude: ['@ffmpeg/ffmpeg', '@ffmpeg/util'] },
  server: { headers: crossOriginIsolationHeaders },
  preview: { headers: crossOriginIsolationHeaders },
  build: { target: 'es2022', chunkSizeWarningLimit: 4096 },
  test: { include: ['tests/**/*.test.ts'], testTimeout: 120000 },
});
```

---

## 3. 目录结构

```
index.html            单页 + DOM 契约（见 §8.1）
package.json  tsconfig.json  vite.config.ts  .gitignore(node_modules/dist/.vite/smoke-*.png)
src/
  config.ts           顶部集中可调参数区 + 派生常量、SimParams/DisplayOptions、sanitizeDisplay（§4）
  core/
    geometry.ts       向量、平面基、losBasis、UAV 顶点线框、凸包、投影阴影、凸-凸交集（§5.2）
    motion.ts         M2 运动学：灯平移 / J2 锁定 / 六步积分 / 线段-点距离 / uav_step（§5.1）
    occlusion.ts      R_occ = 阴影凸包 ∩ 灯球多边形 ÷ 解析 πr² + 遮挡余量（§5.3）
    camera.ts         光轴=速度方向的位姿、针孔投影、视场余量、像面遮挡比（§5.4）
    simulate.ts       主仿真：M2 主循环 + valid 判据 + 线段-球命中 + FPV 预计算（§5.5）
  render/
    buildScene.ts     主场景静态元素（M2 弹道/灯球/灯轨道/UAV/视锥/梳齿/LOS/光轴/阴影）（§6.1）
    mainScene.ts      渲染器 + OrbitControls + 视角预设（target 随灯）+ 逐帧更新（§6.2）
    grid.ts           SceneGrid：粗/细网格 + 三色坐标轴（§6.3）
    fpvScene.ts       像平面 2D 场景（§6.4）
  ui/
    hudLayout.ts      屏幕/导出共用的归一化 HUD 布局（§7.2）
    curve.ts          uPlot 层 + Canvas2D 特效层（无效帧断线）（§7.3）
    timeline.ts       控制条 / 速度滑块 / 进度条 / 第 3 层文字（§7.4）
    panel.ts          Tweakpane 面板 + localStorage（§7.5/§9）
    fpvOverlay.ts     FPV 定位与悬停放大（§7.2）
  export/
    hud.ts            导出时的 Canvas2D 文字重绘（无效帧显示 "—"）（§8）
    mp4.ts            离屏渲染 + 合成 + ffmpeg.wasm 编码（§8）
  main.ts             入口：串联仿真/渲染/UI/导出，主循环 + 无命中告警（§7.6）
  style.css           深色主题布局
  selftest.ts         `?selftest=ffmpeg` 页面内自检（可选，开发用）
tools/
  gen_golden.py       M2 黄金基准生成器（7 组 case + 函数表 + 余量表，§10.2）
  python_ref/         **vendored Python 参考实现**（数值唯一真值来源，§2.4）
  cdp-probe.mjs       用 CDP 驱动无头浏览器做自检/截图（开发用）
tests/                见 §10.1（129 项）
```

### 3.1 `tools/python_ref/`（vendored 参考实现）

从上游 Python 项目复制并改造为 M2 的**唯一数值真值**，`../rm_uav_occlusion` 本身不再被导入：

```
config.py       DartConfig（M2：launch / v0_mps / theta0_deg / speed_decay_per_s）/
                LampConfig / CameraConfig / UAVConfig / SimConfig / HoverStationConfig
                + M2 常量（GRAVITY_MPS2 / MAX_TURN_RATE_DPS / HIT_DISTANCE_M / SIM_T_END_S /
                LAMP_Y_RANGE_MM / LAMP_MOVE_START_S / LAMP_MOVE_END_S / LAMP_SPHERE_RADIUS_M）
                （v1 抛物线链已删除，见 python_ref/README.md）
motion.py       lamp_y_at / lamp_vy_at / lamp_center_at / is_fully_visible / step_dart_m2 /
                segment_to_point_distance / uav_step（v1 的 dart_camera_* 已删除）
geometry.py     向量 / plane_basis / los_basis / uav_corners / project_uav_shadow / 凸-凸交集
occlusion.py    compute_occlusion(C, uav_center, uav_size, lamp_center, los_hat, lamp_polygon_2d, lamp_area_m2)
                + lamp_disc_polygon / occlusion_margin
camera.py       位姿 / 针孔投影 / fov 余量 / image_space_occlusion（基改为 plane_basis(los_hat)）；
                v1 的 camera_pose / in_fov 已删除
experiment.py   run_m2（M2 主循环的唯一 Python 实现）/ occlusion_margin / hover_station_position_at
strategies.py   上游策略模块（**保留但本流程不 import**，登记为"独立参考验证"用）
README.md       本副本与上游的差异清单（含"已删除"与"保留但未被 gen_golden 调用"两张表）
```

> **一致性要求**：`config.py` / `motion.py` / `simulate.ts` 的**分支顺序与表达式顺序**必须逐行一致，
> 否则 `tests/golden/tools_parity.json` 的 1e-12 断言会失败。改动任何一侧都必须重跑 `npm run golden`。

---

## 4. 数值契约：常量与默认值

### 4.1 机械装配数据（唯一录入点，单位 mm）

```ts
export const LAMP_CENTER_MECH_MM: Vec3 = [25037.05, 2868.18, 1139.04];
export const LAUNCH_POINT_MECH_MM: Vec3 = [0.0, 0.0, 570.0];
export const LAUNCH_REL_LAMP_M: Vec3 = [  // 逐分量 (launch - lamp)/1000
  (LAUNCH_POINT_MECH_MM[0] - LAMP_CENTER_MECH_MM[0]) / 1000,   // -25.03705
  (LAUNCH_POINT_MECH_MM[1] - LAMP_CENTER_MECH_MM[1]) / 1000,   //  -2.86818
  (LAUNCH_POINT_MECH_MM[2] - LAMP_CENTER_MECH_MM[2]) / 1000,   //  -0.56904
];
```

### 4.2 顶部集中可调参数区（`src/config.ts`，改这里即可）

文件顶部**只有这一处**定义默认值，其余位置不得重复定义；集中区下方是派生常量与类型。
改动任何数值后**必须**重跑 `npm run golden`，否则 parity 测试失败。

| 常量 | 值 | 说明 |
|---|---|---|
| `SIMPLE_V0_MPS` | **20.0** | 出膛速度 [m/s]，UI [15,30] |
| `SIMPLE_THETA0_DEG` | **30.0** | 初始仰角 [°]，UI [15,45] |
| `SIMPLE_SPEED_DECAY_PER_S` | **0.55** | 阻力系数 k [1/s]，UI [0,1.5] |
| `GRAVITY_MPS2` | 9.8 | 重力加速度 [m/s²] |
| `MAX_TURN_RATE_DPS` | **60.0** | 最大转弯率 ω_max [°/s]（**常量，无 UI 控件**） |
| `LAMP_Y_RANGE_MM` / `LAMP_Y_STEP_MM` | [-240, 240] / 1 | 灯 Y 物理限位 [mm] 与步长 |
| `LAMP_MOVE_START_S` / `LAMP_MOVE_END_S` | 1.2 / **1.8** | 灯开始移动 / 到位时刻 [s] |
| `LAMP_DIAMETER_MM` / `LAMP_SPHERE_RADIUS_M` | 55.0 / 0.0275 | 灯球直径 / 半径 |
| `SIM_T_END_S` | **5.0** | 仿真时间上限 [s]（命中延后） |
| `SIM_DT_S` | 0.01 | 仿真步长 [s] |
| `HIT_DISTANCE_M` | 0.05 | 命中判定距离 [m]（线段到灯心） |
| `MIN_FRAMES` | 2 | 最小帧数 |
| `UAV_MAX_SPEED_MPS` / `UAV_MAX_ACCEL_MPS2` | 5.0 / 2.0 | 无人机运动学上限（只读显示） |
| `UAV_SIZE_X_MM` | **100.0** | 机体 X 轴（沿飞行方向）长度 [mm]，UI [50,200] |
| `UAV_SIZE_Y_MM` | **100.0** | 机体 Y 轴（场地横向）宽度 [mm]，UI [50,200] |
| `UAV_SIZE_Z_MM` | **80.0** | 机体 Z 轴（竖直）高度 [mm]，UI [50,200]；**< 55 mm 时遮挡策略可能失效**（UI 红字警告） |
| `STORAGE_KEY` / `STORAGE_SCHEMA_VERSION` | `'rm-uav-occlusion-params-v3'` / **3** | 持久化 |

### 4.3 派生常量（`src/config.ts` 下半部）

| 常量 | 值 | 说明 |
|---|---|---|
| `LAMP_CENTER` / `LAMP_NORMAL` | (0,0,0) / (-1,0,0) | 灯球基准位置与所在平面法线（网格用） |
| `LAMP_SEGMENTS_DEFAULT/MIN/MAX/STEP` | 1024 / 64 / 1024 / 64 | 灯球多边形逼近段数 |
| `DEFAULT_UAV_BOUNDS` | x[-26,0.5] y[-6,6] z[-2,3] | 机身允许占据的包围盒 [m]；机体中心安全边界 = 各轴 ∓ 该轴半尺寸（v3 逐轴） |
| `CAMERA_WIDTH/HEIGHT/HFOV_DEG` / `CAMERA_UP_REF` | 640 / 480 / 70.0 / (0,0,1) | 相机内参；光轴 = 速度方向，此向量只定滚转 |
| `IMAGE_SPACE_SEGMENTS` | 128 | FPV 灯球投影段数 |
| `HOVER_STATION_STANDOFF_M` / `_OFFSET_U_M` / `_OFFSET_V_M` | 0.15 / 0.0 / 0.0 | 悬停站位 |
| `R_TH_DEFAULT` / `R_TH_MIN` / `R_TH_MAX` | 0.7 / 0.3 / 0.95 | 遮挡分析阈值 |
| `PLAYBACK_SPEED_DEFAULT/MIN/MAX/SNAPS/SNAP_EPS` | 0.5 / 0.25 / 4.0 / [0.25,0.5,1,2,4] / 0.05 | 播放 |
| `EXPORT_FPS/EXPORT_WIDTH/EXPORT_HEIGHT/EXPORT_BITRATE_KBPS/EXPORT_SPEED` | 60 / 1920 / 1080 / 4000 / 0.5 | 导出 |
| `EXPORT_MEMORY_BUDGET_BYTES` | 120·1024·1024 | PNG 预算，超限降级 JPEG |
| `FPV_HOVER_SCALE` / `FPV_HOVER_MAX_HEIGHT_FRACTION` | 1.5 / 0.6 | FPV 悬停放大（基准×1.5、上限视口高 60%） |
| `AXIS_COLORS` | x `0xef4444`、y `0x22c55e`、z `0x3b82f6` | 坐标轴唯一色源（渲染与测试共用） |
| `GRID_COARSE_OPACITY` / `GRID_FINE_OPACITY` | 0.9 / 0.7 | 网格不透明度 |
| `GRID_WIDTH_RANGE` / `GRID_WIDTH_STEP` | [1,4] / 0.5 | 网格线宽范围 [px] 与步长 |
| `COLORS` | bg `#0f172a`、primary `#3b82f6`、glow `rgba(59,130,246,.25)`、threshold `#ef4444`、effective `#10b981`、fill `rgba(16,185,129,.35)`→`0`、text `#f8fafc`、accent `#f59e0b`、grid `#1e293b` | HUD 配色 |

### 4.4 参数类型与范围

```ts
export interface SimParams {
  v0: number; theta0Deg: number; speedDecayPerS: number;      // 弹道 / 制导
  standoffM: number; offsetUM: number; offsetVM: number;      // 悬停站位
  uavSizeXMm: number; uavSizeYMm: number; uavSizeZMm: number; // 机体三轴 [mm]（长=X / 宽=Y / 高=Z）
  lampDiameterMm: number; lampSegments: number;               // 灯球
  hfovDeg: number; dt: number; rTh: number;                   // 相机 / 仿真
  lampTargetYM: number;                                       // 灯 Y 目标偏移 [m]，±0.24
}
export type GridPlane = 'lamp' | 'horizontal' | 'both';
export interface DisplayOptions {
  trail: boolean; shadow: boolean; frustum: boolean; comb: boolean; fpv: boolean; curve: boolean;
  lampTrack: boolean;                                         // 显示灯移动轨道
  gridPlane: GridPlane; showCoarse: boolean; showFine: boolean; showAxes: boolean;
  gridCoarseColor: string; gridFineColor: string; gridCoarseWidth: number; gridFineWidth: number;
}
```

`PARAM_RANGES`：`v0 [15,30]`、`theta0Deg [15,45]`、`speedDecayPerS [0,1.5]`、`standoffM [0.05,0.3]`、
`offsetUM/offsetVM [-0.1,0.1]`、`uavSizeXMm / uavSizeYMm / uavSizeZMm [50,200]`（三条独立、无联动）、
`lampDiameterMm [30,100]`、`hfovDeg [30,120]`、`dt [0.005,0.05]`、`rTh [0.3,0.95]`、
`lampTargetYM [-0.24,0.24]`。
（`stopDistanceM` 已随 v1 抛物线族删除。）

`DEFAULT_PARAMS` = 顺序（20 / 30 / 0.55 / 0.15 / 0 / 0 / **100 / 100 / 80** / 55 / 1024 / 70 / 0.01 / 0.7 / **0**）。

`DEFAULT_DISPLAY` = `trail,shadow,frustum,comb,fpv,curve` 全 `true`；`gridPlane 'horizontal'`；
`showCoarse true`、**`showFine true`**、`showAxes true`；`gridCoarseColor '#4a4a4a'`、
`gridFineColor '#383838'`、两个线宽 `1`、**`lampTrack true`**。

`sanitizeDisplay(raw?: unknown): DisplayOptions`（纯函数、无 DOM 依赖）：
顶层为 `null` / 非对象 / 数组 → 返回 `{...DEFAULT_DISPLAY}`（**新对象**，不是共享引用）；
布尔项（含 `lampTrack`）`typeof === 'boolean'` 否则回落默认；`gridPlane` 走三值白名单；
颜色需匹配 `/^#[0-9a-f]{6}$/i`；线宽需 `Number.isFinite` 并 clamp 到 `[1,4]`。

---

## 5. 核心算法契约（`src/core/**`）

> 以下公式与边界条件必须逐条实现，否则与 Python 的 1e-6 对齐会失败。所有计算用 float64。

### 5.1 运动学 `motion.ts`（M2）

> ⚠️ 本节所有函数与 `tools/python_ref/motion.py` **逐行对应**（分支顺序、表达式顺序都不许改）。
> v1 的解析抛物线族（`simplePointAt/simplePosition/simpleVelocity/simpleTable/simpleFlightTime/
> simplePathLength/effectiveG/sampleFullTrajectory`）**已全部删除**。

```ts
export function lampYAt(t: number, yTarget: number): number;
export function lampVyAt(t: number, yTarget: number): number;
export function lampCenterAt(t: number, yTarget: number): number[];
export function isFullyVisible(P, d, L, camera: CameraConfig): boolean;
export interface StepDartM2Input { P; V; L: readonly number[];
  dt: number; k: number; g: number; omegaMaxRad: number; camera: CameraConfig }
export interface StepDartM2Output { P_next: number[]; V_next: number[]; locked: boolean }
export function stepDartM2(input: StepDartM2Input): StepDartM2Output;
export function segmentToPointDistance(A, B, Q): number;
export function uavStep(y0, v0, y1, vMax, aMax, dt): { y: number; v: number };
```

**灯横向平移（分段线性）**

```
lamp_y_at(t, yT)  = 0                          t < 1.2
                  = yT·(t−1.2)/0.6             1.2 ≤ t < 1.8
                  = yT                         t ≥ 1.8
lamp_vy_at(t, yT) = yT/0.6  仅当 1.2 ≤ t < 1.8，否则 0
lampCenterAt(t)   = [0, lampYAt(t), 0]
```

**J2 锁定判据（"完整看到 55 mm 球"，每帧独立判定）**

```
1. pose = poseFromPositionAndAxis(P, d, CAMERA_UP_REF)
2. c    = toCameraFrame(pose, L)；若 c.z ≤ 1e-9 → false
3. f    = focalPx(camera) = (W/2)/tan(HFOV/2)
   u0   = W/2 + f·c.x/c.z        v0 = H/2 − f·c.y/c.z
4. r_px = f·LAMP_SPHERE_RADIUS_M / c.z          # 小孔近似（与严格式差 <1e-3 px）
5. return u0−r_px ≥ 0 && u0+r_px ≤ W && v0−r_px ≥ 0 && v0+r_px ≤ H
```

**飞镖六步积分 `step_dart_m2`（dt = 0.01 s）**

```
1. d_i = normalize(V_i)                        # |V_i| < 1e-12 时 d_i = (1,0,0) 兜底
2. locked_i = isFullyVisible(P_i, d_i, L(t_i)) # 每帧独立，无显式切换帧
3. a_phys = (0, 0, −g) + (−k·V_i)              # 重力 + 线性阻力
4. V_phys = V_i + a_phys·dt
5. 若 locked_i：
     v_mag  = |V_phys|
     d_aim  = normalize(L(t_i) − P_i)
     Δθ_max = ω_max·dt
     θ      = acos(clamp(dot(d_i, d_aim), −1, 1))
     θ ≤ Δθ_max → d_new = d_aim
     否则  axis = cross(d_i, d_aim)：
             |axis| < 1e-12 → d_new = d_i          # 平行/反平行，不转
             否则 k̂ = normalize(axis)；
                  d_new = d_i·cosΔθ_max + cross(k̂, d_i)·sinΔθ_max
     V_new = v_mag · d_new                     # 速率守恒（C4a）
   否则：
     V_new = V_phys                            # 弹道段不主动转向
6. P_{i+1} = P_i + V_new·dt ；V_{i+1} = V_new
```

**线段-点最近距离（命中判据）**

```
d = B − A；ddot = dot(d, d)
ddot < 1e-24 → |Q − A|
否则 t = clamp(dot(Q−A, d)/ddot, 0, 1)；返回 |A + t·d − Q|
```

**`uavStep` 一维时间最优规划（与 Python 逐行同序）**

```
输入 y0, v0, y1, vMax, aMax, dt ；输出 (y_next, v_next)
d = y1 − y0；sgn = sign(d)；xt = |d|

xt < 1e-12（已在目标）：
  |v0| ≤ aMax·dt → (y1, 0)
  否则 v_new = v0 − sign(v0)·aMax·dt；若符号翻转 → 0
       y_new = y0 + v0·dt − 0.5·sign(v0)·aMax·dt² 并 clamp 到 [y0, y1]

v = sgn·v0；x = 0
v < 0（正远离目标）：
  tStop = −v/aMax
  tStop ≥ dt → x_step = v·dt + 0.5·aMax·dt²；(y0 + sgn·x_step, sgn·(v + aMax·dt))
  否则 x += v·tStop + 0.5·aMax·tStop²；t_rem = dt − tStop；v = 0

vp² = aMax·(xt − x) + v²/2 ；vp = min(√max(vp²,0), vMax)
tA  = (vp − v)/aMax ；xA = (vp² − v²)/(2·aMax)
t_rem ≤ tA → x += v·t_rem + 0.5·aMax·t_rem²；(y0 + sgn·x, sgn·(v + aMax·t_rem))
t_rem −= tA；x += xA；v = vp

denom = max(vp, 1e-12)；xC = max(0, (xt − x − vp²/(2·aMax))/denom)
t_rem ≤ xC/denom → (y0 + sgn·(x + vp·t_rem), sgn·vp)
t_rem −= xC/denom；x += xC

x_rem = xt − x；disc = max(vp² − 2·aMax·x_rem, 0)；t3 = min((vp − √disc)/aMax, t_rem)
x += vp·t3 − 0.5·aMax·t3²；v = max(vp − aMax·t3, 0)
x > xt → x = xt, v = 0
返回 (y0 + sgn·x, sgn·v)
```

> 规划**无平滑**：稳态下 `uavVy` 会有 ±dt 量级的抖动（灯静止时为 0，移动时为常数）。

### 5.2 几何 `geometry.ts`

```ts
export const EPS = 1e-12;
export class GeometryError extends Error {}
export function asVector(v: readonly number[]): number[];          // 长度必须为 3
export function norm/scale/add/sub/dot/cross(...): ...;
export function normalize(v: readonly number[]): number[];          // 零向量抛 GeometryError
export function projectPointToPlane(C, P, planeCenter, planeNormal): number[] | null;
export function planeBasis(planeNormal): [number[], number[]];
export function planeLocalCoords(point, planeCenter, u, v): [number, number];
export function planeLocalToWorld(coords2d, planeCenter, u, v): number[];
export function uavCorners(center, size): number[][];               // 8 个顶点
export function uavEdges(): Array<[number, number]>;                // 12 条棱
export function allVerticesBetweenCameraAndPlane(C, corners, planeCenter, planeNormal, tol = 1e-9): boolean;
export function convexHull2D(points: readonly number[][]): number[][];
export function projectUavShadow(C, uavCenter, uavSize, planeCenter, planeNormal):
  { polygon2D: number[][]; points3D: number[][] };
export function losBasis(C, L): [number[], number[]];               // planeBasis(normalize(L−C))
export function lampPolygon(radiusM: number, segments: number): number[][];
export function signedArea(polygon): number;  export function polygonArea(polygon): number;
export interface Aabb { minX: number; minY: number; maxX: number; maxY: number }
export interface ConvexPolygon { points: number[][]; bounds: Aabb; aabbCorners: number[][]; area: number }
export function makeConvexPolygon(polygon: readonly number[][]): ConvexPolygon;
export function intersectConvexPolygons(a: ConvexPolygon, b: ConvexPolygon): { area: number; polygon: number[][] };
```

- `projectPointToPlane`：`dir = P−C`；`|dot(dir,n)| < EPS` → `null`；`t = dot(pc−C, n)/dot(dir, n)`；
  `t < 0` → `null`；否则 `C + t·dir`。
- `planeBasis(n)`：`ref = |n.z| < 0.9 ? (0,0,1) : (1,0,0)`；`u = normalize(cross(ref, n))`；
  `v = normalize(cross(n, u))`。对本场景 `n = (-1,0,0)` → **`u = (0,-1,0)`、`v = (0,0,1)`**。
- `losBasis(C, L) = planeBasis(normalize(L − C))`：**M2 的阴影平面基逐帧取它**
  （不是固定的灯面法线），因为灯在动、弹道在转弯，视线方向逐帧不同。
- `uavCorners`：`half = |size|/2`；按 `sx∈{−1,1} → sy∈{−1,1} → sz∈{−1,1}` 生成，
  **索引 `idx = 4a + 2b + c`**，因此 `idx^1 / idx^2 / idx^4` 必为相邻顶点。
- `allVerticesBetweenCameraAndPlane`：`dCam = dot(C−pc, n)`，`|dCam| ≤ tol` → false；
  任一顶点 `d·dCam ≤ 0` 或 `|d| > |dCam| + tol` → false；否则 true。
- `convexHull2D`：单调链，按 (x,y) 排序，去掉共线点，返回逆时针凸包；点数不足时返回退化点集。
- `projectUavShadow`：先做 `allVerticesBetweenCameraAndPlane`（不满足抛 `GeometryError`），
  再逐个顶点投影（`null` 抛错），转平面局部二维坐标后取凸包。
- `lampPolygon(r, segments)`：`n = max(8, floor(segments))`，顶点 `theta = −2πi/n`、`[r·cosθ, r·sinθ]`
  —— **从 (r,0) 起、顺时针**，与 shapely `Point(0,0).buffer(r, quad_segs=segments/4)` 一致。
  M2 里灯是球体，该多边形只作为"球在视线法向平面上的截面圆"的离散（§5.3）。
- `intersectConvexPolygons`：① 任一多边形 < 3 点 → 0；② AABB 不相交 → 0；
  ③ 快速接受：`a.aabbCorners` 全在 `b` 内 → 面积取 `a.area`；④ `b.bounds ⊆ a.bounds` 且
  `b.aabbCorners` 全在 `a` 内 → 面积取 `b.area`；⑤ 否则 **顶点多的一方作 subject、少的一方作裁剪窗口**，
  用 Sutherland–Hodgman 裁剪（`inside` 判据叉积 `≥ −1e-15`，交点分母 `|denom| < 1e-18` 时退化返回端点），
  结果 < 3 点 → 0。本场景两个多边形必然是凸的（UAV 阴影是 8 点凸包、灯球截面是正多边形）。

### 5.3 遮挡率 `occlusion.ts`

```ts
export interface LampFrame { center: readonly number[];   // L(t)
  losHat: readonly number[];                              // normalize(L − C)，逐帧
  radiusM: number;                                        // 0.0275
  areaM2: number;                                         // 解析 πr²
  polygon2D: ConvexPolygon }                              // 平面局部坐标的截面圆
export interface OcclusionResult { Rocc: number;
  shadowPolygon: number[][]; shadowPoints3D: number[][] }
export function computeOcclusion(C, uavCenter, uavSize, lamp: LampFrame): OcclusionResult;
export function occlusionMargin(shadowPolygon2D, lampRadiusM): number;
```

流程：`projectUavShadow(C, uavCenter, uavSize, lamp.center, lamp.losHat)` →
捕获 `GeometryError` / 凸包 < 3 点 / 面积 ≤ 0 / AABB 不相交 → 一律 `Rocc = 0`（几何无效与
"阴影没盖到灯"不区分；帧级"几何前提是否成立"由 `SimResult.valid` 单独表达）；
否则 `Rocc = clamp(面积 / lamp.areaM2, 0, 1)`，`lamp.areaM2 = π·r²`（**解析面积**，不是多边形面积）。

> **两种 0 的语义（实测无歧义）**：`projectUavShadow` 的几何前提与 `SimResult.valid` 用的是
> **同一个判据**（`allVerticesBetweenCameraAndPlane`），因此在 8 组 golden 的 1868 帧中
> 几何无效帧 **100% 落在 `valid = 0`**（该帧 `R = NaN`，`Rocc = 0` 被丢弃）；
> `valid = 1` 却几何无效/退化的情况 **0 次**（理论上只剩"射线与灯面平行"这一测度零退化子情形）。
> 下游只看到 `R = 0`（灯笼球完全没被遮住）或 `R = NaN`（该帧无效），不存在"0 分不清来历"的实际歧义。
分母用解析 πr²、分子用 1024 边形交集，因此"完全遮挡"时 `R_occ ≈ 0.999994` 而不是 1。

**遮挡余量 `occlusionMargin`（S3.5 的独立判据）**

```
margin = min_k dist(灯心, 阴影多边形的第 k 条边) − r      # r = 0.0275 m
margin > 0 ⟹ 灯球完全落在阴影里
margin < 0 ⟹ 灯球必然露出一条缝，R_occ < 1（两者互为独立验证）
```

`runSimulation` 逐帧取 `marginMin = min margin`，并记录 `marginMinT`；
`tests/occlusionMargin.test.ts` 用它固化"默认参数 +18.1 mm / (22,35) −15.1 mm"两条事实。

### 5.4 相机 `camera.ts`

```ts
export interface CameraConfig { width: number; height: number; fovHDeg: number }
export interface CameraPose { position: number[]; forward: number[]; right: number[]; up: number[] }
export function fovHRad(camera): number; export function cameraAspect(camera): number;   // W/H
export function focalPx(camera): number;                                                 // (W/2)/tan(HFOV/2)
export function poseFromPositionAndAxis(position, axis, upRef): CameraPose;
export function toCameraFrame(pose, point): number[];        // [dot(d,right), dot(d,up), dot(d,forward)]
export function fovMarginDeg(pose, point, camera): number;
export function projectPixel(pose, point, camera): [number, number] | null;
export function projectCirclePolygon(pose, center, radiusM, basisU, basisV, camera, nSegments): number[][] | null;
export function imageSpaceOcclusion(pose, lamp: { center; radiusM }, basisU, basisV,
  uavCenter, uavSize, camera, nSegments): ImageSpaceResult;   // ImageSpaceResult = { ratio }
```

`fovHRad` / `cameraAspect` / `fovVDeg` / `projectPointsHull` 是**模块内部**函数（不导出）；
`offAxisAngleDeg` 已删除（其唯一消费者 `SimResult.offAxis` 也一并删除）。

- `poseFromPositionAndAxis`：`forward = normalize(axis)`；`right = cross(forward, upRef)`，
  若 `|right| ≤ 1e-9` 依次退化到 `upRef = (0,1,0)`、`(1,0,0)`；`right = normalize(right)`；
  `up = normalize(cross(right, forward))`。**光轴严格等于弹道切线** `dC/dt`。
- `projectPixel`：`c = toCameraFrame`；`c.z ≤ 1e-9` → null；
  `u = W/2 + f·c.x/c.z`、`v = H/2 − f·c.y/c.z`（**v 向下**，与 canvas 一致）。
- `fovMarginDeg`：`c.z ≤ 1e-9` → `−90`；否则 `min(HFOV/2 − |atan2(c.x,c.z)|, VFOV/2 − |atan2(c.y,c.z)|)`（度）。
- `imageSpaceOcclusion`：把灯球截面圆周 128 点与 UAV 8 顶点分别投影取凸包，交集面积 ÷ 灯球像面凸包面积，
  clamp 到 [0,1]；任一投影不可见（有点在相机后方）→ ratio 0。
  **M2 的 `basisU/basisV` 改为逐帧 `planeBasis(los_hat)`**（`losBasis`），灯球的截面圆随之逐帧重建。
  返回值只有 `{ ratio }`（多边形由 FPV 场景从 `lampPx/uavPx` 自行重建）。

### 5.5 主仿真 `simulate.ts`

```ts
export interface SimResult {
  n: number; dt: number; tLast: number;
  t: Float64Array; R: Float64Array;                            // (n) 时间 / 遮挡率（无效帧 = NaN）
  Pd: Float64Array; Vd: Float64Array;                          // (3n) 飞镖位置 / 速度（M2 积分）
  uavPos: Float64Array; uavVy: Float64Array;                   // (3n)/(n) 无人机中心 / Y 速度
  lampY: Float64Array; lampPos: Float64Array;                  // (n)/(3n) 灯心
  locked: Uint8Array; valid: Uint8Array;                       // (n) 逐帧追踪段 / 几何前提
  tLock: number; hitLamp: boolean; tHit: number | null;        // 首次锁定 / 是否命中 / 命中时刻
  lampFrames: LampFrame[];                                     // 逐帧灯状态（含 losHat / polygon2D）
  lampPx: Float64Array; uavPx: Float64Array;                   // (128·2·n)/(8·2·n) FPV 投影像素，NaN=不可见
  imageR: Float64Array; fovMargin: Float64Array;
  shadow2D: Float64Array; shadowCount: Uint8Array; shadow3D: Float64Array;  // 阴影凸包(≤8点)与投影点
  poses: CameraPose[];
  lampRadiusM: number; lampAreaM2: number;                     // 灯球标量几何
  uavSizeM: number[];                                          // 机体三轴尺寸 [m]
  rTh: number; camera: CameraConfig;
  marginMin: number; marginMinT: number;                       // 遮挡余量最小值与时刻
  elapsedMs: number;
}
export function runSimulation(params: SimParams): SimResult;
export function frameIndexAt(result: SimResult, t: number): number;   // clamp(floor(t/dt), 0, n-1)
export function makeCameraConfig(params: SimParams): CameraConfig;
export function bodySafeBounds(sizeM: readonly number[]): { x: [number,number]; y: ...; z: ... };
export function hoverStationPositionAt(params: SimParams, lampY: number): number[];
export function pdAt/uavAt/lampAt(result: SimResult, i: number): number[];
```

`bodySafeBounds`：**每轴独立** `[lo+half_i, hi−half_i]`（`half_i = sizeM[i]/2`）；若反向则退化为中点。
`hoverStationPositionAt(params, lampY)`：（灯心移到 `(0,lampY,0)` 后的）
`L + standoff·n̂ + offsetU·û + offsetV·v̂`，再 clamp 到 `bodySafeBounds`；
对本场景（`n̂=(-1,0,0)`、`û=(0,-1,0)`、`v̂=(0,0,1)`）等价于 `(-standoff, lampY − offsetU, offsetV)`。
内部尺寸取 **三轴**：`uavSizeM = [uavSizeXMm, uavSizeYMm, uavSizeZMm] / 1000`（v3 起不再是立方体）。

**`runSimulation` 主循环（顺序不可变，与 Python `run_m2` 逐帧对应）**

```
初始化：nFull = round(5.0/dt)+1 = 501
        L0 = lampCenterAt(0, yT) = (0,0,0)
        ĥ0 = normalize([L0.x − P0.x, L0.y − P0.y, 0])
        θ0 = theta0Deg·π/180
        P  = LAUNCH_REL_LAMP_M = (−25.03705, −2.86818, −0.56904)
        V  = v0·(cosθ0·ĥ0 + sinθ0·ẑ)
        uavY = −offsetUM，uavV = 0，hitIndex = −1，tLock = −1

逐帧 i（t_i = i·dt）：
  1. L_i = lampCenterAt(t_i, yT)
  2. 无人机：t_i ≥ 1.2 s 且 i > 0 时，uavStep 追 lampYAt(t_i, yT) − offsetUM
     uavCenter = hoverStationPositionAt(params, uavY + offsetUM)
  3. 记录帧 i：t / Pd / Vd / lampPos / lampY / uavPos / uavVy
  4. losHat = normalize(L_i − P)；lampFrame = {center, losHat, radiusM, areaM2, polygon2D}
  5. occ = computeOcclusion(P, uavCenter, uavSizeM, lampFrame)
     valid_i = allVerticesBetweenCameraAndPlane(P, uavCorners(uavCenter), L_i, losHat)
     R_i = valid_i ? occ.Rocc : NaN          # 无效帧记 NaN，不参与统计
     marginMin / marginMinT 由 occ.shadowPolygon 更新
  6. pose = poseFromPositionAndAxis(P, V, CAMERA_UP_REF)（|V| < 1e-12 时用 (1,0,0)）
     填 fovMargin / lampPx（128 点，任一不可见 → 整帧 NaN）
     / uavPx（8 个角点，任一点在相机后方 → 整帧 NaN）/ imageR
  7. i < nFull−1 时：out = stepDartM2(...)；locked_i = out.locked；首个 true → tLock = t_i
     命中判定：segmentToPointDistance(P, out.P_next, L_i) ≤ HIT_DISTANCE_M → hitIndex = i（取第一个）
     P = out.P_next；V = out.V_next

截断：n = hitIndex ≥ 0 ? max(MIN_FRAMES, hitIndex) : nFull     # 命中帧本身不含
      hitLamp = hitIndex ≥ 0；tHit = hitLamp ? hitIndex·dt : null
      tLast = t[n−1]；所有逐帧数组切片到 n
```

**`valid` 帧判据（相机是否仍严格位于无人机与灯面之间）**

```
valid_i = allVerticesBetweenCameraAndPlane(P, uavCorners(uavCenter_i), L_i, losHat_i)
        = (d_cam 与 8 个顶点的 d 同号) && (|d_vertex| < |d_cam|)
```

飞镖最终会飞越无人机（无人机悬停在灯前 0.15 m），此后几何前提失效：
`valid = 0` 的帧 `R = NaN` → 曲线断线、HUD 显示 "—"、不参与 `R_min/R_mean` 统计。
默认参数下共 **1** 个无效帧（末帧）；`standoff = 0.25` 时 2 个（v2 的立方体是 3 个）。

> v1 的 `tEnd / gEff / pathLen / apexZ / apexT / vEnd / fullPath / dart / stopDistanceM /
> lampConvex / lampPolygon / LampSpec` **已全部删除**；v3 清理又删掉了无消费者的
> `offAxis / lampPolygon2D / uavPosition / simTEnd`；
> 3D 场景只画 M2 的 `Pd` 橙色实线，不再有灰色虚线原弹道。

### 5.6 必须复现的期望值（默认参数，即自证基准）

| 量 | 期望值 |
|---|---|
| `n` / `tLast` | **237** / **2.36 s**（命中于第 237 帧，命中帧不含） |
| `tLock` | **0.19 s**（θ0=30° 时灯初始在 FOV 外，0.19 s 后才"完整看到球"） |
| `hitLamp` / `tHit` | **true** / **2.37 s** |
| `marginMin` / `marginMinT` | **−3.36 mm** / **0.75 s**（< 0 ⟹ **默认不再完全遮挡**，见 §11.3） |
| `R_min` / `R_mean` | **0.974843** / **0.984918** |
| `valid` 无效帧数 | 1（末帧 `R = NaN`） |
| `lampY` | 全程 0；`uavY` 全程 0（yT=0 时无人机不需要动） |
| `lamp_y_pos`（yT=+0.24） | n=238 / tHit=2.38 s / R_min 0.974843 / marginMin −3.36 mm @0.75 s |
| `lamp_y_neg`（yT=−0.24） | n=237 / tHit=2.37 s / **R_min 0.477821** / marginMin **−27.49 mm @1.45 s** |
| `lamp_y_pos_v0_22`（v0=22） | n=208 / tHit=2.08 s / tLock 0.36 s / R_min 0.795789 / marginMin −14.18 mm @0.94 s |
| `standoff = 0.12` | n=237 / R_min **0.999994** / marginMin **+1.28 mm**（此站位下仍完全遮挡） |
| `standoff = 0.25` | n=237 / R_min **0.694037** / marginMin **−18.96 mm** / 2 个无效帧 |
| `lamp_segments = 64` | R_min **0.973497**（与 1024 的差 ≈ 0.0013） |
| `uav_rectangular`（150, 70, 90） | n=237 / tHit=2.37 s / R_min **0.994684** / marginMin −1.18 mm @0.00 s |

> 灯球按 1024 边形离散、分母用**解析** πr²，因此「完全遮挡」时 `R_occ` 的上限是
> `0.999994` 而非 1.0；一旦 `minMargin < 0`，`R_occ` 才会真正掉下来（v3 默认即如此）。
> 64 边形与 1e-6 容差不兼容，故默认 1024，64 仅用于面板上的「精度演示」。
> `yT = −0.24` **不是** `yT = +0.24` 的镜像（站位几何不对称），它会明显失去遮挡 —— 实测事实，见 §11.3。

---

## 6. 渲染契约

### 6.1 主场景元素（`render/buildScene.ts`）

| 元素 | 几何 / 材质 |
|---|---|
| M2 弹道 | `Pd` 折线（唯一弹道，**没有**灰色虚线原弹道），橙色 `0xf97316`，逐帧用 `setDrawRange(0, i+1)` 揭示 |
| 拖尾 | 固定 60 帧循环缓冲 + 顶点色渐变（橙） |
| 相机标记 | `ConeGeometry(0.03, 0.09, 16)` 红 `0xef4444`，`quaternion.setFromUnitVectors((0,1,0), forward)` |
| 无人机 | `BoxGeometry(sx, sy, sz)` + `EdgesGeometry(同)` 蓝 `0x3b82f6`（面 `0x1e40af` 透明 0.25）；尺寸取 `result.uavSizeM` **三轴**；**位置逐帧读 `uavPos`** |
| 灯球组 | `lampGroup`：`SphereGeometry(r, 32, 24)` 绿 `0x22c55e` 透明 0.45；赤道 40 点 `Points`（局部 XZ 面）+ 灯心小球；`position = lampFrames[i].center` |
| 灯移动轨道 `lampGroup` | ±240 mm `LineDashedMaterial` 灰虚线 + 两端限位小球 + 原点十字 + 橙色 `0xf59e0b` 当前灯位标记（每帧改 `position`），由 `display.lampTrack` 控显隐 |
| 阴影多边形 | 每帧从 `shadow2D` 重建顶点（容量 8，索引三角扇 `(sc−2)*3`），红 `0xef4444` 透明 0.5；**平面基逐帧取 `losBasis(Pd_i, L_i)`** |
| 顶点投影线 | 8 条线段：UAV 第 i 帧顶点 → 平面投影点，灰 `0x94a3b8`；顶点半尺寸**逐轴**取 `uavSizeM[i]/2`（`mainScene.uavCornerWorld`） |
| LOS / 光轴 | 橙虚线（`pose.position → lampFrames[i].center`）/ 红实线（长 1.5 m） |
| 视锥 | 4 条角射线 + 顶点环（`frustumCornerRays(pose, camera, 1.5)`），红 `0xf87171` |
| 梳齿 | 沿 M2 弹道（由 `Pd` 折线算弧长）每 1.2 m 一条 0.08 m 短切线，灰 `0x64748b` |

`SceneParts.reset(result)` 在参数变化后重建与规模相关的几何（弹道 / 梳齿 / 灯球半径 /
灯位标记 / 机体尺寸）；`dispose()` 遍历释放。

### 6.2 相机与视角（`render/mainScene.ts`）

- `PerspectiveCamera(45, aspect, 0.01, 100)`，`camera.up = (0,0,1)`；`OrbitControls` 参数：
  `object.up=(0,0,1)`、`minDistance 0.1`、`maxDistance 60`、`zoomSpeed 0.8`、
  `minPolarAngle 0`、`maxPolarAngle π−0.05`、`enableDamping true`、`dampingFactor 0.08`、
  `screenSpacePanning true`；**不改鼠标映射**（左键 ROTATE / 右键 PAN / 滚轮 DOLLY）。
- `VIEW_PRESETS`（`id / label / position / target / targetLampFactor`）；
  **target 的 Y 随灯平移**：`targetY = target[1] + targetLampFactor · lampY`（`lampY` 取当前帧）：

| id | label | position | target（灯 Y = 0 时） | factor |
|---|---|---|---|
| `closeup`（默认） | 末端特写 | `(-0.55, -0.42, 0.30)` | `(-0.07, 0, 0)` | 1 |
| `overview` | 全程 | `(-12.5, -28, 10)` | `(-12.5, -1.43, 1.0)` | 0.5 |
| `top` | 俯视 | `(-12.5, -1.43, 30)` | `(-12.5, -1.43, 0)` | 1 |
| `side` | 侧视 | `(-12.5, -30, 1.0)` | `(-12.5, -1.43, 1.0)` | 0.5 |
| `isometric` | 等轴测 | `(-35, -28, 20)` | `(-12.5, -1.43, 1.0)` | 0.5 |
| `firstPerson` | 第一人称 | 每帧取 `poses[i]` 写相机 | — | 0 |

- 预设切换做 **600 ms** 三次缓动（位置与 target 同时 lerp）；过渡期间与第一人称下
  `controls.enabled = false`，结束后恢复。**过渡期间光照跟随冻结**。
- **灯平移的视角跟随**：过渡结束后，`updateFrame` 用「灯 Y 的增量 × factor」同时平移
  `controls.target.y` 与 `camera.position.y`（纯平移，保留用户鼠标平移/缩放的相对关系），
  因此灯到位后 target 恰等于上表的 `targetY`。
- 第一人称由 `updateFrame` 用 `Matrix4.makeBasis(right, up, -forward)` 直接写相机姿态。
- `setDisplay(d)` 同时驱动 `lampTrackGroup.visible`、`grid.apply(...)` 与 `grid.setStyle(...)`；
  `resize(w,h)` 转发网格分辨率。

### 6.3 网格（`render/grid.ts`）

```ts
export interface GridOptions { lampPlaneCenter: THREE.Vector3; lampPlaneNormal: THREE.Vector3;
  coarseExtent: number; coarseStep: number; fineExtent: number; fineStep: number }
export const DEFAULT_GRID_OPTIONS = { center (0,0,0), normal (-1,0,0),
  coarseExtent 30, coarseStep 1, fineExtent 1, fineStep 0.1 };
export interface GridGroups { groupCoarseHorizontal: LineSegments2; groupCoarseLamp: LineSegments2;
  groupFineHorizontal: LineSegments2; groupFineLamp: LineSegments2 }
export function buildGrid(opts?: Partial<GridOptions>): GridGroups;   // 纯构建，不碰 DOM/renderer
export function buildAxes(range: number): Line2[];                    // 3 条
export class SceneGrid {
  readonly group: THREE.Group;      // 恰好 5 个扁平子 group（4 网格 + 1 坐标轴）
  readonly grid: GridGroups; readonly axes: Line2[];
  readonly groupCoarseHorizontal/groupCoarseLamp/groupFineHorizontal/groupFineLamp/groupAxes: THREE.Group;
  apply(cfg: Pick<DisplayOptions,'gridPlane'|'showCoarse'|'showFine'|'showAxes'>): void;
  setStyle(s: Pick<DisplayOptions,'gridCoarseColor'|'gridFineColor'|'gridCoarseWidth'|'gridFineWidth'>): void;
  setResolution(width: number, height: number): void;
  materials(): LineMaterial[];      // 7 个（4 网格 + 3 轴）
  dispose(): void;
}
```

- 几何：每个「平面 × 密度」组合只有 **1 个 `LineSegments2`**，`LineSegmentsGeometry.setPositions(Float32Array)`
  一次构建；段数 = 粗 **122/122**（±30 m，步长 1）、细 **42/42**（±1 m，步长 0.1）。
  水平面用基 `(1,0,0)/(0,1,0)`；灯球面用 `planeBasis(lampNormal)` → `(0,-1,0)/(0,0,1)`。
- 材质：每对象独立 `LineMaterial({ worldUnits:false, transparent:true, depthTest:true, depthWrite:false })`；
  网格 `linewidth` 默认 1、`opacity` 粗 0.9 / 细 0.7，`renderOrder` 粗 **−3**、细 **−2**；
  坐标轴 `linewidth:2`、`opacity:1`、颜色取 `AXIS_COLORS`、`renderOrder` **−1**、`frustumCulled=false`。
- `apply` 只设 5 个 group 的 `visible`（`plane='both'` 时两个平面组都可见；三个开关互相独立）；
  `setStyle` 只改 `material.color.set(hex)` 与 `material.linewidth`（**不重建几何、不动 opacity**）。
- **渲染护栏**：7 个 fat-line 材质统一 `transparent:true`，靠 renderOrder 保证「网格在下、坐标轴在上」；
  新增任何透明物体 `renderOrder` 必须 > −1。全部对象 `frustumCulled=false`（网格跨度 ±30 m，剔除无收益）。
- **resolution 单一入口**：`setResolution()` 是唯一写 `LineMaterial.resolution` 的函数；
  `mainScene.setGridResolution()` 只是转发；`export/mp4.ts` 只调用 `deps.onRenderResolutionChange`。
  交互态下 `LineSegments2.onBeforeRender` 每帧从当前 renderer 的 `getViewport()`（CSS 像素）同步，
  因此线宽单位为 CSS 像素。

### 6.4 FPV 场景（`render/fpvScene.ts`）

- `createFpvScene(canvas)` → `{ renderer, scene, camera, updateFrame(result,i), render(), setSize(w,h), dispose() }`。
- 独立 `Scene` + `OrthographicCamera(0, 640, 0, 480, -1, 1)`，与 `projectPixel` 的
  `[0,W]×[0,H]`（y 向下）像素坐标**完全一致**，无需再平移；
  因 `top=0 < bottom=480` 会翻转投影手性，**所有 Mesh 材质必须 `side: DoubleSide`**（否则被背面剔除）。
- 每帧用 `result.lampPx`（128 点）与 `result.uavPx`（8 角点）构建：灯球绿 `0x22c55e`、UAV 蓝 `0x3b82f6`、
  交集红 `0xef4444`；**UAV 角点必须先 `convexHull2D` 再交给裁剪器**（角点是索引顺序，不是边界顺序）。
  交集多边形按帧索引缓存 1 条。清屏色 `0x020617`。

---

## 7. UI 契约

### 7.1 `index.html` DOM 契约（`main.ts` 按 id 取节点，缺一个就抛错）

`#main-canvas`、`#curve-host`（内含 `#curve-uplot`、`#curve-effects`）、
`#hud-text-layer`（`#hud-roc`、`#hud-time`、`#hud-speed`）、
`#fpv-host`（`#fpv-canvas`、`#fpv-readout` 内含 `#fpv-roc`/`#fpv-image`/`#fpv-margin`）、
`#view-bar`、**`#no-hit-warning`**（无命中告警，`hidden` 默认，见 §7.6）、
`#timeline-bar`（`#timeline-controls` 内含 `#btn-play`、`#btn-reset`、`#speed-host`
（`#speed-slider` + `#speed-value`）、`#speed-hint`、`#btn-export`；`#timeline-track-host` 内含
`#timeline-track` > `#timeline-progress` + `#timeline-thumb`）、`#panel-host`、`#toast-host`、
`#export-overlay`（`#export-stage`、`#export-progress-fill`、`#export-percent`）。

### 7.2 HUD 布局（`ui/hudLayout.ts`，唯一像素来源）

```ts
export const FPV_INSET_ASPECT = 4/3;  export const FPV_INSET_RATIO = 0.012;
export const FPV_INSET_WIDTH_RATIO = 0.25;
export const DEFAULT_HUD_LAYOUT: HudLayout = {
  curveArea: { x: 0.00, y: 0.63, w: 1.00, h: 0.24 },       // 底部控制条上方
  timeline:  { x: 0.00, y: 0.87, w: 1.00, h: 0.13 },       // 最底部
  fpvInset:  { x: 1 - FPV_INSET_RATIO - FPV_INSET_WIDTH_RATIO, y: FPV_INSET_RATIO, w: 0.25, h: 0 },
  textLeft: { x: 0.02, y: 0.935 }, textCenter: { x: 0.50, y: 0.935 }, textRight: { x: 0.98, y: 0.935 },
};
export function toPixels(region, viewport): HudRegion;
export function pointToPixels(point, viewport): { x: number; y: number };
export function fpvInsetPixels(layout, viewport): HudRegion;   // h = w / FPV_INSET_ASPECT
export function fpvHoverPixels(layout, viewport): HudRegion;   // w = min(base.w×1.5, viewport.h×0.6)，右上角锚定
export function hudFontSizes(layout, viewport): { readout; meta };  // 0.17 / 0.128 × timeline band 高
```

归一化坐标 **y 向下**、相对整画布；屏幕与导出共用同一份布局。

### 7.3 曲线（`ui/curve.ts`，三层 HUD 的第 1、2 层）

- **层 1 = uPlot**（仅数据/参数变化时重绘）：x 范围 `[0, tLast]`、y `[0,1]`；坐标轴白色、
  网格 `#1e293b` 虚线 `[4,4]`；series = R_occ（蓝 `#3b82f6`，线宽 1.5）+ R_th 常量线（红 `#ef4444`，`dash [4,4]`）；
  `legend.show=false`、`cursor.show=false`；点击 `.u-over` 用 `posToVal(localX,'x')` 跳帧。
- **层 2 = Canvas2D 特效层**（每帧重绘，尺寸/位置与 uPlot 画布及 DPR 一致）：
  有效区间（`R ≥ R_th`）沿线下方做绿色渐变填充、曲线发光（+4px、`rgba(59,130,246,.25)`）、
  当前帧橙色竖线 + 脉冲圆环（周期 1200 ms、半径 3→10、透明度 0.6→0）；
  用 `valToPos(t,'x',true)` / `valToPos(v,'y',true)` 定位；**scale 未就绪时返回 NaN 必须直接 return**
  （否则 `createLinearGradient` 抛异常会中断初始化）。
- **无效帧断线**：`buildPlot` 把 `R[i]` 为 `NaN` 的帧改成 **`null`** 再交给 uPlot（uPlot 遇 null 自动断开）；
  特效层同步跳过 null（发光路径在 null 处 `moveTo` 重开，填充区间与脉冲圆环也跳过该帧）。

### 7.4 时间轴与播放（`ui/timeline.ts`）

- `snapSpeed(v)`：`|v − s| < 0.05` 时吸附到 `[0.25, 0.5, 1, 2, 4]` 之一。
- 控制条带内布局：控制行高 `0.30 × band.h`；进度条 host `top 0.66 × band.h`、高 `0.34 × band.h`；
  字号由 `hudFontSizes` 推导（倍速值用 readout，提示行用 `meta × 0.62`）。
- 三个文字锚点用 `pointToPixels` + `translate(-100%/-50%/0, -100%)` 定位。
- 进度条支持 `pointerdown/move/up` 拖动跳帧；`R_occ` 文字按阈值在绿/红之间 lerp（每帧系数 0.22），
  时间文本 `t = x.xx / y.yy s`。
- **无效帧显示 "—"**：`setReadouts(rOcc, ...)` 里 `Number.isFinite(rOcc) === false` 时
  锁定 `rInvalid`，`tickSmoothing` 直接把 `#hud-roc` 写成 `R_occ = —`（颜色回落 `COLORS.text`），
  有效帧再恢复正常 lerp。FPV 三行读数同样处理（`ui/fpvOverlay.ts`）。

### 7.5 参数面板（`ui/panel.ts`）

- 根节点标题「参数面板」；分组：**弹道参数 / 悬停站位 / 目标灯位置 / 无人机 / 灯球 / 相机 / 仿真 / 性能 / 显示选项**
  （显示选项内含子分组 **网格样式**），末尾「重置所有参数」。
- 「**目标灯位置**」（`expanded: true`）：`lampTargetYM` 滑块（`min −0.24`、`max 0.24`、`step 0.001`、
  label「灯 Y 偏移 [m]」）+ 按钮「重置灯位置」。改它 → `emitParams()` → 重跑仿真，
  灯轨迹 / 无人机轨迹 / 飞镖轨迹 / `R_occ` 曲线同步变化。
- label 全中文，**英文 key 一律不改**：初始速度 [m/s]、初始仰角 [°]、速度衰减率 [1/s]、
  距灯平面距离 [m]、竖直方向偏置 [m]、水平方向偏置 [m]、**机体长度 / 机体宽度 / 机体高度 [mm]**
  （v3 三滑块，min 50 / max 200 / step 1，**互不联动**）、最大速度 [m/s]、
  最大加速度 [m/s²]、灯球直径 [mm]、灯球多边形精度、精度演示（按钮）、ΔR（对比 1024 边形）、
  水平视场角 [°]、**分辨率（只读）**、仿真步长 [s]、遮挡分析阈值、
  显示弹道拖尾 / 显示阴影多边形 / 显示相机视锥 / 显示光轴梳齿 / 显示 FPV 小窗 / 显示遮挡率曲线、
  **显示灯移动轨道**、网格平面、显示粗网格、显示细网格、显示坐标轴、粗网格颜色、
  粗网格线宽 [px]、细网格颜色、细网格线宽 [px]。
  （v1 的「仿真截断距离 [m]」binding 已随 `stopDistanceM` 删除。）
- 每个分组一个**具名重置按钮**：`重置弹道 / 重置悬停站位 / 重置无人机 / 重置灯 / 重置相机 /
  重置灯位置 / 重置仿真 / 重置显示`（显示组重置整份 `display`，含网格平面、`lampTrack` 与 4 个样式项）；
  **不给分组加通用「重置本组」**——按钮名字必须说明它重置什么。
- 网格平面下拉 `options = { '灯球平面 (X=0)': 'lamp', '水平面 (Z=0)': 'horizontal', '两者都画': 'both' }`。
- **颜色控件**：直接绑定 hex 字符串（`{ picker: 'popup' }`）即可——Tweakpane v4 **没有 `addColor`，
  也没有 `view:'color'`**，`input-color-string` 插件优先级高于普通字符串输入；本项目不用 `color:{type}`
  （那是给 `{r,g,b}` 对象绑定用的）。
- 两个线宽的 `binding.disabled` 跟随 `showCoarse` / `showFine`，开关变更后刷新。
- 面板接口：`createPanel(container, initial: PersistedState, callbacks)` → `{ pane, setPerf, setPrecisionDelta,
  refresh, setParams, setDisplay(partial), getParams, getDisplay, dispose }`；
  `PanelCallbacks = { onParamsChange, onDisplayChange, onPrecisionDemo, onResetAll }`。
- **机体高度警告（v3，panel 内部自足）**：`createPanel` 内、所有 binding 建完后
  `container.appendChild(warnEl)` 创建 `<div id="uav-warning" hidden>⚠ 高度 &lt; 55 mm 时，遮挡策略可能失效</div>`
  （位置＝面板底部，pane DOM 之后）；
  `updateUavWarning()` 只做 `warnEl.hidden = params.uavSizeZMm >= 55`，**不调用** `pane.refresh()`，
  调用点为：① 创建时 ② `uavSizeZMm` 的 `on('change')` ③「重置无人机」后 ④「重置所有参数」内
  ⑤ `PanelView.setParams()` 后；`dispose()` 先 `warnEl.remove()` 再 `pane.dispose()`。
  **不改 `PanelCallbacks`、`main.ts`、`index.html`。**

### 7.6 `main.ts` 编排

- 启动：`loadPersisted()` → `params/display/speed`；先跑一次 `runSimulation`；创建
  `createMainScene(mainCanvas, result, display)`、`createFpvScene(fpvCanvas)`、`createCurveView(...)`、
  `createFpvOverlay(...)`、`createTimelineView(...)`、`createPanel(...)`；建视角预设按钮与「重置视角」。
- 主循环（`requestAnimationFrame`）：`playbackTime += dtReal × speed`（播放中且非导出中），
  到达 `tLast` 自动暂停；`index = frameIndexAt(result, playbackTime)`；`applyFrame(index)` 依次更新
  主场景 / FPV 场景 / 曲线游标 / 时间轴 / HUD 文字 / 过渡动画；随后渲染两个 WebGL 场景。
- 参数变更 → `scheduleRecompute()`（每动画帧最多一次）→ `runSimulation` →
  `mainScene.syncResult` + `curveView.setData` + 性能与 ΔR 读数；显示变更 → `mainScene.setDisplay`
  + 曲线/FPV 显隐，**不重算仿真**。
- 精度演示（ΔR）读**当前帧**的 `result.lampFrames[i]`（`.losHat / .radiusM`），
  分别用当前 `lampSegments` 与 1024 边形重算 `computeOcclusion(...).Rocc` 并显示差值。
- **无命中告警**（`#no-hit-warning`）：`applyFrame` 里
  `noHitWarning.hidden = result.hitLamp || i !== result.n - 1`，即"未命中 **且** 播放到末帧"时
  才叠加黄字「当前参数下飞镖无法命中」（`pointer-events: none`，不挡交互）。
- `applyLayout()`：窗口尺寸 → 主渲染器 resize、曲线矩形、时间轴、FPV 定位与 `fpvScene.setSize`；
  首次布局 400 ms 后给 `#fpv-host` 加 `.animate`（让宽高过渡只在悬停时生效）。
  `ResizeObserver` 监听 `#fpv-host`，把实际尺寸同步给 FPV 渲染器。
- 导出按钮 → 确认弹窗（文案含固定 60 fps、慢放 2×、覆盖时长）→ 全屏遮罩 + 进度 → `exportMP4`
  → `URL.createObjectURL` 下载 `uav-occlusion-<N>f.mp4` → `finally` 恢复网格分辨率并隐藏遮罩。
- 调试钩子 `window.__uav = { result, params, display, setDisplay(partial) }`。

---

## 8. 导出契约（`src/export/**`）

```ts
export interface ExportDeps { result: SimResult; scene: THREE.Scene;
  camera: THREE.PerspectiveCamera; fpvScene: FpvScene; uplotCanvas: HTMLCanvasElement;
  effectsCanvas: HTMLCanvasElement; updateFrame(frameIndex: number): void;
  onRenderResolutionChange?: (width: number, height: number) => void;
  onProgress(phase: string, frac: number): void }
export interface ExportResult { blob: Blob; frames: number; format: 'png'|'jpeg'; bytes: number; elapsedMs: number }
export class ExportError extends Error {}
export const EXPORT_TITLE = '无人机遮挡绿色引导灯 · Web 仿真器';   // 导出画面左上角标题
export function exportTimes(tLast: number): number[];
export async function exportMP4(deps: ExportDeps): Promise<ExportResult>;
```

| 项 | 契约 |
|---|---|
| 帧数 | `N = max(2, round(tLast × EXPORT_FPS / EXPORT_SPEED))` → 默认 **283**（tLast = 2.36 s） |
| 采样 | `t_i = tLast · i / (N − 1)`，首帧 t=0、末帧 t=tLast |
| 分辨率 / 帧率 / 码率 | 1920×1080 / 60 fps / 4000 kbps |
| 编码 | `ffmpeg.exec(['-framerate','60','-i', input, '-c:v','libx264','-pix_fmt','yuv420p','-b:v','4000k','-y','out.mp4'])` |

流程：

1. 自检 `self.crossOriginIsolated`，不满足抛 `ExportError`（提示需要 COOP/COEP）。
2. `deps.onRenderResolutionChange?.(1920, 1080)`。
3. 建离屏 `WebGLRenderer`（1920×1080、`setPixelRatio(1)`、`preserveDrawingBuffer`）与
   `exportCamera = camera.clone()`（aspect = 16/9，每帧复制主板相机的 position/quaternion）；
   另建 FPV 离屏渲染器（尺寸取 `fpvInsetPixels`）。
4. 逐帧：`simIndex = clamp(round(t_i / dt), 0, n−1)` →`deps.updateFrame(simIndex)` → 渲染主场景与 FPV →
   在 2D 合成画布上按顺序 `drawImage`：主画面整幅 → FPV 内嵌矩形 → `uplotCanvas` → `effectsCanvas`
   （矩形取 `toPixels(layout.curveArea)`）→ `drawHudText(...)`（文字层用 Canvas2D 重绘，含 FPV 三行读数与标题）。
5. 首帧编码为 PNG；若 `首帧字节数 × N > EXPORT_MEMORY_BUDGET_BYTES(120 MB)` → 改用 JPEG(q=0.95)，
   写入 `frame_%04d.png|jpg`。
6. 编码 → `readFile('out.mp4')` → `Blob`；`finally` 中删除所有帧文件与 `out.mp4`。

`export/hud.ts` 导出 `drawHudText(ctx, state, layout, viewport)`，`state = { rOcc, rTh, t, tLast, speed,
imageRatio, fovMarginDeg, title }`；位置与字号全部来自 `hudLayout`（`R_occ` 按阈值着色）。
**无效帧**（`R = NaN`）时四处读数一律画 "—"（`R_occ = —` / `R_occ —` / `像面遮挡比 —` / `视场余量 —`），
与网页预览的 HTML overlay 完全一致（所见即所得）。

---

## 9. 持久化契约（`ui/panel.ts`）

- 键 `rm-uav-occlusion-params-v3`，值 `{ version: 3, params, display, speed }`。
- 加载：无记录 → 默认；`version !== 3` → 丢弃并回落默认（控制台 warn：
  `[params] localStorage schema 版本不匹配（x ≠ 3），已回落默认值`）；
  否则 `params = sanitizeParams(raw.params)`、`display = sanitizeDisplay(raw.display)`。
- `sanitizeParams`：逐字段 clamp 到 `PARAM_RANGES`，非有限数回落默认，
  `lampSegments` 取整到 `LAMP_SEGMENTS_STEP` 并 clamp `[64,1024]`，
  `lampTargetYM` 非有限数 → 0、否则 clamp 到 `[-0.24, 0.24]`，
  `uavSizeXMm/YMm/ZMm` 三轴各自 clamp 到 `[50, 200]`（**已导出以便单测**）。
- **旧存档兼容**：`sanitizeDisplay` / `sanitizeParams` 会把缺省的新字段
  （`lampTrack`、`lampTargetYM`、网格 4 项…）补成默认值，**不丢**其余已存参数。
- 保存：参数/显示变更后 **500 ms debounce** 写入；「重置所有参数」清空 localStorage。

---

## 10. 测试与黄金基准

### 10.1 测试文件与断言要点（`npm run test`，共 **129** 项 / 10 个文件）

| 文件 | 项数 | 断言要点 |
|---|---|---|
| `tests/parity.test.ts` | 52 | **8 组**黄金数据逐点比对：`n/tLast/tLock/tHit/hitLamp` 严格一致；`Pd/Vd/lampPos/uavPos/lampY/uavVy` ≤1e-9；有效帧 `R` ≤1e-6 且 **`null ⟺ NaN` 位置严格一致**；`locked/valid` 逐帧一致；`imageR/fovMargin` ≤1e-6；`tools_parity.json` 五类函数 ≤1e-12；`uavCorners`/`planeBasis`/`losBasis`/`isFullyVisible` 与 Python 一致（采样为三轴 `0.15/0.07/0.09`）；64 vs 1024 的 ΔR |
| `tests/uavSize.test.ts` | 11 | v3：`uavCorners` 三轴 8 顶点逐坐标（±0.075/±0.035/±0.045）+ `idx^1/^2/^4` 相邻性；`uavEdges` 12 条；`bodySafeBounds` 逐轴收缩；`hoverStationPositionAt` = `(−0.15, 0, 0)`；`runSimulation` 三轴透传；只改 X 时 Y/Z 完全不变（无联动）；`PARAM_RANGES` 三条 `[50,200]`；`DEFAULT_PARAMS=(100,100,80)`；`sanitizeParams` clamp 300→200 / 10→50 / NaN→默认 |
| `tests/lampOffset.test.ts` | 16 | `lampYAt` 分段边界（0/1.19/1.2/1.5/1.79/1.8/2.0，正负对称）；`lampVyAt` 常数段；`segmentToPointDistance` 三点（内/外/退化）；`uavStep` 三角（0.70 s 到位、峰值 ≈0.686）、梯形（封顶 5.0）、远离目标、已在目标；`stepDartM2` 弹道段逐分量等价、追踪段速率守恒 ≤1e-12、转角 ≤ `ω_max·dt`、可对齐时直接对齐、速度退化不产生 NaN |
| `tests/occlusionMargin.test.ts` | 5 | v3 默认 `marginMinT = 0.75 s`、`marginMin ≈ −3.36 mm < 0`、`R_min ≈ 0.974843`、末帧 `R = NaN`；`(22,35)` 的 `marginMin ≈ −27.33 mm`、`R_min ≈ 0.075741`（模型特性回归固化，容差 ±1 mm / ±0.02） |
| `tests/grid.test.ts` | 14 | 根 group 5 个子 group；每网格组 1 个 `LineSegments2`；`instanceStart.count` = 122/122/42/42；灯球面端点 `|x|≤1e-9`、水平面 `|z|≤1e-9`；`apply` 可见性矩阵；`setStyle` 改色改宽且**几何引用不变**、opacity 不变；`setResolution` 传播到 7 个材质、非法尺寸 no-op；`dispose` 清空；全部 `frustumCulled=false` |
| `tests/axes.test.ts` | 7 | `buildAxes(30)` → 3 条 `Line2` + 3 个独立材质；颜色等于 `AXIS_COLORS` 的 hex（**测试内不写字面量**）；`linewidth 2`、`worldUnits false`、`renderOrder -1`、不写深度；每条 1 段且端点覆盖 ±30；`resolution` 挂在材质上可写 |
| `tests/display.test.ts` | 9 | `sanitizeDisplay()` 与默认深等且非同一引用；`null/undefined/'foo'/42/true/[]` → 全默认；旧存档只保留旧字段；非法布尔/平面/颜色回落；线宽 clamp 与 NaN/Infinity 回落；`lampTrack` 布尔白名单 + 默认 true |
| `tests/hudLayout.test.ts` | 12 | 1080p FPV 480×360、900p 400×300；右边缘 `(1−0.012)×宽`；`fpvHoverPixels` → 648×486 / 540×405 且右上角锚定不变量；曲线区在时间轴正上方且不重叠；文字锚点在 band 内；字号由 band 推导 |
| `tests/polygon.test.ts` | 2 | 236 帧自写裁剪与 `polygon-clipping` 面积 `|Δ| ≤ 1e-9 m²`（灯球多边形取 `result.lampFrames[0].polygon2D`）；打印两者耗时对比 |
| `tests/perf.test.ts` | 1 | 默认参数单次 `runSimulation` 平均耗时打印（宽松上限保护 200 ms） |

### 10.2 黄金基准 `tools/gen_golden.py`

用 `--python-dir`（默认 `tools/python_ref`）导入 vendored Python 参考实现，按 M2 参数
（`dt = 0.01`、`t_end = 5.0`、`hit = 0.05`、`ω_max = 60°/s`、`aMax = 2.0`）现算并写到
`tests/golden/*.json`：

```jsonc
{ "params": {standoff_m, offset_u_m, offset_v_m, uav_size_mm, lamp_diameter_mm,
             lamp_quad_segs, lamp_segments, hfov_deg, v0_mps, theta0_deg,
             speed_decay_per_s, dt, R_th, lamp_target_y_m},
  "n": 237, "tLast": 2.36, "lampYTargetM": 0.0,
  "tLock": 0.19, "tHit": 2.37, "hitLamp": true,
  "lampY": [..n..], "lampPos": [..3n..], "uavY": [..n..], "uavVy": [..n..], "uavPos": [..3n..],
  "Pd": [..3n..], "Vd": [..3n..],                 // M2 位置 / 速度
  "R": [..n..],                                   // ⚠️ 无效帧写 null（不是 NaN）
  "imageR": [..n..], "fovMarginDeg": [..n..],
  "locked": [..n..], "valid": [..n..],            // 0/1
  "lamp": {radiusM, areaM2, polygonAreaM2},
  "camera": {width, height, fovHDeg, focalPx},
  "marginMinM": 0.018086597048602247, "marginMinT": 0.74 }
```

**NaN 序列化（必须处理）**：Python `json.dumps` 会写出非法 JSON 字面量 `NaN`，而 Vite 的 JSON 导入
走 `JSON.parse` 会直接抛错。生成器把无效帧的 `R` 写成 **`null`**，测试侧按
「`null` ⟺ `Number.isNaN(R[i])`」比对，其余帧按 ≤1e-6 比对。

- **8 组 case**：`default`（20/30/yT=0）、`lamp_y_pos`（yT=+0.24）、`lamp_y_neg`（yT=−0.24）、
  `lamp_y_pos_v0_22`（v0=22、yT=+0.24）、`standoff_012`、`standoff_025`、
  `lamp_segments_64`（`lamp_quad_segs=16` → 64 边形）、**`uav_rectangular`（v3 新增，`uav_size_mm=(150,70,90)`）**。
- **`params.uav_size_mm` 是 3 元素数组 `[X, Y, Z]`（v3 起）**：生成器默认取 vendored
  `config.UAV_SIZE_MM`（唯一真值），`build_case` 只在显式传入时覆盖。
- `build_case` 的 8 组默认尺寸均为 `(100, 100, 80)`，因此**除 `uav_rectangular` 外的 7 组也全部随 v3 刷新**。
- `tools_parity.json`：`lampYAt / lampVyAt / stepDartM2 / segmentToPointDistance / uavStep`
  的输入→输出表（Python 现算），TS 端逐条 ≤1e-12 断言（跨语言浮点顺序护栏）。
- `margin.json`：各 case 的 `n / tHit / hitLamp / tLock / R_min / R_mean / invalidFrames /
  marginMinM / marginMinT`。
- `geometry.json`：`uavCorners` 8 点（**v3 采样尺寸 `[0.15, 0.07, 0.09]`**，非立方体，锁定三轴顶点顺序）
  + `planeBasis` + `losBasis` 3 组 + `isFullyVisible` 3 组采样。

---

## 11. 验收与实测

### 11.1 命令

```bash
npm run golden   # 重新生成黄金基准（需 numpy；走 vendored Python）
npm run build    # tsc --noEmit + vite build，必须 0 错误
npm run test     # 129 项全绿
npm run dev      # 人工验证（见 §11.6）
```

### 11.2 已实测结果

| 项 | 结果 |
|---|---|
| `tsc --noEmit` / `vite build` | 0 错误 / 构建通过（产物含 32 MB ffmpeg 核心） |
| `npm run test` | **129/129**（数值 52 / 运动学 16 / 余量 5 / 三轴机体 11 / 网格 14 / 坐标轴 7 / 显示 9 / HUD 12 / 裁剪 2 / 性能 1） |
| 单次仿真 | 30 ~ 55 ms（237 帧 × 遮挡 + FPV 预计算；随机器的并行负载波动） |
| 交集裁剪 | 236 帧：自写 Sutherland–Hodgman ≈4 ms vs `polygon-clipping` ≈210 ms（约 50×） |
| 默认 v3 结果（CDP 读页面内 `__uav.result`） | `n=237`、`tLock=0.19`、`tHit=2.37`、`hitLamp=true`、`marginMin=−3.36 mm @0.75 s`、`R_min=0.974843` |
| `lampTargetYM = +0.24` 回归 | 参数落盘并复现：`lampY[150]=0.12`、`lampY[末]=0.24`、`uavPos.y[末]=0.24`、`tHit=2.38`（与 `lamp_y_pos` 黄金一致） |
| 无命中告警（v0=28, θ0=35） | `hitLamp=false`、`tHit=null`、`n=501`；播放到末帧后 `#no-hit-warning` 显示黄字「当前参数下飞镖无法命中」 |
| 机体高度警告 | 「机体高度」拖到 50 mm → 面板**底部**出现红字 `⚠ 高度 < 55 mm 时，遮挡策略可能失效`；拖回 80 mm 消失；「重置无人机」后消失 |
| 三轴独立 | 只拖「机体长度」到 200 → 机体视觉变长、`R_occ` 变化，宽度/高度不变；`runSimulation` 的 `uavSizeM` 只改 X |
| 持久化版本回落 | 手写 `version=2` → 控制台 warn `[params] localStorage schema 版本不匹配（2 ≠ 3）`，参数回落默认（`uavSizeXMm/YMm/ZMm=100/100/80`、`lampTargetYM=0`、速度 0.50×） |
| FPV 悬停锚定（CDP，1600×900） | 右边缘恒为 1551.2 px，宽度 392.5 → 454.8 → 392.5 |
| 网格 | 三平面可切换、三开关独立；粗网格改红 + 线宽 3 后画面确实变红变粗 |
| ffmpeg.wasm | `crossOriginIsolated = true`、ffmpeg 5.1.4、`libx264` 可用、输出 `ftypisom` |
| 完整导出（无头 Edge，v3 默认参数） | **283 帧 / 1920×1080 / 1.8 MB / 38.7 s**，编码格式回传 `png`；结束后遮罩收起、网格分辨率恢复屏幕值 |

需在真实浏览器中人工确认：鼠标旋转/平移/缩放手感、悬停放大动画流畅度、MP4 下载后的播放观感。

开发期自检：`http://localhost:5173/?selftest=ffmpeg` 在页面内自检跨源隔离、ffmpeg 核心加载与
libx264 编码，结果写入 `document.title`（可配 `node tools/cdp-probe.mjs <url> <waitMs> "document.title"` 读取；
该脚本还支持第 5~6 个参数传截图路径与等待毫秒）。

### 11.3 `0.999994` 是"完全遮挡"的上限，v3 默认掉到 0.974843

`0.999994` 来自"灯球 1024 边形离散 ∩ 阴影 ÷ **解析** πr²"：阴影完全盖住灯球时，
交集 = 多边形面积 = πr²·(n·sin(2π/n)/(2π)) → n = 1024 时正好 0.999994。
**它不等于 1，也不代表"遮挡不满"**——判断是否真正完全遮挡要用与 `R_occ` **独立**的余量判据：
`margin = 灯心到阴影多边形最近边的距离 − 灯球半径`，`margin ≥ 0` 则 `R_occ` 取满 0.999994。

| 配置（机体 100/100/80） | `marginMin` | 时刻 | `R_min` | 结论 |
|---|---|---|---|---|
| 默认 (20, 30, yT=0) | **−3.36 mm** | 0.75 s | **0.974843** | **默认不再完全遮挡**（灯球露出 ≈2.5% 面积） |
| yT=+0.24 | −3.36 mm | 0.75 s | 0.974843 | 同上（最紧时刻在灯开始移动之前） |
| yT=−0.24 | **−27.49 mm** | 1.45 s | **0.477821** | 大幅失效（**不是** +0.24 的镜像） |
| standoff=0.12 | **+1.28 mm** | 0.75 s | 0.999994 | 此站位下**仍完全遮挡** |
| standoff=0.25 | **−18.96 mm** | 0.75 s | **0.694037** | 站位过远 → 失效 |
| `uav_rectangular` (150,70,90) | −1.18 mm | 0.00 s | 0.994684 | 窄而长的机体在 t=0 最紧 |
| (22, 35) | **−27.33 mm** | 0.91 s | **0.075741** | 明显失效（模型特性） |

**为什么 v3 默认会掉下来**：v2 的 120 mm 立方体在站位法线方向上有 60 mm 半展宽，够盖住
"灯球半径 27.5 mm + 阴影中心偏移 ≈17 mm"；v3 把高度 Z 降到 80 mm 后，最紧方向只剩 40 mm 半展宽，
于是在最紧时刻（t = 0.75 s，灯开始移动之前）缺 3.36 mm。**机身高度 Z 是决定性那一维**
（`(100,100,100)` 就能回到 +6.60 mm，而 `(120,120,80)` 仍为 −1.81 mm）。

> 若要把默认恢复成完全遮挡（三选一，均为实测）：`UAV_SIZE_Z_MM ≥ 87`（保持 X=Y=100，@87 → +0.12 mm）、
> `UAV_SIZE_X_MM = UAV_SIZE_Y_MM ≥ 145`（保持 Z=80，@145 → +0.12 mm）、
> 或 `HOVER_STATION_STANDOFF_M ≤ 0.125`（@0.125 → +0.51 mm）。
> 另一条 UI 路径：把「悬停站位」的 `standoffM` 调到 0.12 以下。

> S3.5 的三重验证（针对"余量为负 ⟹ R<1"这一类失效）：① 逐帧 `minMargin < 0` 与 `R < 1`
> **零不一致**；② 用独立方法（投影点凸包面积）重算交集，差 3.5e-12 mm²；
> ③ 蒙特卡洛采样估面积，`R` 差 0.00042。**结论：不是 `computeOcclusion` 的 bug。**

### 11.4 三个可调杠杆（改这里就会加重 / 恢复"完全遮挡"）

> 口径：`yT = +0.24 m`（灯真的会动；`yT = 0` 时无人机不动，追踪类杠杆全部失灵）。
> 机体固定 `(100, 100, 80)`；下表由 vendored Python 现算，TS 端与它 1e-12 对齐。

| 杠杆（改 `src/config.ts` 后重跑 `npm run golden`） | 移动段 `marginMin` | 全程 `R_min` |
|---|---|---|
| **默认**：`LAMP_MOVE_END_S = 1.8`、`UAV_MAX_ACCEL_MPS2 = 2.0` | −3.27 mm @1.20 s | 0.974843 |
| `UAV_MAX_ACCEL_MPS2 = 1.0`（加速度减半 → 追踪滞后变大） | −27.41 mm @1.75 s | **0.380600** |
| `UAV_MAX_ACCEL_MPS2 = 0.5` | −27.43 mm @1.99 s | **0.000000** |
| `LAMP_MOVE_END_S = 1.6`（灯提前到位 → 0.6 m/s） | −27.35 mm @1.38 s | **0.184458** |
| `LAMP_MOVE_END_S = 1.4` | −27.00 mm @1.60 s | **0.000000** |
| `lampTargetYM = ±0.60`（超出 ±240 mm 物理限位） | −27.32 / −27.38 mm | **0.000000**（灯球完全不被遮挡） |
| `yT = −0.24`（**合法**取值，但几何不对称） | −27.49 mm @1.45 s | 0.477821 |
| `UAV_SIZE_Z_MM = 100`（只把高度加回 100） | +6.60 mm @0.75 s | 0.999994（恢复完全遮挡） |

**v3 默认 `LAMP_MOVE_END_S = 1.8` + 机体 `(100,100,80)` 下，最紧时刻（0.75 s）仍在灯开始移动之前**，
所以"追踪滞后"不是主因——真正决定成败的是**机体在站位法线方向的半展宽**与**站位距离**（§11.3）。
要做"彻底失去遮挡"的演示，把「机体高度」拖到 55 mm 以下、或把 `lampTargetYM` 推到 ±0.24 之外即可。

### 11.5 S2 数值实验摘要（v0 / θ0 选型依据）

实验网格（临时脚本对 M2 的等价实现，脚本未入库；可用 `tools/python_ref` 复现）：
`k = 0.55`、`ω_max = 60°/s`、`g = 9.8`、`t_end = 5.0`、`dt = 0.01`、`aMax = 2.0`、**机体 `(100,100,80)`**；
`v0 ∈ {18, 20, 22, 25, 28}` × `θ0 ∈ {20, 26, 30, 35}` × `yT ∈ {0, +0.24}` = **40 组**；
每组记录 `t_lock / t_hit / hitLamp / n / R_min / R_mean / minMargin / uav_late`
（完整 40 行表随 v3 交付消息给出，不入库）。

关键行（已用最终代码复算核对）：

| 类型 | v0 | θ0 | yT | t_lock | t_hit | hitLamp | n | R_min | minMargin [mm] |
|---|---|---|---|---|---|---|---|---|---|
| **默认** | 20 | 30 | 0 | 0.19 | 2.37 | ✓ | 237 | **0.974843** | −3.36 @0.75 s |
| 正极限 | 20 | 30 | +0.24 | 0.19 | 2.38 | ✓ | 238 | 0.974843 | −3.36 @0.75 s |
| 负极限 | 20 | 30 | −0.24 | 0.19 | 2.37 | ✓ | 237 | **0.477821** | −27.49 @1.45 s |
| 最快命中 | 28 | 26 | 0 | 0.00 | 1.31 | ✓ | 131 | 0.997162 | −0.78 @0.55 s |
| 最慢命中 | 18 | 20 | 0 | 0.00 | 2.79 | ✓ | 279 | 0.999994 | +10.37 @0.00 s |
| 无命中代表 | 25 | 30 | 0 | 1.03 | — | ✗ | 501 | **0.000000** | −27.27 @0.76 s |
| 三轴对照 `(150,70,90)` | 20 | 30 | 0 | 0.19 | 2.37 | ✓ | 237 | 0.994684 | −1.18 @0.00 s |

**推荐值 = 选定默认 (20, 30)**：`t_lock = 0.19 s` 使"初始看不到灯 → 飞一段后完整看到 → 开始制导"
三段过程完整可见，`t_hit = 2.37 s` 留出灯运动（1.2~1.8 s）与无人机就位的观察窗口。
**无命中区（UI 会弹黄字告警）**：`v0 ≥ 25 且 θ0 ≥ 30` 的部分组合永不命中
（例：v0=28、θ0=35 → `n = 501` 跑满 5 s、`tHit = null`）。

**v3 默认 (100,100,80) 下的结论**：遮挡策略**不再完全成立** —— `R_min ≈ 0.974843`、
`minMargin ≈ −3.36 mm @ t ≈ 0.75 s`（灯球边缘露出 ≈2.5% 面积）。这是物理必然
（机身高度从 120 降到 80 mm 使最紧方向的半展宽从 60 mm 掉到 40 mm），不是 bug。
**策略适用范围**：`yT ∈ [0, +0.24]` 时缺口很小（`R_min > 0.97`）；`yT < 0` 时在某一阈值后明显失效
（`yT = −0.24` → `R_min 0.477821`）。恢复完全遮挡的三条路径见 §11.3。

**`uav_late` 口径**（yT = +0.24、aMax = 2.0、vMax = 5.0，全部由 `tools/python_ref` 现算）：

- **主指标：实测 0.180 s** —— 逐步调 `uavStep`、按「|Δy| ≤ 1 mm 且 |v| ≤ 0.05 m·s⁻¹」的收敛判据，
  测"灯 1.8 s 到位之后无人机还要多久才算就位"（到达于 t = 1.98 s）。
- **移动段稳态滞后 40.00 mm** —— 灯匀速平移期间无人机与灯的最大 Y 差；
  把这 40 mm 从 v ≈ 0.4 m/s 以 aMax = 2 m/s² 制动到 0 的解析时间 ≈ 0.2 s，与实测 0.180 s 相互印证。
- 作为对照：若无人机**等灯停下再追**、从静止走完 0.24 m，最短也要 `2√(r/a) = 0.693 s`；
  实测 0.180 s 远小于它，说明它确实是"追移动中的灯"。

**注意：v3 默认下最紧时刻（0.75 s）发生在灯开始移动之前，所以遮挡缺口与追踪滞后无关**，
它由站位几何（机体半展宽 vs 灯球半径 + 阴影中心偏移 ≈17 mm）决定；把 `aMax` 减半会进一步
把 `R_min` 压到 0.380600（§11.4）。

### 11.6 人工验证清单（`npm run dev`）

1. 默认 (20, 30) 打开页面 → 播放 → 确认**灯动 / 无人机追 / 飞镖制导 / 命中**四件事都发生；
2. `lampTargetYM` 拖到 ±0.24 → 重跑 → 灯轨迹 / 无人机轨迹 / 飞镖轨迹 / `R_occ` 曲线同步变化；
3. `v0 / θ0 / k` 面板可调，观察 `t_lock` / `t_hit` 变化（ω_max / g / T1 / aMax 是集中参数区的常量，
   改后必须重跑 `npm run golden`）；
4. **三轴独立**：「机体长度」拖到 200 → 机体视觉变长、`R_occ` 曲线变化，「宽度/高度」保持不变；
   再把「机体宽度」拖到 60 → 机体变窄（三轴互不联动）；
5. **高度警告**：「机体高度」拖到 50 mm → 面板**底部**出现红字 `⚠ 高度 < 55 mm 时，遮挡策略可能失效`；
   拖回 80 mm 或点「重置无人机」→ 警告消失，三轴回到 `(100, 100, 80)`；
6. 把 `v0=28、θ0=35` → 应触发**无命中黄字告警**；
7. 显示选项 → 开关「显示灯移动轨道」（±0.24 m 虚线 + 限位 + 当前灯位标记同步显隐）；
8. 视角预设 target 随灯平移（灯在 +0.24 时点「俯视」，落点仍在灯上）；
9. 刷新页面参数恢复；手改 `localStorage.version = 2` → 回落默认并 `console.warn`；
10. 「悬停站位」拖到 0.12 → `R_occ` 回到 0.999994（§11.3 的恢复路径之一）；
11. 导出 MP4，内容与预览一致（含网格颜色/线宽/显隐、无效帧 "—"）。

---

## 12. 已知限制

- **t = 1.2 s 没有显式切换**：弹道段 / 追踪段由 `locked` **逐帧独立**判定，允许振荡
  （追踪中灯临时出 FOV → 回到弹道段 → 再进 FOV 再锁定）。这是 C8=B 的预期行为。
- **无人机实时追踪有稳态滞后 ≈ 40 mm**：`uavStep` 无平滑，稳态有 ±dt 量级抖动；
  `R_occ` 全程 1.0 的原因是站位几何，不是滞后（§11.5）。
- **末端无效帧**：飞镖飞越无人机后 `valid = 0`，该帧 `R = NaN` → 曲线断线、HUD 与导出一律 "—"、
  不参与 `R_min/R_mean`。默认 1 帧，standoff 0.25 时 3 帧。
- **无命中区**：`v0 ≥ 25 且 θ0 ≥ 30` 的部分组合永不命中（跑满 5 s），UI 叠加黄字告警。
- **v3 默认 (100,100,80) 下不再完全遮挡**：`R_min = 0.974843`、`minMargin = −3.36 mm @0.75 s`；
  恢复路径见 §11.3（`Z ≥ 87` / `X=Y ≥ 145` / `standoff ≤ 0.125`）。
- **`yT = −0.24` 会明显失去遮挡**（`R_min = 0.477821`、`minMargin = −27.49 mm`），
  因为站位几何对 Y 不正对称 —— 模型特性，不是 bug（§11.3）。
- **机体高度 < 55 mm**：面板底部红字提示「遮挡策略可能失效」，但**不拦截、不改仿真逻辑**；
  `Z = 50 mm`（UI 下限）时缺口远大于 55 mm 情形。
- **`standoffM = 0.05`（UI 下限）**：无人机贴着灯平面，`valid = 0` 覆盖全部帧
  → `R` 全为 `NaN`、曲线整段断线、HUD/导出一律 "—"（几何前提不再成立，不是 bug）。
- **`fullPath` 已取消**：3D 场景只画 M2 的 `Pd` 橙色实线，不再有灰色虚线原弹道。
- FPV 在飞行前段几乎看不到内容：灯球在 640×480 像面上只有十几 px（真实针孔成像结果，非渲染错误）；
  拖到后段即可看到完整绿球 / 蓝轮廓 / 红交集。
- 1080p 导出较慢（wasm 单线程 x264）；如需加速可换 `@ffmpeg/core-mt`（COOP/COEP 配置不变）。
- 未实现热图扫描与站位可行区间扫描（属 Python 项目能力）。
- `minPolarAngle = 0` 允许相机停到 +Z 正上方，极点处拖拽有轻微抖动（未加护角）。
- `LineBasicMaterial.linewidth` 在 Chrome/ANGLE 被忽略（永远 1 px），因此网格与坐标轴统一改用
  fat line（`LineSegments2` / `Line2` + `LineMaterial`）；颜色/线宽只改材质属性，不重建几何。
- Tweakpane v4 **没有 `addColor`、也没有 `view:'color'`**：绑定 hex 字符串即自动使用颜色选择器
  （仅 `view:'text'` 会降级成文本框）；`color: {type}` 只对 `{r,g,b}` 对象/数值分量绑定才有意义，
  本项目**不使用**它。
- 无头浏览器截图对多 WebGL canvas 只呈现其中一个（开发环境现象，真实浏览器正常）。

改动渲染层前请先读 `src/render/grid.ts` 顶部的透明队列 / renderOrder / frustumCulled 约定
（`frustumCulled = false` 是显式选择：网格跨度 ±30 m、始终在视野语义内，剔除收益为零；
three r169 的 `LineSegmentsGeometry.setPositions()` 已自动算好包围球，开启剔除也不会闪没）。

---

## 13. 附录：逐字文件

下面两个文件请**原样保存**（其余文件按 §2~§10 的契约实现即可）。

### 13.1 `index.html`

```html
<!doctype html>
<html lang="zh-CN">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>无人机遮挡绿色引导灯 · Web 仿真器</title>
  </head>
  <body>
    <div id="app">
      <canvas id="main-canvas"></canvas>

      <!-- 层 1 + 层 2：R_occ 曲线（uPlot 画布 + Canvas2D 特效层） -->
      <div id="curve-host">
        <div id="curve-uplot"></div>
        <canvas id="curve-effects"></canvas>
      </div>

      <!-- 层 3：HUD 文字读数 -->
      <div id="hud-text-layer">
        <span id="hud-roc">R_occ = —</span>
        <span id="hud-time">t = 0.00 / 0.00 s</span>
        <span id="hud-speed">0.50×</span>
      </div>

      <!-- FPV 小窗（独立 canvas + 独立正交场景） -->
      <div id="fpv-host">
        <canvas id="fpv-canvas"></canvas>
        <div id="fpv-readout">
          <span id="fpv-roc">R_occ —</span>
          <span id="fpv-image">像面遮挡比 —</span>
          <span id="fpv-margin">视场余量 —</span>
        </div>
      </div>

      <!-- 顶部浮动条：视角预设 -->
      <div id="view-bar"></div>

      <!-- 无命中告警：参数组合下飞镖无法命中时，播放到末帧叠加黄字 -->
      <div id="no-hit-warning" hidden>当前参数下飞镖无法命中</div>

      <!-- 底部时间轴控制条 -->
      <div id="timeline-bar">
        <div id="timeline-controls">
          <button id="btn-play" title="播放 / 暂停">▶</button>
          <button id="btn-reset" title="回到起点">⟲</button>
          <div id="speed-host">
            <span class="speed-end">慢放</span>
            <input id="speed-slider" type="range" min="0.25" max="4" step="0.01" value="0.5" />
            <span class="speed-end">快放</span>
            <span id="speed-value">0.50×</span>
          </div>
          <span id="speed-hint">当前预览 0.50×，导出视频固定 0.50×</span>
          <button id="btn-export">导出 MP4</button>
        </div>
        <div id="timeline-track-host">
          <div id="timeline-track">
            <div id="timeline-progress"></div>
            <div id="timeline-thumb"></div>
          </div>
        </div>
      </div>

      <div id="panel-host"></div>
      <div id="toast-host"></div>

      <!-- 导出遮罩（拦截 UI 交互，但进度条持续动画） -->
      <div id="export-overlay" hidden>
        <div class="export-card">
          <h3>正在导出 MP4</h3>
          <p id="export-stage">准备中…</p>
          <div class="export-progress"><div id="export-progress-fill"></div></div>
          <p id="export-percent">0%</p>
        </div>
      </div>
    </div>
    <script type="module" src="/src/main.ts"></script>
  </body>
</html>
```

### 13.2 `src/style.css`

```css
:root {
  --bg: #0f172a;
  --primary: #3b82f6;
  --threshold: #ef4444;
  --effective: #10b981;
  --text: #f8fafc;
  --accent: #f59e0b;
  --grid: #1e293b;
  --panel: rgba(15, 23, 42, 0.86);
  font-family: Inter, system-ui, 'Segoe UI', sans-serif;
}

* {
  box-sizing: border-box;
}

html,
body {
  margin: 0;
  padding: 0;
  width: 100%;
  height: 100%;
  overflow: hidden;
  background: var(--bg);
  color: var(--text);
}

#app {
  position: fixed;
  inset: 0;
}

#main-canvas {
  position: fixed;
  inset: 0;
  width: 100%;
  height: 100%;
  display: block;
}

#curve-host {
  position: fixed;
  left: 0;
  top: 0;
  width: 100px;
  height: 60px;
  background: rgba(15, 23, 42, 0.55);
  border-top: 1px solid rgba(148, 163, 184, 0.25);
  z-index: 5;
}

#curve-uplot {
  position: absolute;
  inset: 0;
}

#curve-effects {
  position: absolute;
  inset: 0;
  pointer-events: none;
}

.u-legend {
  display: none;
}

#hud-text-layer {
  position: fixed;
  inset: 0;
  pointer-events: none;
  z-index: 7;
}

#hud-text-layer span {
  position: absolute;
  white-space: nowrap;
  font-weight: 500;
  text-shadow: 0 1px 3px rgba(0, 0, 0, 0.7);
}

#hud-roc {
  color: var(--effective);
}

#hud-time {
  color: var(--text);
}

#hud-speed {
  color: var(--accent);
}

#fpv-host {
  position: fixed;
  top: 0;
  /* JS 会用 hudLayout 推导的 px 值覆盖（恒等于 1.2% 视口宽）；这里是首帧兜底 */
  right: 1.2%;
  width: 200px;
  height: 150px;
  border: 2px solid var(--threshold);
  border-radius: 8px;
  overflow: hidden;
  background: #020617;
  /* 只对尺寸做过渡，且首次布局后才启用：left/top 参与过渡会导致窗口错位 */
  z-index: 6;
}

#fpv-host.animate {
  transition:
    width 0.3s ease,
    height 0.3s ease;
}

#fpv-canvas {
  display: block;
  width: 100%;
  height: 100%;
}

#fpv-readout {
  position: absolute;
  left: 50%;
  bottom: 0;
  transform: translateX(-50%);
  display: flex;
  gap: 0.6em;
  padding: 0.15em 0.4em;
  font-size: 11px;
  background: rgba(2, 6, 23, 0.65);
  border-radius: 4px 4px 0 0;
  color: var(--text);
  white-space: nowrap;
}

#view-bar {
  position: fixed;
  left: 50%;
  top: 12px;
  transform: translateX(-50%);
  display: flex;
  gap: 6px;
  padding: 6px 8px;
  background: var(--panel);
  border: 1px solid rgba(148, 163, 184, 0.25);
  border-radius: 10px;
  z-index: 8;
}

button {
  font-family: inherit;
  font-size: 13px;
  color: var(--text);
  background: rgba(30, 41, 59, 0.85);
  border: 1px solid rgba(148, 163, 184, 0.3);
  border-radius: 6px;
  padding: 5px 10px;
  cursor: pointer;
  transition:
    background 0.15s ease,
    border-color 0.15s ease;
}

button:hover {
  background: rgba(51, 65, 85, 0.95);
}

button.active,
button.primary {
  background: var(--primary);
  border-color: var(--primary);
}

button.accent {
  background: rgba(245, 158, 11, 0.85);
  border-color: var(--accent);
  color: #0f172a;
  font-weight: 600;
}

#timeline-bar {
  position: fixed;
  left: 0;
  top: 0;
  width: 100%;
  height: 100px;
  background: var(--panel);
  border-top: 1px solid rgba(148, 163, 184, 0.25);
  z-index: 8;
}

#timeline-controls {
  position: absolute;
  display: flex;
  align-items: center;
  gap: 10px;
  padding: 0 12px;
}

#speed-host {
  display: flex;
  align-items: center;
  gap: 8px;
}

#speed-slider {
  width: 180px;
  accent-color: var(--accent);
  transition: transform 0.1s ease;
}

#speed-slider.snap-pop {
  transform: scale(1.06);
}

#speed-value {
  font-weight: 600;
  color: var(--accent);
  min-width: 4.2em;
  text-align: right;
}

.speed-end {
  font-size: 11px;
  opacity: 0.75;
}

#speed-hint {
  opacity: 0.7;
  white-space: nowrap;
}

#btn-export {
  margin-left: auto;
  background: rgba(16, 185, 129, 0.85);
  border-color: var(--effective);
  color: #052e16;
  font-weight: 600;
}

#timeline-track-host {
  position: absolute;
  display: flex;
  align-items: center;
  padding: 0 16px;
}

#timeline-track {
  position: relative;
  width: 100%;
  height: 8px;
  background: rgba(148, 163, 184, 0.25);
  border-radius: 4px;
  cursor: pointer;
  touch-action: none;
}

#timeline-progress {
  position: absolute;
  left: 0;
  top: 0;
  height: 100%;
  width: 0;
  background: var(--primary);
  border-radius: 4px;
}

#timeline-thumb {
  position: absolute;
  top: 50%;
  left: 0;
  width: 14px;
  height: 14px;
  margin-left: -7px;
  transform: translateY(-50%);
  background: var(--accent);
  border-radius: 50%;
  pointer-events: none;
}

#panel-host {
  position: fixed;
  left: 12px;
  top: 64px;
  width: 300px;
  max-height: calc(100% - 260px);
  overflow: auto;
  z-index: 9;
}

/* 无命中告警：播放到末帧且 hitLamp=false 时叠加的中央偏上黄字 */
#no-hit-warning {
  position: fixed;
  left: 50%;
  top: 18%;
  transform: translateX(-50%);
  padding: 10px 22px;
  border-radius: 10px;
  background: rgba(15, 23, 42, 0.82);
  border: 1px solid rgba(245, 158, 11, 0.55);
  color: #fbbf24;
  font-size: 20px;
  font-weight: 600;
  letter-spacing: 0.04em;
  pointer-events: none;
  z-index: 15;
}

#no-hit-warning[hidden] {
  display: none;
}

/* 机体高度警告：由 ui/panel.ts 创建并 append 到 #panel-host 末尾（面板底部） */
#uav-warning {
  margin: 8px 0;
  padding: 6px 10px;
  background: rgba(239, 68, 68, 0.12);
  border: 1px solid rgba(239, 68, 68, 0.5);
  border-radius: 6px;
  color: #fca5a5;
  font-size: 12px;
  line-height: 1.4;
}

#uav-warning[hidden] {
  display: none;
}

#export-overlay {
  position: fixed;
  inset: 0;
  display: flex;
  align-items: center;
  justify-content: center;
  background: rgba(2, 6, 23, 0.72);
  z-index: 20;
}

#export-overlay[hidden] {
  display: none;
}

.export-card,
.modal-card {
  min-width: 420px;
  max-width: 620px;
  padding: 20px 24px;
  background: #0b1220;
  border: 1px solid rgba(148, 163, 184, 0.3);
  border-radius: 12px;
  box-shadow: 0 18px 45px rgba(0, 0, 0, 0.5);
}

.export-card h3,
.modal-card h3 {
  margin: 0 0 8px;
  font-size: 18px;
}

.export-card p,
.modal-card p {
  margin: 4px 0;
  font-size: 13px;
  line-height: 1.6;
  color: rgba(248, 250, 252, 0.85);
}

.export-progress {
  margin: 14px 0 6px;
  height: 8px;
  background: rgba(148, 163, 184, 0.25);
  border-radius: 4px;
  overflow: hidden;
}

#export-progress-fill {
  height: 100%;
  width: 0;
  background: linear-gradient(90deg, var(--primary), var(--effective));
  transition: width 0.15s linear;
}

#toast-host {
  position: fixed;
  inset: 0;
  display: none;
  align-items: center;
  justify-content: center;
  background: rgba(2, 6, 23, 0.6);
  z-index: 30;
}

#toast-host.visible {
  display: flex;
}

.modal-actions {
  margin-top: 16px;
  display: flex;
  justify-content: flex-end;
  gap: 8px;
}
```
