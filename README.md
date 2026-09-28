# 无人机遮挡绿色引导灯 —— 几何可行性仿真（RoboMaster 2027 前瞻场景）

> 最后更新：2026-09-13　唯一弹道模型：**simple（5 参数抛物线）**　主场景：**无人机定点悬停 + 飞镖全弹道遮挡**
> 视频形态：**两个独立、帧对帧同步的 1920×1080 @30fps MP4**（全程 overview / 末端 terminal）
> 本文档只描述**当前实际能力**；已删除的历史实现见附录 A。

---

## 0. 完成情况

| 模块 | 状态 | 说明 |
|---|---|---|
| 坐标系与机械数据 | ✅ | 灯心为原点，X 沿飞行方向；发射点由机械数据相减得到 |
| 最简弹道模型 `simple` | ✅ | 发射点 + 落点 + v0 + 仰角 + 减速律 → 解析抛物线 |
| 几何遮挡率 `R_occ` | ✅ | 射线投影 → 阴影凸包 ∩ 灯盘 ÷ 灯盘面积，退化几何受控 |
| 相机光轴与视场 | ✅ | 光轴 = 弹道切线；视场余量 + 第一人称（FPV）成像 |
| 悬停站位与距离扫描 | ✅ | 可行站位区间 standoff ∈ [0.08, 0.16] m（函数保留，默认不调用） |
| **双视频输出** | ✅ | `animation_overview.mp4`（全程 25 m）+ `animation_terminal.mp4`（末端 0.5 m），帧对帧同步 |
| 参数总表 | ✅ | 默认不生成，`--params` 按需导出 `out/parameters.md` / `.csv` |
| 规则合规性声明 | ✅ | 见 §1.3 |
| 六状态质点模型 / 54 组机动矩阵 / los·predictive / 分段 x 轴融合视图 | 🗑 已删除 | 见附录 A |

---

## 1. 项目简介

用**纯 Python**验证一个几何问题：飞镖相机沿弹道 `C(t)` 接近基地时，一台定点悬停的
无人机能否在**灯平面**上遮住绿色引导灯的发光区，并量化遮挡效果。

### 1.1 坐标系（与机械组一致）

以**绿色引导灯几何中心**为原点：`X` 从发射点指向基地、沿飞行方向；`Y` 场地横向；
`Z` 竖直向上。机械装配数据 [mm]：绿灯中心 `(25037.05, 2868.18, 1139.04)`、
发射点 `(0, 0, 570.00)`，两者相减得发射点在灯局部坐标系下的位置：

```
发射点 = (-25.03705, -2.86818, -0.56904) m     # 水平距灯 25.04 m、横向 -2.87 m、低于灯心 0.569 m
```

灯盘位于 `X = 0` 平面、法线 `(-1, 0, 0)`（朝向迎面而来的飞镖）。这里的 Z 是**相对灯心的高度**，不是离地高度。

### 1.2 相机

光轴 **严格等于弹道切线** `dC/dt`（“朝哪飞就看哪”），因此全程姿态随弹道变化。
光轴只用于**视场判定与 FPV 渲染**；几何遮挡率 `R_occ` **只取决于光心位置 C**。

### 1.3 规则与合规性声明（务必阅读）

* 依据截至 **2026-09-11** 公开的《RoboMaster 2027 高校系列赛规则变更前瞻手册》（2026-09-09 发布）建模；
  该手册只公布部分变更，**完整规则以正式版为准**。
* 官方**尚未明确授权**“以遮挡飞镖引导灯为目的的视觉干扰”。本项目定位为**几何可行性研究**，
  **不代表**已获裁判确认的合法战术，也不构成参赛建议。
* `R_occ` 是**几何遮挡率**，**不得**称为“视觉识别失败概率”，也不等价于官方识别失效阈值；
  `R_th` 只是本项目的**分析阈值**。
* 发射点取自机械装配数据（`USER_SPEC`）；灯盘姿态是 `MODELING_ASSUMPTION`。
* 不接入 ROS、官方仿真器、Unity、Isaac Sim、Gazebo，不做空气动力学建模。

---

## 2. 快速开始

```powershell
cd D:\Study\VibeCoding\UAV\rm_uav_occlusion

# 0) 首次准备（已装可跳过）
python -m pip install -r requirements.txt
python -m pip install imageio-ffmpeg        # 视频用内置 ffmpeg，免系统安装

# 1) 清空旧产物（可选）
Remove-Item -Recurse -Force .\out

# 2) 编译检查 + 一键生成（5 张静态图 + 2 个 MP4）
python -m py_compile config.py geometry.py occlusion.py motion.py strategies.py `
    experiment.py validation.py visualize.py params.py camera.py main.py
python main.py
```

实测：编译 `EXIT=0`，`python main.py` 约 **20 s**，产出 7 个文件。

### 常用开关

| 目的 | 命令 |
|---|---|
| 跳过视频（只出静态图，约 6 s） | `python main.py --no-video` |
| 导出参数总表（默认不生成） | `python main.py --params` |
| 换悬停站位到灯平面的距离 | `python main.py --standoff 0.12` |
| 换站位竖直偏置 | `python main.py --offset-v 0.03` |
| 换分析阈值 | `python main.py --R-th 0.9` |
| 换策略（当前只有 hover / fixed） | `python main.py --strategy fixed` |
| 改视频帧率 / 分辨率（默认 30 fps / dpi100=1920×1080） | `python main.py --fps 25 --video-dpi 80` |

---

## 3. 当前弹道模型（simple）

只有 5 个输入参数，其余全部是派生量：

| 输入 | 参数名 | 当前值 |
|---|---|---|
| 初始发射位置 | `DartConfig.launch` | (-25.037, -2.868, -0.569) m |
| 末端命中位置 | `DartConfig.impact` | (0, 0, 0) m（灯心） |
| 初始速度 | `DartConfig.v0_mps` | 22 m/s |
| 初始仰角 | `DartConfig.theta0_deg` | 26° |
| 速度降低率 | `DartConfig.speed_decay_per_s` | 0.55 1/s |

```
几何：z(x) = z0 + x·tanθ0 − g_eff·x² / (2·v0²·cos²θ0)
      g_eff = 2·v0²·cos²θ0·(R·tanθ0 − Δh) / R²        ← 由“抛物线必须穿过落点”反解
时间：v(t) = v0·e^(−k·t)  →  s(t) = (v0/k)(1 − e^(−k·t))  →  位置按弧长反查
```

当前取值下的实测派生量：等效重力 **14.434 m/s²**、弹道弧长 **26.09 m**、
飞行时间 **1.920 s**、末端速度 **7.65 m/s**。

---

## 4. 遮挡与相机

1. 从光心 `C` 向无人机 8 个顶点做射线，与灯平面 `X = 0` 求交；
2. 要求无人机**整体**位于相机与灯平面之间（不满足 → `R_occ = 0` 并显式告警）；
3. 投影点取凸包得到阴影多边形；空/非面状等退化几何按 `0.0` 处理；
4. 与灯圆盘求交：`R_occ = A_交集 / (πr²)`，限制在 `[0, 1]`；有效遮挡判据 `R_occ ≥ R_th`；
5. `T_occ = sum(eff)·dt`，并统计最长连续段与首次遮挡时刻。

**相机**：`forward = dC/dt`，`right/up` 由 `CAMERA_UP_REF`（默认世界 +Z）构造；
据此算出灯心相对光轴的偏角与视场余量，并用 `f = (W/2)/tan(HFOV/2)` 做 FPV 成像。

---

## 5. 悬停站位与结果

站位由 `HoverStationConfig` 描述：
`position = L + standoff·n̂ + offset_u·û + offset_v·v̂`（`v̂` 是灯平面内的竖直方向）。

| 参数 | 当前值 | 备注 |
|---|---|---|
| `standoff_m` | 0.15 m | 到灯平面的距离（CLI `--standoff`） |
| `offset_u_m` | 0.0 | 横向偏置 |
| `offset_v_m` | 0.0 | 竖直偏置；**v=0 表示机身中心与灯心等高** |

站位世界坐标 = **(-0.15, 0, 0) m**。

> `offset_v` 取 0 的实测依据：25 m 弹道下阴影放大率 `k ≈ 1`，阴影中心≈机身自身位置。
> `v = 0.06` 会把阴影中心抬高 6 cm，`R_occ` 从 **0.970 掉到 0.453**（失效，低于 0.7）。

**结果（站位 0.15 m，t ∈ [0, 2.96] s 的有效段）**：

| 指标 | 数值 |
|---|---|
| `T_occ`（`R_th = 0.7`） | 1.880 s（截断前全部时长） |
| `R_occ` 最小 / 均值 / 最大 | 0.970 / 0.996 / 1.000 |
| 灯在相机视场内帧占比 | 100%（最小视场余量 3.00°） |
| 灯心相对光轴最大偏角 | 24.70° |
| 站位可行区间（`R_min ≥ 0.7`） | **standoff ∈ [0.08, 0.16] m** |

---

## 6. 参数总表（真正影响结果的参数）

### 6.1 调参指南（代码里已写详细注释，改这里最稳）

所有调参点都在源码里标了 `# ---- 调参 …` 或 `★`，含义/单位/影响/典型范围都写在注释里。速查：

| 想改什么 | 参数（文件） | 单位 | 影响与注意 |
|---|---|---|---|
| 出膛速度 | `SIMPLE_V0_MPS`（config） | m/s | ★最常改；改大 → 同距离更高更快，等效重力随之反解 |
| 初始仰角 | `SIMPLE_THETA0_DEG`（config） | deg | ★打高/打低主旋钮；当前 26° 是"抛物线正好穿过灯心"的解 |
| 速度降低率 | `SIMPLE_SPEED_DECAY_PER_S`（config） | 1/s | ★`v(t)=v0·e^(−kt)`；改大 → 后段更慢、飞行时间更长 |
| 悬停站位距离 | `HOVER_STATION_STANDOFF_M`（config）/ `--standoff` | m | ★最敏感；可行区间 **[0.08, 0.16] m**，超出即遮挡失效 |
| 站位竖直偏置 | `HOVER_STATION_OFFSET_V_M`（config）/ `--offset-v` | m | ★保持 0 = 机身中心与灯心等高；0.06 会让 R_occ 掉到 0.453 |
| 灯直径 | `LAMP_DIAMETER_MM`（config） | mm | 改变 R_occ 分母；越小越易遮住 |
| 机体尺寸 | `UAV_SIZE_MM`（config） | mm | 越大阴影越大、越易遮；同时改变机身安全边界 |
| 相机 FOV | `CAMERA_HFOV_DEG`（config） | deg | 只影响视场判定与 FPV；不影响 R_occ |
| 分析阈值 | `SimConfig.R_th` / `--R-th` | — | 支持 0.5 / 0.7 / 0.9 |
| 仿真步长 / 时长上限 | `SIM_DT_S` / `SIM_T_END_S`（config） | s | 时长只是上限，主场景会被飞行时间与截断覆盖 |
| 飞行边界 | `DEFAULT_UAV_BOUNDS`（config） | m | 机身包围盒；抬高 z 下界会破坏最优悬停高度 |
| 演示截断距离 | `DART_STOP_DISTANCE_M`（main） | m | 0.20 m → 约 1.86 s；调大则末端细节看不到 |
| 视频帧率 / 分辨率 | `--fps` / `--video-dpi`（默认 30 / 100） | — | 30 fps+100 dpi = 1920×1080 / 56 帧（时长被 `DART_STOP_DISTANCE_M` 截断到 t=1.86 s → round(1.86×30)=56） |
| 视频 A 拉长 / 铺满 | `OVERVIEW_RECT=(-0.41,-0.41,1.82,1.82)`（visualize.py 顶部常量） | — | ★"拉长一点"就调它：等比放大，长度与高度**同时**增长。实测内容占画布 **横 97.5% / 纵 98.9%**（旧值 1.68 时 93.5%×95.1%），弹道自身由 78.4%×64.4% 长到 **86.0%×76.9%**，投影宽高比 **1.75 ≈ 画布 1.78**（等比铺满，无额外各向异性拉伸）。实测梯度：1.68→93.5%×95.1%、1.76→95.7%×97.5%、1.80→96.7%×98.4%、1.82→97.5%×98.9%、1.84→98.4%×99.5%（已贴边）。**⚠ 不要用 `zoom>1`**（会切掉发射段/命中段） |
| 视频 A 盒子形状 | `OVERVIEW_Z_BOX=13.5`（visualize） | — | 盒子 x:y:z = 26:4:13.5；它与 `elev=18°` 一起决定投影宽高比 = 26/(y·sin(elev)+z·cos(elev))，令其 ≈1.78（画布比例）→ z≈13.5。**改大 z** → 弹道更陡更高、横向变窄；**改小 z** → 更"长扁"、上下留白 |
| 视频 A 数据范围 | `OVERVIEW_XLIM / YLIM / ZLIM`（visualize） | m | ★"弹道在盒子里占多满"靠这里（盒子在画布上的大小只由 `RECT`+`Z_BOX` 决定）。当前 = 弹道包络 + 少量余量，x 占 97.8% / y 占 84.4% / z 占 84.8%（旧值 96.3 / 71.7 / 76.7%）。**⚠ 不要小于包络 x −25.04→0、y −2.87→0、z −0.57→2.65**，否则起/终点贴墙 |
| 视频 A 标题/图例位置 | `fig.suptitle(..., y=0.985)` / `fig.legend(bbox_to_anchor=(0.008,0.945))`（visualize） | — | 轴外扩后 `ax.set_title`/`ax.legend` 会被顶出可视区，必须用画布层 API；`y=0.985` 给中文字形顶部留 2~3 px，避免放大后被画布上边缘切掉 |
| 视频 B 观察窗口 | `build_terminal_animation` 里的 `window`（visualize） | m | 0.5 m 立方体；扩大 x 下限可让相机更早入画 |
| 视频 A 视锥长度 / 是否裁剪 | `OVERVIEW_FRUSTUM_LEN_M=1.5`、`OVERVIEW_FRUSTUM_CLIP=True`（visualize） | m, — | 固定 1.5 m；**裁剪开关**决定射线"止于盒壁"（True，推荐）还是"伸出盒子"（False，需 `OVERVIEW_RECT ≤ 1.70`，否则顶部被画布切掉并压住标题）。裁剪只改画面、不改任何遮挡计算 |
| 光轴梳齿密度 | `build_overview_animation` 里的 `1.2` / `0.3`（visualize） | m | 间隔 / 长度 |
| 到位控制器 | `TRACKING_OMEGA_RAD_S` / `TRACKING_ZETA`（config） | rad/s, — | 影响"飞向站位"的过程，不影响稳态遮挡 |

`python main.py --params` 会打印完整总表并写出 `out/parameters.md` / `.csv`
（含参数名、当前值、来源标签、改动方式）。核心参数：

```
几何：LampConfig.center / normal / diameter_mm     UAVConfig.size_m     SimConfig.uav_bounds
相机：CameraConfig.resolution / fov_h_deg          （仅用于 FPV 与视场判定）
运动：SimConfig.dt / t_end                          UAVConfig.max_speed / max_accel
                                                      TRACKING_OMEGA_RAD_S / TRACKING_ZETA（到位控制器）
弹道：DartConfig.launch / impact / v0_mps / theta0_deg / speed_decay_per_s
判定：SimConfig.R_th
站位：HoverStationConfig.standoff_m / offset_u_m / offset_v_m
```

来源标签：`OFFICIAL_2027`、`OFFICIAL_2026_BASELINE`、`ENGINEERING_ASSUMPTION`、
`MODELING_ASSUMPTION`、`USER_SPEC`、`UNKNOWN`。

---

## 7. 输出清单（默认 7 个文件）

| 文件 | 说明 |
|---|---|
| `animation_overview.mp4` | **视频 A（全程）**：25 m 全弹道，**线性 x 轴**（弹道平滑无折角），侧视 `elev=18, azim=-100`、正交投影；3D 轴外扩铺满画布（`add_axes(OVERVIEW_RECT)=(-0.41,-0.41,1.82,1.82)`，内容实测占 **97.5%×98.9%**，`set_box_aspect((26,4,13.5))`，**不用 zoom**）；坐标范围收紧到弹道包络（`OVERVIEW_*LIM`），弹道自身占 **86.0%×76.9%**；含起点黑圆 / 末端深绿 X / 0.5 s 间隔点 / 相机红三角 / 无人机线框 / 灯盘 / LOS / 相机光轴 / 视锥（固定 1.5 m，**按坐标盒裁剪**，不再顶出画布）/ 光轴梳齿；**不画阴影**（留给视频 B）；`R_occ` 与相机坐标写在**画布顶部标题（suptitle, y=0.985）**，图例用 `fig.legend` 固定在左上 |
| `animation_terminal.mp4` | **视频 B（末端）**：末端 0.5 m 立方体窗口（`x∈[-0.4,0.1]`, `y,z∈[-0.25,0.25]`），`elev=35, azim=-135`、透视投影；8 类精细元素全保留（相机、视锥、阴影多边形（按窗口裁剪）、8 条顶点投影线、灯盘 41 个采样点、阴影边界 49 个采样点、LOS、光轴）+ 无人机线框/中心、灯盘/灯心、灯平面参考框 |
| `scene_dart_hover.png` / `scene_3d.png` | 静态 3D 场景（同精细度） |
| `heatmap_lamp_plane.png` | 平行灯平面截面（x = -0.15 m）遮挡率热图 |
| `heatmap_xz.png` / `heatmap_xy.png` | 侧视 / 水平切片热图 |
| `parameters.md` / `.csv` | **仅在 `--params` 时生成**：参数总表 |

**两个视频的规格**：H.264 (High) + `yuv420p` + `+faststart`、**1920×1080**、**30 fps**、
**56 帧（1.86 s）**，且**帧数相同、第 k 帧对应同一仿真时刻**（同一个 `run_dynamic` 结果，
用 `idx = round(k·(n_sim−1)/(n_frames−1))` 重采样）。每次运行覆盖旧文件。

---

## 8. 模块结构

```
rm_uav_occlusion/
├── config.py       参数与来源标签（唯一弹道模型 simple + 站位/仿真/相机配置）
├── geometry.py     射线-平面求交、平面基、线框顶点、阴影投影
├── occlusion.py    R_occ、退化几何保护、有效遮挡判据
├── camera.py       相机位姿（光轴=弹道切线）、视场判定、像面投影
├── motion.py       弹道分发、Vmax/Amax 约束运动、边界裁剪
├── strategies.py   TargetPolicy 接口 + HoverStationPolicy + FixedPointPolicy
├── experiment.py   网格扫描、站位扫描、动态仿真、指标、相机统计
├── validation.py   几何/边界/速度/站位校验
├── visualize.py    热图、静态 3D 场景、双视频（build_overview / build_terminal）
├── params.py       参数总表（`--params` 时调用）
└── main.py         7 步一键流程
```

---

## 9. 附录 A：已删除的历史实现

| 已删除 | 原因 |
|---|---|
| `linear` / `approach_descend` 两种弹道 + 其参数与实验矩阵 | 需求收敛为“唯一最简抛物线” |
| 六状态质点模型（`dart_model.py`：RK4 + 制导） | 主场景只需“穿过灯心的抛物线” |
| `los` / `predictive` 机动策略 | 当前阶段无人机不做机动 |
| 54 组实验矩阵（`build/run_experiment_plan`） | 依赖上面两种弹道，且主流程不调用 |
| `parabolic` 模式整块 | 被 `simple` 取代 |
| **分段 x 轴融合视图**（`fuse_x` / `fuse_points` / `build_fused_scene_animation`） | 分段映射让弹道出现**视觉折角**，且压缩远端；改用“线性 x 轴 + 两个独立视频” |
| GIF 与 PNG 帧序列导出（`export_animation_frames` / `ffmpeg_command_hint`） | 只保留 MP4 |
| FPV 静态拼图 `camera_view_strip.png` | 视频里不需要第一人称拼图 |
| 20+ 个无人调用的工具函数、冗余常量 | 死代码清理 |

> 需要恢复其中任何一项时，请从 git 历史（提交 `7cb41a5` 及之后）取回，不要凭本文档假设它们仍存在。

---

## 10. 附录 B：经验与踩坑

### 10.1 matplotlib 3D
1. **`set_box_aspect(..., zoom>1)` 会裁切数据**：曾为填满画布设 `zoom=1.20/1.85`，导致 25 m 弹道的
   **起始段或命中段被切出画框**（表现为“发射点/落点不见了”）。**结论：不放 zoom**，
   要让内容占满应通过“一格一视频 + 合理坐标范围”实现，本项目最终采用**双视频**方案。
2. **3D 轴在宽矩形里按方形绘制**，加高画布不会让宽面板里的盒子变大。
3. **透视投影压缩远端**：细长场景改用 `set_proj_type("ortho")` 后全程等比例（视频 A 用 ortho，视频 B 用 persp 更自然）。
4. **俯仰角压扁立方体**：`elev=20°` 时 z 向投影只有 `sin20°≈0.34`，120 mm 立方体像长条；改 `elev=35°` 后正常。

### 10.2 模型
1. 报告里的 `γ̇ = α_v − g·cosγ/v` 量纲不一致，物理一致写法是 `γ̇ = (α_v − g·cosγ)/v`。
2. `c_d = 0.02`（1/m，等效阻力参数）**≠** `C_D = 0.47`（CFD 阻力系数）。
3. 报告的 33° 是结构强度分析工况；本场景发射点只比灯心低 0.569 m。
4. 纯追踪制导在末端必然过载失灵，正确做法是“参考路径 + 前视点”。

### 10.3 代码质量
1. **批量删除脚本必须自带校验**：用 `ast` 取 `lineno/end_lineno` 精确切行，删完立刻 `ast.parse` 复核 + 打印字节数变化，才能发现“脚本报成功但文件没变”。
2. **PowerShell heredoc 经管道传给 python 会按 GBK 编码**，中文会变乱码；写中文文件用 `Set-Content -Encoding utf8`（cmdlet 管道）或脚本内 `\uXXXX` 转义。
3. `_default_uav_mass()` 曾 `return` 空导致 `mass_kg=None`；常量曾重复定义两次；均已修。

### 10.4 需求层面
1. **先确认坐标系约定**：X 轴方向、发射点相对灯心的符号一旦搞错，全部结果作废。
2. **“看起来不对”先分清是数据还是渲染**：先打印端点/极值，再改画法。
3. **弹道折角 = 非线性坐标映射**；要同时看“全程 + 末端细节”，正确做法是**两个独立视频**，而不是把两种尺度压进一张图。

### 10.5 视频 A/B 的渲染与调参经验

1. **"铺满画布"有三个旋钮，别用 `zoom`**：① `OVERVIEW_RECT`（等比放大，长度与高度**同时**增长）；
   ② `OVERVIEW_Z_BOX`（改盒子形状，决定投影宽高比）；③ `OVERVIEW_XLIM/YLIM/ZLIM`（收紧数据范围，
   让弹道在盒子里占得更满，但不改盒子在画布上的大小）。
2. **画布比例 = 命中比例**：画布 1920×1080 = 1.78:1，所以盒子投影宽高比也 ≈1.78（`OVERVIEW_Z_BOX≈13.5`）
   时横纵同时铺满，且不引入额外的各向异性拉伸；只加大 `z_box` 会"只长高、不长长"。
3. **轴外扩后标题/图例必须放到画布层**：`ax.set_title` / `ax.legend` 会被顶出可视区，
   改用 `fig.suptitle(..., y=0.985)` 与 `fig.legend(bbox_to_anchor=(0.008, 0.945))`。
4. **视锥会溢出盒子**：mplot3d **不裁剪**超出坐标范围的数据，1.5 m 视锥在弹道顶点附近会顶出画布
   并压住标题 → 用 `clip_segment_to_box()` 裁到盒壁（`OVERVIEW_FRUSTUM_CLIP=True`）；
   设为 `False` 时若 `OVERVIEW_RECT > 1.70`，顶部必被画布切掉。
5. **帧数由截断后的时长决定**：`main.py` 先 `truncate_run_before_contact(DART_STOP_DISTANCE_M)`，
   再 `n_frames = round(t_end·fps)` → 1.86 s × 30 fps = **56 帧**（不是 1.92 s 对应的 58 帧）。
6. **帧对帧同步靠同一份仿真结果**：两段视频共用一次 `run_dynamic` 的输出，按
   `idx = round(k·(n_sim−1)/(n_frames−1))` 重采样，第 k 帧严格对应同一 `t` 与同一 `R_occ`。
7. **`UAV_SIZE_MM` 同时决定几何尺寸与飞行边界**：边界 = `DEFAULT_UAV_BOUNDS` ∓ 半边长，
   改尺寸会连带挪动可行站位区间。
8. **`SIM_DT_S` 与到位控制器稳定性**：步长过大时 `Kd·dt`、`Kp·dt²` 越过稳定区间会让
   "飞向站位"段发散（悬停稳态不受影响）。
