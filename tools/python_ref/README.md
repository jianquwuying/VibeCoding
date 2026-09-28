# Python 参考实现（vendored copy）

本目录是 `../rm_uav_occlusion` 在 **M2 + v3 时点**的 vendored copy。
**所有 M2 / v3 改动只在本副本进行**，原项目不再同步。

## 与上游的差异（M2 + v3）

| 文件 | 改动 |
|---|---|
| `config.py` | v0 22→20、θ0 26→30；`SIM_DT_S` 0.02→0.01、`SIM_T_END_S` 3.0→5.0；新增 `GRAVITY_MPS2 / MAX_TURN_RATE_DPS / HIT_DISTANCE_M / LAMP_Y_RANGE_MM / LAMP_MOVE_START_S / LAMP_MOVE_END_S / LAMP_SPHERE_RADIUS_M`；`SimConfig` 增 `lamp_target_y_m`；**删除** v1 解析抛物线族（`_simple_table_cached` 与 `DartConfig` 的 `_simple_table / effective_g_mps2 / simple_flight_time / simple_path_len_m / simple_position / simple_velocity / _simple_g_eff / _simple_point_at`）；`DartConfig.t_end` 改为返回积分时域上限 `SIM_T_END_S` |
| `motion.py` | **新增** `lamp_y_at / lamp_vy_at / lamp_center_at / is_fully_visible(J2) / step_dart_m2(六步) / segment_to_point_distance / uav_step`；**删除** v1 的 `dart_camera_trajectory / dart_camera_velocity`（M2 轨迹由数值积分给出，无解析解） |
| `geometry.py` | 新增 `los_basis(C, L)` |
| `occlusion.py` | `compute_occlusion` 改签名：`(C, uav_center, uav_size, lamp_center, los_hat, lamp_polygon_2d, lamp_area_m2)`；投影平面改为"过灯心且垂直于视线" |
| `camera.py` | `image_space_occlusion` 增加 `los_hat` 参数，圆盘径向基改用 `plane_basis(los_hat)`；**删除**依赖 v1 抛物线的 `camera_pose / in_fov`（功能分别由 `pose_from_position_and_axis` 与 `fov_margin_deg` 覆盖） |
| `experiment.py` | **重写**为 M2 主循环：`hover_station_position_at / run_m2 / occlusion_margin / compute_metrics`；删除 v1 的网格扫描、站位扫描、相机统计与 `strategies` 依赖 |
| `strategies.py` | 原样复制，M2 主循环不再使用（保留以保持与上游完整一致） |

## 已删除（v1 遗留，与 M2 矛盾）

- **v1 解析抛物线全体**：`config.py` 的 `DART_MODE_PARABOLIC`、`PARABOLIC_LAUNCH_M /
  PARABOLIC_IMPACT_M / PARABOLIC_FLIGHT_TIME_S / PARABOLIC_APEX_RISE_M`、
  `ARC_LENGTH_SAMPLES`、`_arc_length_table / _arc_length_table_cached`，以及 `DartConfig`
  的 `is_parabolic / parabolic_coeffs / t_apex / path_param_apex / apex_z_m / apex_height_m /
  parabolic_path / arc_length_to / path_param_of_time / path_length_m / speed_mps /
  parabolic_position / parabolic_velocity / impact_speed_mps`。
  （`DartConfig.launch` 的默认值改由 `LAUNCH_REL_LAMP_M` 提供，数值不变；
  `DART_MODES` 现在只有 `'simple'`。）
- `motion.dart_camera_trajectory`、`motion.dart_camera_velocity`（唯一消费者是下面的
  `camera_pose / in_fov`）。
- `camera.camera_pose`、`camera.in_fov`（已被 `pose_from_position_and_axis` /
  `fov_margin_deg` 覆盖）。

> 保留删除**不影响黄金基准**：`gen_golden.py` 从不调用这些符号，删除前后
> `tests/golden/*.json` 逐字节一致（已实测）。

## 保留但未被 `gen_golden` 调用（独立参考验证用）

以下符号**刻意保留**：它们不参与 M2 主调用链，但作为上游能力的参考实现（视锥几何、
参数来源元数据、便捷构造器、v1 PD 控制律、策略骨架）仍有对照价值。
静态扫描会显示"未被引用"——**这是预期**，不是漏删。

> **统计口径**：从"`gen_golden.py` 的模块/函数 + 各模块 import 期代码"出发，按**名字级可达性闭包**
> 判定；不可达者即为下表成员。当前实测 **57 个登记符号** = 56 个 AST 候选（146 个候选中 90 个可达）
> + 1 个小写类型别名 `motion.BoundsLike`（`BoundsLike = Mapping[str, Tuple[float, float]]`，
> 是 `clamp_to_bounds` / `update_uav_state` 的形参类型，必须与 `update_uav_state` 同去同留）。
> 已知口径偏差：`config.DART_MODES` 只因 `DartConfig.__post_init__`（dunder，未纳入容器）显示不可达，
> 实际在每次构造 `DartConfig` 时被调用；`AXES`/`CameraPose.distance` 等属"仅被本表成员引用"。
>
> 分组计数：**4(camera) + 12(config 模块级) + 17(config 成员) + 2(geometry) + 9(motion) + 13(strategies) = 57**
> （motion 的 9 = `update_uav_state` + 5 helper + `AXES` + `interp_keypoints` + `BoundsLike`）。
> `config.py` 的另三个小写别名 `Vector3` / `Interval` / `Bounds` 均有活跃消费者（注解与 `SimConfig`），**不在登记内**。
>
> 登记集合的权威数字是 **57**。若后续复扫得不同数字，按"AST 扫描口径 + 本表逐项核对"判断，不要固守数字。

| 文件 | 符号 | 保留理由 |
|---|---|---|
| `camera.py` | `frustum_corner_rays`、`CameraPose.contains` / `CameraPose.distance` / `CameraPose.off_axis_angle_deg` | 视锥/视场几何，可独立于仿真模型做成像验证（只依赖 `pose` + `camera`） |
| `config.py` | `default_lamp` / `default_camera` / `default_uav` / `default_dart` / `default_sim` | 便捷构造器，API 面的一部分 |
| `config.py` | `scene_source_table`、`SOURCE_LEGEND`、`LampConfig.source_rows` / `HoverStationConfig.source_rows` / `CameraConfig.source_rows` / `UAVConfig.source_rows` / `DartConfig.source_rows` / `SimConfig.source_rows` | 参数来源元数据（上游 `params.py` 报告功能），与仿真模型无关 |
| `config.py` | `A_MAX_OPTION`、`INITIAL_NEAR_LAMP`、`LAMP_Y_RANGE_MM`、`SIMPLE_V0_MPS`、`DEFAULT_HOVER_STATION` | 常量/枚举留档（`DEFAULT_HOVER_STATION` 被 `strategies.py` 使用） |
| `config.py` | `HoverStationConfig.from_distance`、`UAVConfig.weight_from_size_mm / mass_valid_size_range_mm / half_size_m / volume_m3`、`SimConfig.n_steps / time_array / bounds_array` | 站位反解与质量/时域辅助 API |
| `geometry.py` | `plane_local_to_world`、`uav_edges` | `plane_local_coords` 的逆变换；线框棱索引（与 TS 的 `uavEdges` 对称） |
| `motion.py` | `update_uav_state` + `clamp_to_bounds` / `limit_velocity_change` / `limit_speed` / `limit_acceleration` / `tracking_gains` + `BoundsLike` | v1 的 **3D PD 控制律**，作为"M2 时间最优 `uav_step`"之外的另一套参考实现；两套模型**不共用代码路径** |
| `motion.py` | `interp_keypoints` | 关键点分段线性插值（"关键点轨迹接口"的求值器；M2 无解析轨迹故当前无调用） |
| `config.py` | `DartConfig.is_simple` / `DartConfig.keypoints` / `DartConfig.t_start` | **关键点轨迹接口**（`keypoints` 被 `DartConfig.source_rows` 打印；`is_simple`/`t_start` 原本只被已删除的 `dart_camera_*` 调用）。四个符号拆散没有意义，整体保留 |
| `strategies.py` | `TargetPolicy`（接口）/ `HoverStationPolicy` / `FixedPointPolicy` 三个策略类**及其 `describe` / `target` / `target_velocity` 成员**、`POLICY_NAMES`、`make_policy` | 策略骨架（机动能力的扩展点）；策略类成员逐一登记，不做隐式归并 |

## 何时需要重新生成黄金基准

改动本目录任何**数值**（常量、公式、浮点运算顺序）后，必须在 web 项目根目录执行：

```bash
npm run golden      # 重新生成 tests/golden/*.json
npm run test        # parity 测试会校验 TS 与 Python 是否逐点一致
```

`tools/gen_golden.py` 通过 `DEFAULT_PYTHON_DIR = HERE / "python_ref"` 引用本副本。
