# configs/ 分层配置与字段详解

本文是 `configs/README.md` 的展开：README 讲分层结构、各选择器字段的消费现状和 gate 的设计动机（"为什么"）；本文逐层、逐文件、逐字段讲**每个参数是什么意思、被谁读取、和其他层怎么组合**（"是什么"）。解析逻辑的唯一事实源：

- defaults / scenario / experiment 三层：`include/runtime/config.hpp` + `src/runtime/config.cpp`（yaml-cpp）
- rig 层：`schemas/proto/uw/domain/calibration.proto`（直接解析成 `RigCalibrationSnapshot` protobuf 消息，没有平行手写 struct）

本文与代码不一致时，以代码为准。

---

## 1. 四层是怎么叠起来的

### 1.1 每层一句话

| 层 | 回答的问题 | 内容性质 | 加载成什么 |
|---|---|---|---|
| `defaults/` | 平台基线参数是什么 | 算法/工程默认值，**不含**任何具体机体、场景信息 | `PlatformDefaultsConfig` |
| `rig/` | 这台机体"长什么样" | 标定唯一事实源：传感器外参/内参/噪声模型/时钟偏移 | `RigCalibrationSnapshot`（proto） |
| `scenario/` | 跑什么数据 | 轨迹几何、噪声、seed、声呐目标（合成数据生成参数） | `ScenarioConfig` |
| `experiment/` | 这次怎么跑 | 组装上面三层 + 选算法变体 + 定验收 gate + 定输出 | `ExperimentConfig`（内含全部三层） |

记忆法：**defaults = 算法基线，rig = 硬件，scenario = 数据，experiment = 一次实验的组装说明书**。只有 experiment 文件是直接喂给程序的入口（`--experiment <path>`），另外三层都是被它引用的组件。

### 1.2 "叠加"的真实语义（最容易误解的点）

"`defaults → rig → scenario → experiment` 四层叠加"**不是**"同一张配置表的四次整体覆盖"。真实机制（见 `LoadExperimentConfig`，`src/runtime/config.cpp`）：

1. experiment 文件用根层的 `rig:` / `scenario:` / `defaults:` 三个 key **引用**另外三个文件。路径相对 **`configs/` 目录**（experiment 文件的祖父目录）解析，不是相对 experiment 文件自己所在目录——`rig: rig/example_auv.yaml` 而不是 `rig: ../rig/example_auv.yaml`。
2. **rig 和 scenario 两个维度互不覆盖**：它们字段完全不重叠（一个管标定、一个管数据），加载后分别装进 `ExperimentConfig::rig` / `::scenario`，是"正交组合"而不是"后者盖前者"。
3. 真正的**字段级覆盖**只发生在 experiment 根层 → defaults 层：
   - `gates:` 块**逐字段**覆盖（写了的覆盖，没写的沿用 defaults 值），不是整块替换；
   - `estimation.solver`、`frontends.*`、`estimator_mode`、`map_backend`、`output.write_run_manifest` 也都是"experiment 写了才算"。
4. defaults YAML **不必写全**所有可配字段：没写的字段回落到 `config.hpp` 里各 struct 的默认值（例如 `loop_closure.*` 除 enabled 外的全部参数、`visual_odometry`、`stereo_matching`、`frontends.stereo_rectification` 都不在 `platform.yaml` 里，全走结构体默认值）。
5. 最高优先级是显式 CLI 参数：如 `--seed` 覆盖 scenario 层的 `seed`。

```
CLI 显式参数（--seed 等；最高优先级，仅覆盖 scenario 少数字段）
   │
experiment/*.yaml              ← 入口：--experiment <path>
   ├─ rig: / scenario: / defaults:      三层引用（路径相对 configs/）
   ├─ frontends.{optical,sonar,landmark_detector} / estimator_mode /
   │     map_backend / estimation.solver   算法选择（覆盖 defaults 同名字段）
   ├─ gates:                            验收 gate（逐字段覆盖 defaults 层）
   └─ output: / factor_builders:        输出与声明性信息（factor_builders 不被解析）
   │
   ├──→ defaults/*.yaml   平台基线；未列出的键用 config.hpp 结构体默认值
   ├──→ rig/*.yaml        正交维度：标定 → RigCalibrationSnapshot (proto)
   └──→ scenario/*.yaml   正交维度：数据 → ScenarioConfig
```

### 1.3 谁消费哪些层

| 消费方 | 读了什么 | 没读什么 |
|---|---|---|
| `apps/synth_bag_gen`（生成合成 bag） | scenario 全部字段（轨迹/噪声/目标/seed）；rig 的 `cameras`（当作合成图像的渲染参数）、`frame_tree`、`imu_noise`；以及 `estimator_mode` 的一个特定用途——等于 `imu_preintegration` 时才生成 `/raw/imu` 流 | defaults 的算法字段、gates |
| `apps/replay_demo`（回放求解） | 全部。加载后立即调 `ValidateExperimentConfigSelections()`：任何算法选择字段写了不认识的值，启动即报错退出（不会静默按硬编码管线跑）；gates 在求解完后逐条检查，任一不满足 exit 2（轨迹/manifest 照常写出，方便失败时排查） | — |
| `RunManifest` | 每次运行都会产出不可变 manifest（`include/runtime/run_manifest.hpp`），记录实际生效的配置/标定/代码哈希 | — |

两个关键后果：

- rig 的 `cameras` 块有**双重角色**：既是标定，又是 `synth_bag_gen` 的合成图像生成参数（详见 §3.2）。
- `factor_builders:` 列表和 `output.trajectory_format` 目前**不被 loader 解析**，是声明性文档：写出来表达"这个实验的管线用哪些因子"，但实际构造哪些 factor 由 `replay_demo` 按 bag 里实际存在的证据话题和 `estimator_mode` 决定（例如真实 bag 没有声呐话题时，`sonar_range_v1` 自然不出现——见 `real_holoocean_vo.yaml` 头注释）。

---

## 2. defaults/ 层逐字段

两份文件：`platform.yaml`（所有实验的共享基线）和 `platform_loop_closure.yaml`（唯一区别是开回环闭合）。

### 2.1 platform.yaml

```yaml
estimation:
  solver: gauss_newton_v1        # 求解器标识。唯一受支持值：Eigen 手写 Gauss-Newton/LM。
                                 # experiment 根层 estimation.solver 可覆盖（方便基准对比）。
  max_iterations: 30             # LM 外层迭代上限。
  initial_lambda: 1.0e-3         # Levenberg-Marquardt 初始阻尼系数。
  warmup_seconds: 0.0            # 开局前 N 秒的 keyframe 整体不进图（0 = 不启用）。
```

- `warmup_seconds` 的语义：warmup 窗口内的 keyframe 只做 dead reckoning（继续吃 relative-pose 因子）、暂不融合 sonar/depth 这类绝对参考因子；第一个存活下来的 keyframe 成为新的 fixed anchor（实现见 `apps/replay_demo.cpp`）。动机是同类 SVIn+ HoloOcean 部署里"IMU 偏置没收敛前丢弃早期帧"的工程经验。

```yaml
reliability:
  default_sqrt_information:      # 各证据类型的 sqrt-information（≈1/σ）上限 cap。
    relative_pose:
      translation: 20.0          # 平移/旋转拆成两个独立上限：VO 前端现在带真实 6x6 协方差，
      rotation: 20.0             # 单一标量会让两者互相拖累对方的量级。
    sonar_range: 15.0
    depth: 20.0
```

- 语义：这是架构文档第 8.4 节 `final_information = min(learned, physical, calibration, cross_modal)` 的 v1 简化——只有固定常数上限这一路，真正的多路 cap 是后续工作。cap 越大 = 该证据最多可以被信得越"硬"（σ 下限越小）。

```yaml
frontends:
  sonar_cfar:                    # 声呐 CFAR 前端（frontends::SonarFrontendConfig）。
    training_cells: 16           # CA-CFAR 训练单元数（须为偶数）：待检单元两侧各 8 个
                                 # range bin 用来估计背景噪声水平。
    guard_cells: 4               # 保护单元数（偶数）：紧贴待检单元、不参与背景估计，
                                 # 防止目标能量泄漏进噪声估计。
    probability_false_alarm: 0.01  # Pfa，决定自适应阈值的放大因子；越小阈值越高。
    detector_threshold: 50       # 附加强度下限（0–255）：CFAR 判为目标但原始强度低于
                                 # 此值的仍丢弃（镜像 imaging_sonar.py 的做法）。
    dbscan_eps_m: 0.20           # 极坐标→笛卡尔后 DBSCAN 聚类的邻域半径（米）。
    dbscan_min_samples: 2        # 成簇最少点数；不足的点当噪声丢弃。
    default_range_sigma_m: 0.05      # 声呐证据的默认量测噪声 σ，喂给下游因子。
    default_bearing_sigma_rad: 0.01
```

- 这个块启用了 `RejectUnknownKeys`：写不存在的 key 会直接抛错，不是静默忽略。

**defaults 层还能配、但 platform.yaml 里没写**（全部走结构体默认值，需要时在 defaults 文件里加即可）：

| 键（defaults YAML 根层） | 控制什么 | 默认值 |
|---|---|---|
| `visual_odometry` | VO 前端健康度：`max_consecutive_failures`（连续失败几次放弃）、`max_condition_number`、`residual_variance_floor_m2`、`max_inlier_rmse_m`（≤0 = 关） | 3 / 1e8 / 1e-8 / +inf |
| `stereo_matching` | 双目块匹配过滤：`min_texture_variance`、`min_uniqueness_margin`、`left_right_max_diff_px` | 25 / 2.0 / 1.0 |
| `frontends.stereo_rectification` | OpenCV 双目 rectify：`alpha`、`crop_policy`（full_canvas / common_valid_roi）、`frame_suffix` | 0 / full_canvas / _rectified |
| `loop_closure` | 回环闭合，见 §2.2 | enabled: false |
| `gates` | 验收 gate 的 defaults 层基值（见 §5.3） | 只开 require_converged |

### 2.2 platform_loop_closure.yaml

与 `platform.yaml` 逐字段一致，唯一区别：

```yaml
loop_closure:
  enabled: true
```

为什么独立成一份文件而不是往 `platform.yaml` 里加：`platform.yaml` 是其余所有 experiment 的共享默认，**"回环默认关闭"本身就是保证既有实验零行为变化的机制**，不应该被一次性的对比实验改掉。`loop_closure` 是 defaults 层字段——想开关它就换 experiment 里引用的 defaults 文件，而不是在 experiment 里覆盖（experiment 层没有它的覆盖入口）。

只写 `enabled: true` 时，其余 `LoopClosureConfig` 字段全部用结构体默认值：

| 字段 | 默认 | 含义 |
|---|---|---|
| `candidate_search_radius_m` | 3.0 | 位姿邻近检索半径。v1 刻意用位姿邻近而不是 DBoW2 外观检索（`LoopClosureFrontend` 头注释讲了这条范围边界的理由）。**不要为让合成场景变绿而放宽**——见 `experiment/synthetic_loop_closure_vo_enabled.yaml` 头注释记录的对照实验（放宽到 8~20 后 ATE 从 1.26m 恶化到 9.24m）。 |
| `min_keyframe_index_gap` | 15 | 候选 keyframe 对的最小序号差，排除时间近邻。 |
| `max_accepted_translation_m` | 5.0 | 对 RANSAC 恢复出的相对位姿做 sanity gate（平移模长上限）。 |
| `max_accepted_rotation_rad` | 0.6 | 同上，旋转角上限。 |
| `min_landmarks_for_pose` | 3 | 恢复相对位姿所需的最少 landmark 数。 |
| `max_loop_edges_per_keyframe` | 1 | 每个 keyframe 最多接受几条回环边。 |
| `huber_delta` | 1.5 | 回环边白化残差的 Huber 阈值（透传给 `GaussNewtonOptions::huber_delta`），压错误回环边的影响。 |

使用方式：`experiment/synthetic_loop_closure_vo.yaml`（引 platform.yaml，关）和 `synthetic_loop_closure_vo_enabled.yaml`（引 platform_loop_closure.yaml，开）除 defaults 外完全一样，同一份 bag 对比两者 ATE 就是回环闭合的端到端效果验证。

---

## 3. rig/ 层逐字段（RigCalibrationSnapshot）

rig 描述**一台具体机体**：传感器装在哪（外参）、每个传感器的模型（内参/噪声）、时钟偏移。字段名对应 `schemas/proto/uw/domain/calibration.proto`；这里是它的唯一人可读来源（标定唯一事实源，架构文档第 7.1 节）。

以 `example_auv.yaml` 为例：

```yaml
calibration_version: "example_auv_v2"   # 标定快照版本字符串（进 RunManifest）。

frame_tree:                # 以 base_link 为根的外参树。每条边是 parent→child 的
                           # 4x4 齐次变换（row-major；平移在前三行第四列）。
                           # 机体系约定：x 前 / y 左 / z 上（与全仓库一致，无欧拉角）。
  - parent_frame: base_link
    child_frame: camera_left_link
    transform_row_major: [1,0,0,0.15, 0,1,0,0.06, 0,0,1,0, 0,0,0,1]   # 左相机：x+0.15, y+0.06
  ...                      # camera_right_link / sonar_link / imu_link 同理。
                           # 两个相机 link 位置的差 = 立体基线（占位值是纯 y 轴 0.12m）。
```

### 3.1 各块含义

| 块 | 字段 | 含义 / 注意点 |
|---|---|---|
| `cameras` | `sensor_id` / `width` / `height` / `k_matrix_row_major`（3x3 内参）/ `distortion`（plumb-bob 5 参数）/ `distortion_model` | **双重角色**：既是回放侧标定，又被 `synth_bag_gen` 直接当作合成图像的渲染参数——改这里的数字会改变合成 demo 的图像和 ATE 基线（README「运行端到端 Demo」在跟踪的数字）。真实标定值放 `example_auv_real_camera.yaml`，不要覆盖合成占位值。畸变模型全仓库只有 plumb-bob 一种。 |
| `time_offset_seconds` | 每传感器一个数 | 硬件时钟偏移，符号约定 `t_reference = t_sensor_capture + time_offset_seconds[sensor_id]`；上限 ±10s（`kMaxAbsoluteSensorTimeOffsetSeconds`，时钟校准只该修传感器级 skew，不该桥接不同纪元）。 |
| `time_offset_provenance` | 每传感器一个字符串 | 偏移值是怎么来的（`measured:simulation` / `measured:calibration-session` / `measured:system-clock`），审计用。 |
| `vehicle_state_sensors` | sensor_id 列表 | 哪些话题提供机体状态真值（`rov-state`）。 |
| `imu_noise` | `sigma_gyro_c` / `sigma_accel_c`（角速度计/加速度计连续时间噪声密度）、`sigma_gyro_bias` / `sigma_accel_bias`（初始偏置先验）、`sigma_gyro_bias_walk_c` / `sigma_accel_bias_walk_c`（偏置随机游走密度——与初始偏置先验是**不同语义、不互相回退**）、`rate_hz`、`gravity_mps2` | IMU 预积分因子和合成 IMU 流共用的噪声模型。三份 rig 刻意保持同一组值，保证换 rig 不改变 IMU 流的一个字节（`synthetic_imu_preintegration_camera.yaml` 的对照测试在把关这条性质）。 |
| `sonar_beam_models` | `horizontal_fov_rad`（水平视场 2.09 rad ≈ 120°）、`elevation_aperture_rad`（俯仰孔径 0.19 rad）、`range_resolution_m`、`nominal_speed_of_sound_mps`（水中 1500）、`sonar_enabled` | 声呐成像几何模型。 |
| `depth_models` | `noise_sigma_m` / `depth_enabled` | 深度计噪声 σ。⚠️ 符号陷阱：`PressureDepthMeasurement.depth_m` 是**正向下**的水深；world/body frame 是 Z-up，所以位姿 z 用 `-depth_m`。它跟光学帧 `FusedDepthMeasurement.depth_m`（正向前）同名不同义，画在同一张图：`docs/frames-and-sign-conventions.svg`。 |

### 3.2 三份 rig 文件的差异与用途

| 文件 | 是什么 | 谁在用 |
|---|---|---|
| `example_auv.yaml` | 合成 demo 的占位标定：640x480、fx=420 的相机，纯 y 轴 0.12m 基线（近平行基线） | 所有 `synthetic_*` / `acoustic_optic_demo` experiment |
| `example_auv_sonar_only.yaml` | 同上但**去掉相机**。用途：两个 app 都用 `rig.cameras_size() > 0` 判断要不要跑逐像素的声光稠密路径（12 帧就能产出 340 万 map points），无相机的 rig 让求解器规模基准不被稠密视觉处理淹没 | `synthetic_imu_preintegration.yaml`、求解器规模基准 |
| `example_auv_real_camera.yaml` | 真实标定值（2026-08-21 对真实 HoloOcean 场景棋盘格标定，亚像素 rms）：512x512、fx≈250、真实基线含 15-17% 的 x/z 分量、非零畸变 | `real_holoocean_vo.yaml`（真实数据管线）。头部注释列了**仍未验证**的项（相机相对 base_link 的绝对安装位置仍是占位值） |

---

## 4. scenario/ 层逐字段

scenario 描述"跑什么数据"，字段与 `ScenarioConfig`（`config.hpp`）一一对应，是 `apps/synth_bag_gen` 生成合成 bag 的全部输入。**不描述算法**。显式 CLI 参数（`--seed` 等）会覆盖这里的同名字段。

```yaml
seed: 42                 # 全局随机种子，确定性回放的基础。synth_bag_gen 内部按
                         # "每种噪声用途一条独立 salted RNG 流"拆流（MakeStreamRng），
                         # 所以改一处噪声不会错位其他流的序列（历史坑见 CLAUDE.md）。
num_keyframes: 12        # keyframe 数量。
radius_m: 8.0            # 轨迹几何：机体沿半径 8m 的圆弧运动……
arc_radians: 1.4         # ……总共扫过 1.4 rad ≈ 80° 短弧。
depth_m: 12.0            # 深度 12m（正向下水深；位姿 z = -12）。

noise:                   # 三路噪声各自的 σ，各自独立 RNG 流：
  relative_pose_noise_m: 0.02       # 烘焙进 bag 的相对位姿证据（black_box_vio 读的就是它）
  sonar_range_noise_m: 0.03         # 声呐 range 量测
  sonar_bearing_noise_rad: 0.01     # 声呐 bearing 量测

sonar_targets_world:     # 世界系（Z-up，所以 z 是负数）里的声呐点目标。
  - [2.0, 3.0, -13.0]    # 每个目标会被 synth_bag_gen 画进声呐量测；是否同时进相机
  ...                    # 视场由真实几何决定——目标数量/位置还会影响 RNG 流的消耗
                         # 次数（见 CLAUDE.md「RNG 拆流」条目），改它 ATE 会变。
```

合成视觉 landmark 云是**按世界坐标生成一次、每个 keyframe 复用同一份**的（不是逐帧重新生成）——这是"转一圈回到起点能看到同一批 landmark"、回环闭合场景成立的前提。

### 4.1 三份 scenario 文件的差异

| 文件 | 与 synthetic_smoke.yaml 的唯一区别 | 存在的理由 |
|---|---|---|
| `synthetic_smoke.yaml` | —（基线） | 80° 短弧、3 个声呐目标。经真实几何计算，这 3 个目标**都不在相机窄视场内**——所以引用它的 experiment 必须关掉声光 gate（§5.3）。 |
| `acoustic_optic_demo.yaml` | 目标换成 1 个，位置放在 kf0 相机正前方 | 全仓库唯一"目标同时进相机窄视场 + 声呐宽视场"的场景，专门让真实的 ACCEPTED 声光关联可达。 |
| `synthetic_loop_closure.yaml` | `arc_radians: 2*pi`、`num_keyframes: 48` | 整整一圈回到起点，landmark 云真正"视觉重访"；48 帧保持相近角密度，且首尾序号差 47 远超 `min_keyframe_index_gap=15`，不会被时间邻近门槛误伤。 |

---

## 5. experiment/ 层逐字段

experiment 是唯一直接喂给程序的文件。通用骨架：

```yaml
rig: rig/example_auv.yaml            # 三层引用，路径相对 configs/
scenario: scenario/synthetic_smoke.yaml
defaults: defaults/platform.yaml

frontends:                           # 算法选择（见 §5.2 现状表）
  optical: stereo_depth_frontend_v1
  sonar: sonar_cfar_frontend_v1
  landmark_detector: bright_blob     # 可省略；默认 bright_blob

factor_builders:                     # ⚠️ 声明性文档，loader 不解析（§1.3）
  - relative_pose_v1
  - sonar_range_v1
  - depth_v1

estimator_mode: black_box_vio        # 相对位姿来源（历史字段名）
map_backend: submap_point_cloud_v1   # 预留的地图实现选择

output:                              # write_run_manifest 被解析；trajectory_format 不解析
  trajectory_format: tum
  write_run_manifest: true

gates:                               # 验收 gate，逐字段覆盖 defaults 层（§5.3）
  require_converged: true
  ...
```

### 5.1 消费与校验

`apps/replay_demo` 加载后立即调 `ValidateExperimentConfigSelections()`：下表里任何一个字段写了不认识的值，**启动即报错退出**——关掉"配置存在但不驱动实现"风险里"未识别的算法选择必须启动失败"这一半。

### 5.2 算法选择字段现状

| 字段 | 合法值 | 真的驱动分支？ |
|---|---|---|
| `estimator_mode` | `black_box_vio` / `stereo_landmark_vo` / `imu_preintegration` | ✅ **选择相对位姿证据来源**（历史字段名，不切换求解器）：`black_box_vio` 从 bag 的 `/evidence/relative_pose` 读预生成量测；`stereo_landmark_vo` 用 `stereo_landmark_vo_frontend` 从 `/raw/camera/left,right` 实时算（需要 rig 带相机）；`imu_preintegration` 走 IMU 预积分（synth_bag_gen 也用它决定是否生成 `/raw/imu`）。三条路径喂同一个求解器。 |
| `frontends.landmark_detector` | `bright_blob`（默认）/ `harris_corner` | ✅ `stereo_landmark_vo` 前端内部选检测器：`bright_blob` 是给合成高亮方块调的固定阈值检测器；`harris_corner` 给真实相机画面（没有孤立高亮块可找）。 |
| `estimation.solver`（defaults 层 `estimation.solver`，experiment 根层可覆盖） | `gauss_newton_v1` | ✅ 但只有这一个实现（Eigen 手写 GN/LM；Ceres 适配器已随 2026-09 精简移除）。 |
| `frontends.sonar` | 仅 `sonar_cfar_frontend_v1` | fail-fast 单值（还没有第二条实现可切换）。 |
| `frontends.optical` | 仅 `stereo_depth_frontend_v1` | fail-fast 单值。 |
| `map_backend` | 仅 `submap_point_cloud_v1` | fail-fast 单值；预留的地图实现选择。注意 `SurfelMap` 虽存在但不是这个字段的第二个可选值。 |

### 5.3 gates（验收 gate）

`gates:` 可写在 defaults 层（基值）和 experiment 根层（逐字段覆盖）。`platform.yaml` 没写 gates，所以基值就是结构体默认：**除 `require_converged` 默认开外全部默认关**，每个 experiment 按实测结果自己决定开不开、开多严。求解完后 `application::EvaluateReplayGates` 逐条检查，任一不满足 exit 2（输出照常写出，只影响退出码）。

| gate | 默认 | 含义 |
|---|---|---|
| `require_converged` | **true** | 求解器不收敛任何时候都不可接受（`stalled` 会让 replay_demo 直接 gate failure）。 |
| `max_ate_rmse_m` | -1（关） | ATE rmse 上限（米）。开启时阈值应留约 2 倍余量而不是卡实测值，避免正常跨 seed 波动造成 CI flaky。 |
| `min_matched_ate_poses` | 0（关） | 参与 ATE 对齐的位姿数下限。 |
| `require_nonempty_map` | false | 只要求"有地图内容"——optical-only 深度点就算满足，不要求声光关联成功。 |
| `min_acoustic_optic_accepted` | 0（关） | **专盯声光关联本身**：`contribution_mask == DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC` 的 accepted 关联数（在 `BuildMapEvidenceFromFusedDepth` 抹掉逐点来源**之前**由 `CountDepthContributions` 计数）。只在 `acoustic_optic_demo.yaml` 开——它的 scenario 才把目标特意放进相机视场；`synthetic_smoke` 的目标经真实几何不进视场，开了就必须伪造 acceptance 才能变绿，这是明确不做的。 |
| `min_acoustic_optic_map_points` | 0（关） | 同上，声光贡献的地图点数下限。 |

控制台会打印 `acoustic-optic: ... accepted, ... map evidence points added (N optical-only, M acoustic-optic)`，用来区分"完全没有地图内容"和"有地图内容但没有真正的声光融合"。

### 5.4 八份 experiment 逐份速览

| 文件 | 一句话用途 | 关键差异 |
|---|---|---|
| `synthetic_smoke.yaml` | 冒烟基线：从 bag 读预生成相对位姿 | `estimator_mode: black_box_vio`；gates 卡 ATE ≤ 0.2m |
| `synthetic_smoke_vo.yaml` | 同场景，双目实时算相对位姿 | `estimator_mode: stereo_landmark_vo` |
| `acoustic_optic_demo.yaml` | 声光关联演示 | scenario 换 `acoustic_optic_demo.yaml`；全仓库唯一开 `min_acoustic_optic_*` gate 的 |
| `synthetic_loop_closure_vo.yaml` | 回环闭合对比的**基线**（关） | 整圈 scenario + platform.yaml |
| `synthetic_loop_closure_vo_enabled.yaml` | 回环闭合对比的**实验**（开） | 只换 defaults 为 `platform_loop_closure.yaml`；`require_converged: false`（原因见头部注释：stall 但解有限、非发散） |
| `real_holoocean_vo.yaml` | 第一个真实（非合成）数据端到端验证 | rig 换 `example_auv_real_camera.yaml`；`landmark_detector: harris_corner`；无 gates 块（只继承 require_converged 默认） |
| `synthetic_imu_preintegration.yaml` | IMU 预积分闭环（PREP-B-01） | `estimator_mode: imu_preintegration`；rig 无相机 |
| `synthetic_imu_preintegration_camera.yaml` | 对照 bag：验证"生成相机与否不改变 /raw/imu 一个字节" | 与上一份只差 rig（带相机）；不用来跑 replay_demo |

现有文件两两之间的组织原则是**"只差一处"**：对比实验通过"复制最接近的文件、只改一个 key"来做，diff 一眼能看出变量。新加 experiment 时沿用这个原则。

---

## 6. 新增一个 experiment 的 checklist

1. **复制最接近的现有文件**，只改必要的 key（最小差异原则，见 §5.4）。
2. `rig:` / `scenario:` / `defaults:` 三个路径**相对 `configs/`** 写，不是相对 experiment 文件所在目录。
3. 算法选择字段（§5.2）只能写表里的合法值——写错 `replay_demo` 启动即报错；`frontends.sonar_cfar` 等块还有 unknown-key 检查。
4. gates 按实测开：先跑、看实际 ATE/关联数，再定阈值并留余量；预期会失败的场景不要为"变绿"而放宽判定或伪造 acceptance（场景矩阵的预期拒绝不是回归，见 CLAUDE.md）。
5. 改 scenario 的目标数量/位置会改变 RNG 流的消耗序列，**ATE 跟着变是预期行为**，不要误读成前端/因子的问题（CLAUDE.md「RNG 拆流」条目）。
6. 跑完记得**实跑 demo** 验证，不要只信单元测试（CLAUDE.md 的多条历史坑都是实跑才发现的）。

## 7. 深入索引

- 分层结构与选择器消费现状的"为什么"：`configs/README.md`；根 README「分层配置」一节的分层配置图
- 解析代码：`include/runtime/config.hpp`、`src/runtime/config.cpp`
- rig schema：`schemas/proto/uw/domain/calibration.proto`
- gate 实现：`application::EvaluateReplayGates` / `application::CountDepthContributions`
- 坐标系与符号约定（depth 正负、相机系/机体系共轭方向）：`docs/frames-and-sign-conventions.svg`
- RunManifest：`include/runtime/run_manifest.hpp`
