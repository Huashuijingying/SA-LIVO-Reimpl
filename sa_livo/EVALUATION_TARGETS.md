# SA-LIVO Reproduction: Evaluation Targets and Paper Alignment

This document defines the quantitative comparison targets and evaluation
protocol for this independent SA-LIVO reproduction and engineering extension.
The values are taken from arXiv:2606.25699v2 Table II; the paper's "Ours"
values are unchanged from v1 for these 29 sequences.

- The main Sect. IV–VII architecture is implemented on the FAST-LIVO2 ROS2
  base, without claiming source-level equivalence to the authors' code.
- All 29 "Ours" benchmark sequences are covered: HILTI'22 (15), Newer College
  Dataset (7), Oxford Spires (7).
- Baselines are restricted to FAST-LIVO2 (other comparison methods from the
  paper are not run here).

## 0. 论文版本对齐范围

本仓库当前不是严格的 v1 或 v2 实现：

- 状态、IMU 传播、SAIF 与联合 InEKF 对齐 v1/v2 共有的核心设计。
- LiDAR 地图是混合实现：保留 v1 风格的 `kappa` 与绝对特征值双重判定、
  完整平面协方差等路径，同时包含 v2 风格的固定哈希体素充分统计量、查询时
  多尺度支持域与首个合格平面策略。
- 视觉前端实现了 v1/v2 共有的信息形式；v2 的 Huber 权重、图像网格选点等
  以可选路径存在，但并非所有评测配置均启用。
- NCD Stairs 的 frontier region-grow、自适应 SAIF 阈值和退化保护属于本仓库
  工程扩展，两个论文版本均未描述。
- 下表包含逐序列最优配置结果，不等价于论文要求的单套全局参数实验。

因此，本文档中的“目标”是结果比较基准，不是严格复现验收声明。

当前默认评测配置与 v2 的已知直接差异还包括：`min_eigen_abs=0.01`
仍参与平面判定，`eps_pose_inflate=0.01` 而非 v2 的 `0.02`，且
`photometric_huber_en` 与 `point_select_image_grid` 默认关闭。这些开关和
逐序列配置是本仓库实验设计的一部分，不能视为 v2 默认实现。

## 1. 目标：论文 Table II “Ours” 绝对平移 RMSE（m）

每条序列的目标值为论文 Table II “Ours” 列：

### HILTI’22（官方 sparse-survey-point 评估器）

| 序列 | 论文 Ours (m) |
|---|---:|
| exp01 construction ground level | 0.010 |
| exp02 construction multilevel | 0.020 |
| exp03 construction stairs | 0.029 |
| exp04 construction upper level | 0.024 |
| exp05 construction upper level 2 | 0.007 |
| exp06 construction upper level 3 | 0.011 |
| exp07 long corridor | 0.044 |
| exp09 cupola | 0.130 |
| exp10 cupola 2 | 0.075 |
| exp11 lower gallery | 0.015 |
| exp14 basement 2 | 0.024 |
| exp15 attic to upper gallery | 0.122 |
| exp16 attic to upper gallery 2 | 0.069 |
| exp18 corridor lower gallery 2 | 0.022 |
| exp21 outside building | 0.022 |

### Newer College Dataset (NCD)

| 序列 | 论文 Ours (m) |
|---|---:|
| Quad Easy | 0.026 |
| Quad Medium | 0.068 |
| Quad Hard | 0.055 |
| Stairs | 0.036 |
| Underground Easy | 0.047 |
| Underground Medium | 0.040 |
| Underground Hard | 0.051 |

### Oxford Spires

| 序列 | 论文 Ours (m) |
|---|---:|
| blenheim-palace-01 | 0.112 |
| blenheim-palace-05 | 0.152 |
| christ-church-03 | 0.015 |
| keble-college-02 | 0.024 |
| keble-college-05 | 0.110 |
| observatory-quarter-01 | 0.055 |
| observatory-quarter-02 | 0.051 |

## 2. 评估协议

- HILTI’22：使用官方 sparse-survey-point 评估器
  （`datasets/hilti2022/evaluation/evaluation.py`），即：读取 TUM 轨迹、
  施加 poletip 标定、与 sparse survey-point GT 时间同步（max_diff=2 s）、
  Umeyama SE(3) 对齐（不缩放），报告 translation APE RMSE。
- Oxford Spires：GT 用 `processed/trajectory/gt-tum.txt`，SE(3) 刚性 Umeyama
  对齐，平移 APE RMSE（`datasets/oxford_spires/scripts/eval_spires_ape.py`）。
- NCD：GT 用官方轨迹，SE(3) Umeyama 对齐，平移 APE RMSE。

## 3. 论文固定的全局参数

论文明确规定、不得按序列调参的项：

- 点云下采样 leaf：HILTI 0.2 m；其余 0.5 m。
- 地图 voxel：HILTI 0.5 m；其余 1.0 m。
- 图像流下采样到 10 Hz（与 LiDAR 对齐）。
- W（滑窗）= 5。
- sigma_min = 1。
- sigma_px = 1（视觉噪声底）。
- kappa_th = 0.05；epsilon0 = 1e-3。
- tau_L ≈ 25（v2 标为数据集调定的近似值）。
- rho_d = 4 px；delta_d = 0.5 m；theta_max = 80 deg。
- patch spacing sp = 8 -> rho = 3 px。
- delta_rms = 12；sigma_rms = 8（q_rms 阈值）。
- N_PCA = 25（累计支持点数达到该值后才开始执行多尺度 PCA）。
- K_max = 5（联合 InEKF 迭代上限）。
- epsilon_P = 0.02（v2 明确给出的收敛后位姿协方差膨胀系数）。

v2 的平面接受仅使用 `kappa < kappa_th`，不再联合固定 `epsilon_min`。
其余未给出数值的参数（tau_V、Ncap、Nobs、
Nq、sigma_r/sigma_theta、parallax/delta_abs/delta_alpha、eta_min/eta_max、
delta_R/delta_t）必须在“所有序列共用一套固定值”的前提下，通过复现实验
回溯到能逼近 Table II 的取值；不允许 per-sequence 调参。

## 4. 报告口径 / Reporting

- Per-sequence APE RMSE is reported against the Table II "Ours" column.
- The paper-fidelity criterion is a single set of global parameters across
  all 29 sequences; this repository additionally reports per-sequence best
  configurations (see `RESULTS.md`).
- Every reported number is produced by a deterministic replay and verified
  with the official evaluators; regression checks run on multiple sequences
  after each change.
