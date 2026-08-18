# SA-LIVO (ROS2) — 非官方复现

基于 FAST-LIVO2 ROS2 代码库，对 **SA-LIVO: Efficient LiDAR-Inertial-Visual
Odometry with Subspace-Aware Degeneracy Handling**（arXiv:2606.25699，
IEEE T-RO 2026）论文理论模块（Sect. IV–VII）的独立复现，并在论文的 29 条
基准序列（HILTI'22 ×15、New College ×7、Oxford Spires ×7）上按官方评估
协议完成评测。

> **非官方声明**：本仓库为基于论文与 FAST-LIVO2 代码库的独立复现，
> 并非作者官方实现。

## 结果

29/29 条序列全部跑通；24/29 达到/接近论文 Table II 水平（平移 RMSE
≤ ~1.5×；HILTI 13/15、New College 6/7、Oxford Spires 5/7）。完整逐序列
结果见 [`sa_livo/RESULTS.md`](sa_livo/RESULTS.md)。剩余 5 条的根因
（光度残差模型、SA 多尺度聚合、信息尺度装配）均有文档记录。

## 引用

使用本复现请引用 SA-LIVO 论文：

```bibtex
@article{cao2026salivo,
  title={SA-LIVO: Efficient LiDAR-Inertial-Visual Odometry with
         Subspace-Aware Degeneracy Handling},
  author={Cao, Yinong and He, Xin and Chen, Yuwei and Liu, Shijie and
          Li, Chunlai and Wang, Jianyu},
  journal={IEEE Transactions on Robotics},
  year={2026},
  note={arXiv:2606.25699}
}
```

本复现所基于的代码库（FAST-LIVO2）：

```bibtex
@article{zheng2024fastlivo2,
  title={FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry},
  author={Zheng, Chunran and Xu, Wei and Zou, Zuhao and Hua, Tong and
          Liu, Changwu and Zhang, Fu},
  journal={IEEE Transactions on Robotics},
  year={2025},
  note={arXiv:2407.12500}
}
```

## 构建

```bash
# ROS2 Humble；必须 OpenCV 4（OpenCV 5 会使节点崩溃）
# 使用任意 colcon 工作区；将本仓库放在工作区的 src/ 目录下。
mkdir -p ~/sa_livo_ws/src
cd ~/sa_livo_ws/src
git clone https://github.com/Huashuijingying/SA-LIVO-Reimpl.git
# 同时把 livox_ros_driver2 和 rpg_vikit 放到这个 src/，或先 source
# 提供它们的 underlay。
cd ~/sa_livo_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sa_livo --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
```

依赖：[`livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2)、
[`rpg_vikit`（ROS 2 适配版）](https://github.com/WilsonGuo/Fast-LIVO2-Drvier-ROS2/tree/main/rpg_vikit)；
可放在同一工作区的 `src/` 下，也可由已 source 的 underlay 提供。

## 运行与评估

系统在进程内确定性回放 ROS2 db3 bag（图像 10 Hz 子采样）。**必须串行
运行**（并发会破坏确定性轨迹）。

```bash
# HILTI'22
bash datasets/hilti2022/scripts/run_hilti_direct.sh <bag.db3> <seq> <suffix>
python3 datasets/hilti2022/scripts/eval_hilti_ape.py Log/result/<seq>_<suffix>.txt <gt>

# New College / Oxford Spires：datasets/ncd、datasets/oxford_spires 对应脚本
```

评估协议：HILTI 使用官方 sparse-survey-point 评估器（2 s 时间同步、刚性
SE(3) Umeyama 对齐、平移 APE）；NCD/Spires 为刚性 SE(3) Umeyama 平移 APE。

## 许可

本仓库包含并修改了 [integralrobotics 的 FAST-LIVO2 ROS2
适配版](https://github.com/integralrobotics/FAST-LIVO2)及其上游
[FAST-LIVO2 官方代码](https://github.com/hku-mars/FAST-LIVO2)。上游仓库提供
GNU GPL version 2 许可文本，但未授予“or later”选项，因此本仓库采用
[GPL-2.0-only](LICENSE)，以遵守 FAST-LIVO2 的上游许可；这一选择不会改变
外部依赖各自的许可证。

当前审计识别出的直接依赖许可证如下：

| 依赖组 | 声明的许可证 |
|---|---|
| FAST-LIVO2 基础代码 | GPL-2.0-only |
| WilsonGuo ROS2 `rpg_vikit`（`vikit_common`、`vikit_ros`） | GPLv3 |
| `livox_ros_driver2`、Sophus、可选 mimalloc | MIT；驱动内第三方声明仍适用 |
| ROS 2 核心、launch、消息与 rosbag2 | 主要为 Apache-2.0 |
| ROS 2 TF、PCL、图像、视觉桥接与 RViz 组件 | BSD 变体和/或 Apache-2.0 |
| OpenCV | 4.5.0 及以后为 Apache-2.0；更早版本为 BSD-3-Clause |
| Eigen、PCL、Boost | 主要为 MPL-2.0、BSD-3-Clause、BSL-1.0 |
| OpenMP 运行时 | 取决于工具链；GCC `libgomp` 带 GCC Runtime Library Exception |

**二进制分发警告：**当前 ROS2 `rpg_vikit` 声明为 GPLv3，与
GPL-2.0-only 代码组合后再分发时存在不兼容。采用 Apache-2.0 的 ROS 2
组件和 OpenCV 4.5+ 在二进制分发前也需要结合具体打包与链接方式审查。
本仓库目前仅发布源码；在通过兼容替代、额外授权或合格法律审查解决前，
请勿分发组合上述依赖后编译出的可执行文件、二进制包或容器镜像。

请尊重 SA-LIVO 论文作者权益以及基准数据集许可（New College / Oxford
Spires 为 CC BY-NC-SA 4.0）。完整来源、依赖链接、兼容性状态与审计范围见
[NOTICE](NOTICE.md)。

## 目录结构

见 [README.md](README.md) 的 Project structure 一节。

## 贡献

提交 Issue/PR 前请阅读 [CONTRIBUTING.md](CONTRIBUTING.md) 与
[CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md)；安全漏洞报告见
[SECURITY.md](SECURITY.md)。
