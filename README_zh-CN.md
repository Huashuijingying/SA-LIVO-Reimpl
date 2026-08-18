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

## 构建

```bash
# ROS2 Humble；必须 OpenCV 4（OpenCV 5 会使节点崩溃）
cd fast_livo_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sa_livo --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
```

依赖：`livox_ros_driver2`、`rpg_vikit`（见工作区 `fast_livo_ws/src`）。

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

本仓库派生自 FAST-LIVO2 ROS2 代码库，按原项目的
[GPLv2](LICENSE) 许可发布。请尊重 SA-LIVO 论文作者权益以及基准数据集
许可（New College / Oxford Spires 为 CC BY-NC-SA 4.0）。

## 目录结构

见 [README.md](README.md) 的 Project structure 一节。

## 贡献

提交 Issue/PR 前请阅读 [CONTRIBUTING.md](CONTRIBUTING.md) 与
[CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md)；安全漏洞报告见
[SECURITY.md](SECURITY.md)。
