# NOTICE

## Provenance

This repository is an **independent, non-official reproduction** of:

- **SA-LIVO: Efficient LiDAR-Inertial-Visual Odometry with Subspace-Aware
  Degeneracy Handling** — Y. Cao, X. He, Y. Chen, S. Liu, C. Li, J. Wang,
  IEEE Transactions on Robotics, 2026, arXiv:2606.25699.

It is built on the
[integralrobotics FAST-LIVO2 ROS2 port](https://github.com/integralrobotics/FAST-LIVO2)
(audited against commit `d4ad05174e258d5604f26cbd346ae97ba1b8c4b8`), which
adapts the [official FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)
code base (C. Zheng, W. Xu, Z. Zou, T. Hua, C. Liu, F. Zhang,
arXiv:2407.12500). FAST-LIVO2's own code lineage derives from LOAM
(J. Zhang, S. Singh, RSS 2014) and FAST-LIO. No code from the SA-LIVO authors
is included.

This repository includes modified FAST-LIVO2 source files as well as new
SA-LIVO reproduction modules, including `saif`, `sa_vio`, and
`adaptive_voxel_map_v2`. FAST-LIVO2 copyright and license notices are retained
in the inherited files. Modified inherited files now also carry dated notices
for the ROS 2 adaptation and this reproduction's changes, as applicable.

## License

- The source code is derived from FAST-LIVO2, whose repository supplies the
  GNU GPL version 2 license text and does not grant an "or later" option. This
  repository is therefore distributed as **GPL-2.0-only** (see `LICENSE`).
- The immediate ROS 2 port has the same GPLv2 root `LICENSE` and identifies
  GPLv2 in its README, although its `package.xml` says BSD. Because a package
  manifest cannot relicense inherited GPL code, this repository treats the
  GPLv2 grant as controlling and corrects its own manifest accordingly.
- The `package.xml` license tag is `GPL-2.0-only`.
- This license choice applies to the repository's code. It does not change or
  supersede the licenses of external dependencies, papers, or datasets.

## Direct dependency inventory

The following direct build or link dependencies were identified from
`sa_livo/CMakeLists.txt`, `sa_livo/package.xml`, and source includes. They are
not vendored in this repository. Exact versions and file-level notices remain
controlling.

| Component | Role | Declared license |
|---|---|---|
| [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) and its [integralrobotics ROS 2 port](https://github.com/integralrobotics/FAST-LIVO2) | Included and modified base code | GPL version 2; treated here as GPL-2.0-only |
| WilsonGuo ROS2 [`vikit_common`](https://github.com/WilsonGuo/Fast-LIVO2-Drvier-ROS2/blob/main/rpg_vikit/vikit_common/package.xml) and [`vikit_ros`](https://github.com/WilsonGuo/Fast-LIVO2-Drvier-ROS2/blob/main/rpg_vikit/vikit_ros/package.xml) | Required linked workspace dependencies | GPLv3 in both package manifests |
| [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | Required ROS 2 driver/messages | MIT, with bundled third-party notices |
| ROS 2 core, launch, messages, and rosbag2 (`ament_cmake`, `ament_index_python`, `launch`, `launch_ros`, `rclcpp`, `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, `rosbag2_cpp`, `rosbag2_storage`, `demo_nodes_cpp`) | Required build/runtime dependencies | Primarily Apache-2.0; package manifests control |
| ROS 2 TF, PCL, image, vision, and visualization packages (`tf2`, `tf2_ros`, `tf2_geometry_msgs`, `pcl_ros`, `pcl_conversions`, `cv_bridge`, `image_transport`, `rcpputils`, `rviz2`) | Required or optional runtime dependencies | BSD variants and/or Apache-2.0; package manifests control |
| [OpenCV](https://github.com/opencv/opencv) | Required vision library | Apache-2.0 for 4.5.0 and later; BSD-3-Clause for earlier releases |
| [Eigen](https://gitlab.com/libeigen/eigen) | Required linear algebra library | Primarily MPL-2.0; file-level exceptions apply |
| [Point Cloud Library](https://github.com/PointCloudLibrary/pcl) | Required point-cloud library | BSD-3-Clause |
| [Sophus](https://github.com/strasdat/Sophus) | Required Lie group library | MIT |
| [Boost](https://www.boost.org/) | Required thread and utility library | BSL-1.0 |
| OpenMP runtime | Optional toolchain runtime | Implementation-specific; GCC `libgomp` uses GPLv3+ with the GCC Runtime Library Exception |
| [mimalloc](https://github.com/microsoft/mimalloc) | Optional allocator | MIT |

Transitive system and middleware dependencies are not exhaustively reproduced
in this table. Distributors must inspect the manifests and license files of the
exact dependency versions they ship.

## Compatibility and distribution status

This source audit found two unresolved issues for redistribution of a combined
binary:

1. The ROS2 `rpg_vikit` source used by this project declares **GPLv3**, while
   the inherited FAST-LIVO2 code is **GPL-2.0-only**. GPLv2-only and GPLv3 code
   cannot currently be combined and redistributed under one of those licenses.
2. Several required ROS 2 components and OpenCV 4.5+ use **Apache-2.0**. The
   Apache Software Foundation identifies Apache-2.0 as compatible with GPLv3,
   but not GPLv2. The effect on a particular dynamically linked or system-
   packaged distribution requires case-specific review and is not represented
   here as cleared.

Accordingly, this repository currently distributes source only and does not
include prebuilt SA-LIVO executables. Until the conflicts are resolved through
compatible replacements, additional permissions, or qualified legal review,
do not redistribute compiled executables, binary packages, or container images
that combine this code with the affected dependencies.

Changing this repository to GPLv3 alone would not solve the first issue,
because the inherited FAST-LIVO2 code has not been granted an "or later"
license in its repository. This notice is a technical license inventory, not
legal advice.

## Benchmark datasets

- HILTI'22: research use; see the official HILTI 2022 dataset terms.
- New College and Oxford Spires: CC BY-NC-SA 4.0 (the authors retain all
  rights; this repository does not redistribute dataset files).

## Contact

Maintainer: Huashuijingying <1098763683@qq.com>.
