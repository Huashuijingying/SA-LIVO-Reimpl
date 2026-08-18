# NOTICE

## Provenance

This repository is an **independent, non-official reproduction** of:

- **SA-LIVO: Efficient LiDAR-Inertial-Visual Odometry with Subspace-Aware
  Degeneracy Handling** — Y. Cao, X. He, Y. Chen, S. Liu, C. Li, J. Wang,
  IEEE Transactions on Robotics, 2026, arXiv:2606.25699.

It is built on the **FAST-LIVO2** ROS2 code base
(C. Zheng, W. Xu, Z. Zou, T. Hua, C. Liu, F. Zhang, arXiv:2407.12500),
whose own code lineage derives from LOAM (J. Zhang, S. Singh, RSS 2014) and
FAST-LIO. No code from the SA-LIVO authors is included.

## License

- The source code is derived from FAST-LIVO2 and is distributed under the
  **GNU General Public License v2** (see `LICENSE`), consistent with the
  upstream project.
- The `package.xml` license tag is `GPL-2.0-only`.

## Third-party notices

- Build-time dependencies are not vendored: `livox_ros_driver2`
  (BSD), `rpg_vikit` (BSD-3), Eigen (MPL-2.0), OpenCV (Apache-2.0),
  PCL (BSD), ROS 2 (Apache-2.0). Their respective licenses apply.

## Benchmark datasets

- HILTI'22: research use; see the official HILTI 2022 dataset terms.
- New College and Oxford Spires: CC BY-NC-SA 4.0 (the authors retain all
  rights; this repository does not redistribute dataset files).

## Contact

Maintainer: Huashuijingying <1098763683@qq.com>.
