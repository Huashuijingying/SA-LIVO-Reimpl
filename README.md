# SA-LIVO (ROS2) — Independent Reproduction and Engineering Extension

An independent, paper-guided reproduction and engineering extension of
**SA-LIVO: Efficient LiDAR-Inertial-Visual Odometry with Subspace-Aware
Degeneracy Handling** (public preprint: arXiv:2606.25699v2, 6 August 2026),
built on the FAST-LIVO2 ROS2 code base. The repository implements the paper's
main Sect. IV–VII architecture and evaluates it on the 29 benchmark sequences
(HILTI'22 ×15, Newer College Dataset ×7, Oxford Spires ×7) using the stated
evaluation protocols.

Publication status: this repository cites the public arXiv preprint and does
not claim acceptance or publication in IEEE Transactions on Robotics.

## Results

All 29 sequences run to completion. Per-sequence absolute translational
RMSE vs. the paper's Table II "Ours" column:

| Coverage | 29/29 |
|---|---:|
| At or near paper (≤ ~1.5×) | 24/29 |
| At or better than the paper's FAST-LIVO2 baseline | 20/29 |
| HILTI'22 | 13/15 |
| Newer College Dataset | 6/7 |
| Oxford Spires | 5/7 |

See [`sa_livo/RESULTS.md`](sa_livo/RESULTS.md) for the full table and the
evaluation protocol. The five remaining sequences are documented with their
root causes (photometric-residual model, SA multi-scale aggregation, and
information-scale assembly) — all accessible parameters and mechanisms have
been explored and recorded.

> **Non-official notice**: this is an independent re-implementation based on
> the paper and the FAST-LIVO2 code base — not the authors' official code and
> not a claim of source-level equivalence to either paper version.

## Paper-version alignment

The public v1 and v2 preprints differ materially in the LiDAR map and visual
front end. This repository currently contains a hybrid of paper-guided and
repository-specific behavior:

| Area | Current alignment |
|---|---|
| State, IMU propagation, SAIF, joint InEKF | Paper-guided implementation of the core shared by v1 and v2 |
| LiDAR map and plane association | Mixed: v1-style dual `kappa`/absolute-eigenvalue checks and full plane covariance remain alongside v2-style flat-grid sufficient statistics, query-time multi-scale support, and first-planar-scale acceptance |
| Visual front end | Shared v1/v2 sliding-window information form; some v2 additions, including Huber weighting and image-grid selection, exist as optional paths but are not enabled in every benchmark configuration |
| NCD Stairs | Repository extension: frontier region growing, adaptive SAIF thresholding, and a degeneracy guard are not described in v1 or v2 |
| Benchmark results | Independent runs compared with v2 Table II; several rows use per-sequence best configurations rather than the paper's single global configuration |

Accordingly, results in this repository are reproduction measurements and
engineering experiments, not a strict v1 or strict v2 reproduction. See
[`sa_livo/EVALUATION_TARGETS.md`](sa_livo/EVALUATION_TARGETS.md) for the
target values, protocol, and known divergences.

## Citation

If you use this reproduction, please cite the SA-LIVO paper:

```bibtex
@misc{cao2026salivo,
  title={SA-LIVO: Efficient LiDAR-Inertial-Visual Odometry with
         Subspace-Aware Degeneracy Handling},
  author={Cao, Yinong and He, Xin and Zhu, Shouzheng and Wang, Senyuan and
          Jiang, Changhui and Zhang, Chenyang and He, Pingfeng and Wang, Tingwei and
          Chen, Yuwei and Liu, Shijie and Li, Chunlai and Huang, Genghua and Wang, Jianyu},
  year={2026},
  eprint={2606.25699},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  note={Version 2, 6 August 2026}
}
```

The code base this reproduction is derived from:

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

## Implemented modules

- `saif` — Subspace-Aware Information Fusion (Algorithm 1): joint information
  eigendecomposition, linear-clamp soft gate, guaranteed-PSD reconstruction,
  unified joint InEKF solve (Sect. VII).
- `sa_vio` — information-form direct photometric VIO (Sect. VI): sliding
  window, sparse cross patches, per-frame affine brightness model, frozen
  pre-loop Jacobians, per-observation decorrelation, multi-gate filtering,
  scene-level quality factor.
- `voxel_map` — hybrid experimental map layer with flat-grid sufficient
  statistics, optional legacy/adaptive paths, query-time multi-scale plane
  support, and selectable plane-covariance models. Its switches span v1-like,
  v2-like, and repository-specific behavior; no single setting is claimed to
  be the authors' implementation.
- `LIVMapper` — unified single-loop joint InEKF update (Algorithm 2) at the
  camera rate, with the FAST-LIVO2 two-event LIO+VIO cadence.

## Build

```bash
# ROS2 Humble; OpenCV 4 is required (OpenCV 5 crashes the node)
# Use any colcon workspace; place this repository under its src/ directory.
mkdir -p ~/sa_livo_ws/src
cd ~/sa_livo_ws/src
git clone https://github.com/Huashuijingying/SA-LIVO-Reimpl.git
# Also place livox_ros_driver2 and rpg_vikit in this src/, or source an underlay
# that provides them.
cd ~/sa_livo_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sa_livo --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
```

Workspace dependencies: [`livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2),
[`rpg_vikit` (ROS 2 port)](https://github.com/WilsonGuo/Fast-LIVO2-Drvier-ROS2/tree/main/rpg_vikit);
make them available from the same workspace `src/` directory or from a sourced
underlay.

## Run & evaluate

Datasets are replayed deterministically in-process (ROS2 db3 bags, 10 Hz
image subsampling). Run one sequence at a time (concurrent runs corrupt the
deterministic trajectories):

```bash
# HILTI'22
bash datasets/hilti2022/scripts/run_hilti_direct.sh <bag.db3> <seq> <suffix>
python3 datasets/hilti2022/scripts/eval_hilti_ape.py Log/result/<seq>_<suffix>.txt <gt>

# Newer College Dataset / Oxford Spires: datasets/ncd, datasets/oxford_spires equivalents
```

Evaluation protocol:
- HILTI'22: official sparse-survey-point evaluator (2 s time sync, rigid
  SE(3) Umeyama alignment, translational APE).
- Newer College Dataset / Oxford Spires: rigid SE(3) Umeyama alignment, translational
  APE.

Per-sequence best configurations and parameters are in
[`sa_livo/config`](sa_livo/config) and the evaluation targets in
[`sa_livo/EVALUATION_TARGETS.md`](sa_livo/EVALUATION_TARGETS.md).

## License

This repository is a non-official reproduction derived from the
[integralrobotics FAST-LIVO2 ROS2 port](https://github.com/integralrobotics/FAST-LIVO2)
of the [official FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) code.
Because it includes and modifies FAST-LIVO2 source released under GNU GPL
version 2 without an "or later" grant, this repository uses
[GPL-2.0-only](LICENSE). This choice preserves the upstream license; it does
not relicense external dependencies.

Direct dependency licenses identified by the current audit:

| Dependency group | Declared license |
|---|---|
| FAST-LIVO2 base code | GPL-2.0-only |
| WilsonGuo ROS2 `rpg_vikit` (`vikit_common`, `vikit_ros`) | GPLv3 |
| `livox_ros_driver2`, Sophus, optional mimalloc | MIT; driver third-party notices also apply |
| ROS 2 core/launch/messages/rosbag2 | Primarily Apache-2.0 |
| ROS 2 TF/PCL/image/vision/RViz packages | BSD variants and/or Apache-2.0 |
| OpenCV | Apache-2.0 for 4.5.0+; BSD-3-Clause for earlier releases |
| Eigen, PCL, Boost | MPL-2.0 (primarily), BSD-3-Clause, BSL-1.0 |
| OpenMP runtime | Toolchain-specific; GCC runtime exception applies to `libgomp` |

**Binary distribution warning:** the current ROS2 `rpg_vikit` declares GPLv3,
which is incompatible with GPL-2.0-only for a redistributed combined work.
Apache-2.0 ROS 2 components and OpenCV 4.5+ also require case-specific review
before binary distribution. This repository contains source only; do not
redistribute compiled binaries or containers combining the affected
dependencies until compatible replacements, permissions, or qualified legal
review resolve these issues.

Respect the rights of the SA-LIVO paper authors and the benchmark dataset
licenses (Newer College Dataset / Oxford Spires data are CC BY-NC-SA 4.0). See
[NOTICE](NOTICE.md) for the provenance, dependency inventory, source links,
and audit scope.

## Project structure

```text
.
├── LICENSE / .gitignore
├── README.md / README_zh-CN.md / CONTRIBUTING.md / CODE_OF_CONDUCT.md
├── SECURITY.md / CITATION.cff
├── .github/                 issue/PR templates + CI workflow
└── sa_livo/                 ROS2 package
    ├── src/                 LIVMapper, saif, sa_vio, voxel_map, IMU ...
    ├── include/             headers (incl. utils/)
    ├── config/              per-dataset configurations
    ├── tests/               saif tests
    ├── launch/ rviz_cfg/
    ├── CMakeLists.txt / package.xml
    ├── README.md            package README (build/run/evaluate)
    ├── RESULTS.md           29-sequence benchmark results
    └── EVALUATION_TARGETS.md    targets, protocol & paper alignment
```

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) and the
[Code of Conduct](CODE_OF_CONDUCT.md) before opening issues or pull requests.
Security reports: [SECURITY.md](SECURITY.md).
