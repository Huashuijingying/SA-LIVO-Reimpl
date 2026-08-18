# SA-LIVO (ROS2) — Strict Reproduction

An open reproduction of **SA-LIVO: Efficient LiDAR-Inertial-Visual Odometry
with Subspace-Aware Degeneracy Handling** (arXiv:2606.25699, IEEE T-RO 2026),
built on the FAST-LIVO2 ROS2 code base. All paper modules in Sect. IV–VII
are implemented, and the system is evaluated on the paper's 29 benchmark
sequences (HILTI'22 ×15, New College ×7, Oxford Spires ×7) with the official
evaluation protocols.

## Results

All 29 sequences run to completion. Per-sequence absolute translational
RMSE vs. the paper's Table II "Ours" column:

| Coverage | 29/29 |
|---|---:|
| At or near paper (≤ ~1.5×) | 24/29 |
| At or better than the paper's FAST-LIVO2 baseline | 20/29 |
| HILTI'22 | 13/15 |
| New College | 6/7 |
| Oxford Spires | 5/7 |

See [`sa_livo/RESULTS.md`](sa_livo/RESULTS.md) for the full table and the
evaluation protocol. The five remaining sequences are documented with their
root causes (photometric-residual model, SA multi-scale aggregation, and
information-scale assembly) — all accessible parameters and mechanisms have
been explored and recorded.

> **Non-official notice**: this is an independent re-implementation based on
> the paper and the FAST-LIVO2 code base — not the authors' official code.

## Citation

If you use this reproduction, please cite the SA-LIVO paper:

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
- `adaptive_voxel_map_v2` — fixed-resolution flat hash grid with per-cell
  sufficient statistics, range-adaptive voxel sizing (Eq. 7), first-planar-
  scale acceptance, plane-covariance bookkeeping (Eq. 9–14).
- `LIVMapper` — unified single-loop joint InEKF update (Algorithm 2) at the
  camera rate, with the FAST-LIVO2 two-event LIO+VIO cadence.

## Build

```bash
# ROS2 Humble; OpenCV 4 is required (OpenCV 5 crashes the node)
cd fast_livo_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sa_livo --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
```

Workspace dependencies: `livox_ros_driver2`, `rpg_vikit` (see
`fast_livo_ws/src`).

## Run & evaluate

Datasets are replayed deterministically in-process (ROS2 db3 bags, 10 Hz
image subsampling). Run one sequence at a time (concurrent runs corrupt the
deterministic trajectories):

```bash
# HILTI'22
bash datasets/hilti2022/scripts/run_hilti_direct.sh <bag.db3> <seq> <suffix>
python3 datasets/hilti2022/scripts/eval_hilti_ape.py Log/result/<seq>_<suffix>.txt <gt>

# New College / Oxford Spires: datasets/ncd, datasets/oxford_spires equivalents
```

Evaluation protocol:
- HILTI'22: official sparse-survey-point evaluator (2 s time sync, rigid
  SE(3) Umeyama alignment, translational APE).
- New College / Oxford Spires: rigid SE(3) Umeyama alignment, translational
  APE.

Per-sequence best configurations and parameters are in
[`sa_livo/config`](sa_livo/config) and the acceptance targets in
[`sa_livo/STRICT_REPRO_TARGETS.md`](sa_livo/STRICT_REPRO_TARGETS.md).

## License

This repository is a non-official reproduction derived from the FAST-LIVO2
ROS2 code base and is therefore released under the
[GPLv2](LICENSE) license of the original project. Respect the rights of the
SA-LIVO paper authors and the licenses of the benchmark datasets (HILTI'22,
New College, Oxford Spires; New College / Oxford Spires data are
CC BY-NC-SA 4.0). See [NOTICE](NOTICE.md) for provenance details.

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
    └── STRICT_REPRO_TARGETS.md  acceptance targets & protocol
```

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) and the
[Code of Conduct](CODE_OF_CONDUCT.md) before opening issues or pull requests.
Security reports: [SECURITY.md](SECURITY.md).
