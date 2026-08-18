# SA-LIVO package (ROS2)

This package implements **SA-LIVO: Efficient LiDAR-Inertial-Visual Odometry
with Subspace-Aware Degeneracy Handling** (arXiv:2606.25699, IEEE T-RO 2026)
on the FAST-LIVO2 ROS2 code base, as a strict reproduction of the paper's
theory modules (Sect. IV–VII) and benchmark evaluation.

## Implemented modules

- `saif` — Subspace-Aware Information Fusion (Algorithm 1): eigendecomposition
  of the joint information matrix ΛL + q·ΛV, linear-clamp soft gate
  g(k) = min(√λk/σmin, 1), guaranteed-PSD reconstruction, unified joint
  InEKF solve (Sect. VII).
- `sa_vio` — information-form direct photometric VIO (Sect. VI): sliding
  window W=5, sparse 9-pixel cross patches, per-frame affine brightness
  model, pre-loop frozen Jacobians, per-observation decorrelation
  wdec = 1/nused, multi-gate filtering, scene-level quality factor q.
- `adaptive_voxel_map_v2` — flat hash grid with per-cell sufficient
  statistics, range-adaptive voxel sizing (Eq. 7), first-planar-scale
  acceptance, and plane-covariance bookkeeping (Eq. 9–14).
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

The system replays ROS2 db3 bags deterministically in-process (10 Hz image
subsampling). Run one sequence at a time; concurrent runs corrupt the
deterministic trajectories.

```bash
# HILTI'22
bash datasets/hilti2022/scripts/run_hilti_direct.sh <bag.db3> <seq> <suffix>
python3 datasets/hilti2022/scripts/eval_hilti_ape.py Log/result/<seq>_<suffix>.txt <gt>

# New College
bash datasets/ncd/run_ncd_direct.sh <bag.db3> <seq> <suffix>
python3 datasets/ncd/eval_ncd_ape.py Log/result/<seq>_<suffix>.txt <gt>

# Oxford Spires
bash datasets/oxford_spires/scripts/run_spires_direct.sh <bag.db3> <seq> <suffix>
python3 datasets/oxford_spires/scripts/eval_spires_ape.py Log/result/<seq>_<suffix>.txt <gt>
```

Evaluation protocol:

- HILTI'22: official sparse-survey-point evaluator (2 s time sync, rigid
  SE(3) Umeyama alignment, translational APE).
- New College / Oxford Spires: rigid SE(3) Umeyama alignment, translational
  APE.

Results on the 29 benchmark sequences: `RESULTS.md`. Acceptance targets and
the fixed global parameters: `STRICT_REPRO_TARGETS.md`.

## Tests

```bash
cd tests && g++ -std=c++17 -O2 -I/usr/include/eigen3 -I../include \
  saif_test.cpp ../src/saif.cpp -o saif_test && ./saif_test
```

Covers PSD of the fused matrix, linear-clamp attenuation of degenerate
directions, VIO rescue of LiDAR-degenerate directions, Newton-step
equivalence of the joint solve, and graceful degradation under concurrent
sensor failure.

## Reproduction notes

- The paper's 18-dim state maps onto the code base's 19-dim state (the
  exposure entry is inert in SA-LIVO mode).
- LiDAR Jacobian follows the FAST-LIVO2 convention h = +Jᵀ (verified
  numerically); the photometric Jacobian is derived in the same body-frame
  error basis and verified against finite differences.
- Image subscription uses sensor-data QoS (BEST_EFFORT); camera drivers and
  image_transport republish publish BEST_EFFORT, so the node subscribes with
  matching QoS.

## License

This package is derived from the FAST-LIVO2 ROS2 code base (GPLv2) and is
released under GPLv2-compatible terms. Respect the rights of the SA-LIVO
paper authors and the licenses of the benchmark datasets (HILTI'22, New
College, Oxford Spires; New College / Oxford Spires are CC BY-NC-SA 4.0).
This is a research reproduction, not the authors' official implementation.
