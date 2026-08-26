# SA-LIVO package (ROS2)

This package implements **SA-LIVO: Efficient LiDAR-Inertial-Visual Odometry
with Subspace-Aware Degeneracy Handling** (public preprint:
arXiv:2606.25699v2, 6 August 2026) on the FAST-LIVO2 ROS2 code base. It is an
independent, paper-guided implementation and engineering extension of the
Sect. IV–VII architecture, not a strict implementation match.

Publication status: this package cites the public arXiv preprint and does not
claim acceptance or publication in IEEE Transactions on Robotics.

## Implemented modules

- `saif` — Subspace-Aware Information Fusion (Algorithm 1): eigendecomposition
  of the joint information matrix ΛL + q·ΛV, linear-clamp soft gate
  g(k) = min(√λk/σmin, 1), guaranteed-PSD reconstruction, unified joint
  InEKF solve (Sect. VII).
- `sa_vio` — information-form direct photometric VIO (Sect. VI): sliding
  window W=5, sparse 9-pixel cross patches, per-frame affine brightness
  model, pre-loop frozen Jacobians, per-observation decorrelation
  wdec = 1/nused, multi-gate filtering, scene-level quality factor q.
- `voxel_map` — hybrid map layer with flat-grid sufficient statistics,
  optional legacy/adaptive paths, multi-scale support growth, and selectable
  plane-covariance models.
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

## Real-time ROS2 replay

The default AVIA launch uses live ROS2 subscriptions. Start the mapper and
the bag player in separate terminals:

```bash
# Terminal 1
source /opt/ros/humble/setup.bash
source ~/sa_livo_ws/install/setup.bash
ros2 launch sa_livo mapping_avia.launch.py use_rviz:=true

# Terminal 2
source /opt/ros/humble/setup.bash
source ~/sa_livo_ws/install/setup.bash
ros2 bag play /absolute/path/to/YOUR_DOWNLOADED_BAG --rate 1.0
```

The AVIA defaults subscribe to `/livox/lidar`, `/livox/imu`, and
`/left_camera/image`. Use `ros2 bag info` to verify the bag topics and message
types. For different topic names, pass a parameter overlay with
`avia_params_file:=...`. Live subscriptions use Best Effort QoS by default;
leave `common.lossless_replay_qos` disabled for this workflow. Leave
`common.direct_bag_path` empty for live use; setting it opts into the
deterministic in-process evaluator path.

## Run & evaluate

The system replays ROS2 db3 bags deterministically in-process (10 Hz image
subsampling). Run one sequence at a time; concurrent runs corrupt the
deterministic trajectories.

```bash
# HILTI'22
bash datasets/hilti2022/scripts/run_hilti_direct.sh <bag.db3> <seq> <suffix>
python3 datasets/hilti2022/scripts/eval_hilti_ape.py Log/result/<seq>_<suffix>.txt <gt>

# Newer College Dataset
bash datasets/ncd/run_ncd_direct.sh <bag.db3> <seq> <suffix>
python3 datasets/ncd/eval_ncd_ape.py Log/result/<seq>_<suffix>.txt <gt>

# Oxford Spires
bash datasets/oxford_spires/scripts/run_spires_direct.sh <bag.db3> <seq> <suffix>
python3 datasets/oxford_spires/scripts/eval_spires_ape.py Log/result/<seq>_<suffix>.txt <gt>
```

Evaluation protocol:

- HILTI'22: official sparse-survey-point evaluator (2 s time sync, rigid
  SE(3) Umeyama alignment, translational APE).
- Newer College Dataset / Oxford Spires: rigid SE(3) Umeyama alignment, translational
  APE.

Results on the 29 benchmark sequences: `RESULTS.md`. Acceptance targets and
the paper-alignment notes: `EVALUATION_TARGETS.md`.

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

- The implementation is not strictly aligned with either preprint version.
  The LiDAR path retains v1-style dual planarity checks and full covariance
  code while also implementing v2-style flat-grid sufficient statistics and
  query-time multi-scale support. Optional visual paths cover some v2 changes.
- The NCD Stairs configuration adds frontier region growing, adaptive SAIF
  thresholding, and a degeneracy guard that are repository extensions rather
  than paper methods.
- Benchmark targets come from v2 Table II. Per-sequence best configurations
  are reported separately from the paper's fixed-global-parameter setting.
- The paper's 18-dim state maps onto the code base's 19-dim state (the
  exposure entry is inert in SA-LIVO mode).
- LiDAR Jacobian follows the FAST-LIVO2 convention h = +Jᵀ (verified
  numerically); the photometric Jacobian is derived in the same body-frame
  error basis and verified against finite differences.
- Image subscription uses sensor-data QoS (BEST_EFFORT); camera drivers and
  image_transport republish publish BEST_EFFORT, so the node subscribes with
  matching QoS.

## License

This package includes and modifies the
[integralrobotics FAST-LIVO2 ROS2 port](https://github.com/integralrobotics/FAST-LIVO2)
of the official FAST-LIVO2 code and is released as GPL-2.0-only to preserve
that upstream license. External dependencies retain their own licenses.

The currently used ROS2 `rpg_vikit` declares GPLv3, creating an unresolved
compatibility issue for redistribution of a combined GPL-2.0-only binary.
Apache-2.0 ROS 2/OpenCV components also require distribution-specific review.
This repository distributes source only. See [`../NOTICE.md`](../NOTICE.md)
for the direct dependency inventory and binary distribution warning.

Respect the rights of the SA-LIVO paper authors and the licenses of the
benchmark datasets. This is a research reproduction, not the authors'
official implementation.
