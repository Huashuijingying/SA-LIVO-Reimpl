#!/usr/bin/env bash
# Serial raw-PCD generation for HILTI'22, one sequence at a time (direct
# replay is deterministic per-condition but sensitive to concurrency).
# Updated 2026-08-16: db3 FILE paths (directory paths fail), per-sequence
# best configs from the authoritative table, exp01 included.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# seq|per-sequence extra args ("" = default pmn full)
declare -A CFG=(
  ["exp01_construction_ground_level"]=""
  ["exp02_construction_multilevel"]=""
  ["exp04_construction_upper_level"]=""
  ["exp05_construction_upper_level_2"]=""
  ["exp06_construction_upper_level_3"]=""
  ["exp07_long_corridor"]=""
  ["exp10_cupola_2"]="-p saif.force_lio_only_en:=true"
  ["exp11_lower_gallery"]="-p lio.baseline_octree_map_en:=true -p saif.force_lio_only_en:=true"
  ["exp14_basement_2"]="-p lio.sigma2_state_cov_scale:=0.1 -p saif.force_lio_only_en:=true -p imu.imu_int_frame:=900"
  ["exp18_corridor_lower_gallery_2"]="-p lio.plane_var_scale:=10.0 -p imu.imu_int_frame:=900 -p lio.beam_err:=0.1 -p saif.force_lio_only_en:=true"
  ["exp21_outside_building"]=""
)

# Gate-pending (stable but not yet 达标; enable only after performance gate
# confirms): exp03 strict full (0.9725), exp09 LIO-only+m200 (0.4257),
# exp15 LIO-only (0.3510), exp16 LIO-only (0.2724).

for seq in "${!CFG[@]}"; do
  bag=$(find "$ROOT/${seq}_ros2_full" -name '.subsample_tmp_*.db3' | head -1)
  if [ -z "$bag" ]; then echo "SKIP (no db3): $seq"; continue; fi
  echo "=== $seq ($bag) ==="
  XDG_STATE_HOME=/tmp/sa_state ROS_HOME=/tmp/ros_home \
    SA_EXPECTED_PROCESSED=100000 SA_MIN_POSES=100 \
    SA_HILTI_EXTRA_ARGS="-p pcd_save.pcd_save_en:=true ${CFG[$seq]}" \
    bash "$ROOT/scripts/run_hilti_direct.sh" "$bag" "$seq" pcd
  echo "=== $seq done ==="
done
echo "ALL HILTI PCD GENERATION DONE"
