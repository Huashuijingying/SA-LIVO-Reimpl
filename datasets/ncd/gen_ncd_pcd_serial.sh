#!/usr/bin/env bash
# Serial raw-PCD generation for NCD 2021, one sequence at a time.
# Updated 2026-08-16: db3 FILE paths, per-sequence best configs,
# underground easy/medium added (LIO-only + i900).
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# bag dir|seq|extra args
declare -A CFG=(
  ["2021-04-12-11-06-47-easy|ug-easy"]="-p saif.force_lio_only_en:=true -p imu.imu_int_frame:=900"
  ["2021-04-12-11-11-33-medium|ug-medium"]="-p saif.force_lio_only_en:=true -p imu.imu_int_frame:=900"
  ["2021-07-01-11-31-35_0-quad-medium|quad-medium"]=""
  ["2021-07-01-11-35-14_0-quad-hard|quad-hard"]="-p lio.plane_var_scale:=10.0 -p saif.force_lio_only_en:=true"
)

# Gate-pending: quad-easy LIO-only (0.0645, 2.48x); stairs divergent;
# ug-hard data blocked (Drive quota, email form pending).

for key in "${!CFG[@]}"; do
  bagdir="${key%%|*}"; seq="${key##*|}"
  bag=$(find "$ROOT/${bagdir}_ros2" -name '.subsample_tmp_*.db3' | head -1)
  if [ -z "$bag" ]; then echo "SKIP (no db3): $seq"; continue; fi
  echo "=== $seq ($bag) ==="
  XDG_STATE_HOME=/tmp/sa_state ROS_HOME=/tmp/ros_home \
    SA_NCD_EXTRA_ARGS="-p pcd_save.pcd_save_en:=true ${CFG[$key]}" \
    bash "$ROOT/run_ncd_direct.sh" "$bag" "ncd_$seq" pcd
  echo "=== $seq done ==="
done
echo "ALL NCD PCD DONE"
