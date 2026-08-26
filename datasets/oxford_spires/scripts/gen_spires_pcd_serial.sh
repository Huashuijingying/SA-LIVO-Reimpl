#!/usr/bin/env bash
# Serial raw-PCD generation for all Oxford Spires sequences using the default
# (strict) config, matching the standalone re-run trajectories.
set -Eeuo pipefail

declare -A BAGS=(
  ["keble-college-02"]="2024-03-12-keble-college-02"
  ["keble-college-05"]="2024-03-12-keble-college-05"
  ["observatory-quarter-01"]="2024-03-13-observatory-quarter-01"
  ["observatory-quarter-02"]="2024-03-13-observatory-quarter-02"
  ["blenheim-palace-01"]="2024-03-14-blenheim-palace-01"
  ["blenheim-palace-05"]="2024-03-14-blenheim-palace-05"
  ["christ-church-03"]="2024-03-18-christ-church-03"
)

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for seq in keble-college-02 keble-college-05 observatory-quarter-01 observatory-quarter-02 blenheim-palace-01 blenheim-palace-05 christ-church-03; do
  dir="${BAGS[$seq]}"
  # Prefer the 10 Hz blurred+filtered bag used for the evaluated trajectories.
  bag=$(find "$ROOT/$dir/raw/ros2bag" -name "*_blurred_filtered*.db3" | head -1)
  [ -z "$bag" ] && bag=$(find "$ROOT/$dir/raw/ros2bag" -name "*.db3" | head -1)
  if [ -z "$bag" ]; then echo "SKIP (no db3): $seq"; continue; fi
  echo "=== $seq ==="
  XDG_STATE_HOME=/tmp/sa_state ROS_HOME=/tmp/ros_home \
    SA_SPIRES_EXTRA_ARGS="-p pcd_save.pcd_save_en:=true" \
    bash "$ROOT/scripts/run_spires_direct.sh" "$bag" "spires_$seq" pcd
  echo "=== $seq done ==="
done
echo "ALL SPIRES PCD DONE"
