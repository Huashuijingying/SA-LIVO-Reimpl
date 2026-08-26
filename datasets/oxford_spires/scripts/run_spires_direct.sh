#!/usr/bin/env bash
# Deterministic in-process bag replay for Oxford Spires (CompressedImage + QT64).
set -Eeo pipefail
BAG="$1"; SEQ="$2"; SUFFIX="$3"
EXPECTED_PROCESSED="${SA_EXPECTED_PROCESSED:-2800}"
MIN_POSES="${SA_MIN_POSES:-2750}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
PKG_SRC="${SA_LIVO_SRC:-$REPO_ROOT/sa_livo}"
STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/sa_livo"
mkdir -p "$STATE_DIR"
NODE_LOG="$STATE_DIR/direct_spires_${SUFFIX}.log"
TRAJ="$PKG_SRC/Log/result/${SEQ}_${SUFFIX}.txt"

MAIN_YAML="${SA_SPIRES_MAIN_YAML:-$PKG_SRC/config/oxford_spires.yaml}"
CAM_YAML="${SA_SPIRES_CAMERA_YAML:-$PKG_SRC/config/camera_oxford_spires.yaml}"
EXTRA="${SA_SPIRES_EXTRA_ARGS:-}"

rm -f "$NODE_LOG" "$TRAJ"
set +u
source /opt/ros/humble/setup.bash
source "${SA_LIVO_WS:-/home/zxy/slam/fast_livo_ws}/install/setup.bash"
set -u
export XDG_STATE_HOME="${XDG_STATE_HOME:-/tmp/sa_state}"
export ROS_HOME="${ROS_HOME:-/tmp/ros_home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROS_HOME/log}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"

ros2 run sa_livo fastlivo_mapping \
  --ros-args -r __node:=laserMapping \
  --params-file "$MAIN_YAML" \
  --params-file "$CAM_YAML" \
  -p common.direct_bag_path:="$BAG" \
  -p evo.seq_name:="${SEQ}_${SUFFIX}" \
  -p evo.pose_output_en:=true \
  $EXTRA > "$NODE_LOG" 2>&1 &
NODE_PID=$!

DIRECT_DONE=0; STALL_COUNT=0; LAST_PROCESSED=-1
for _ in $(seq 1 3600); do
  if ! kill -0 "$NODE_PID" 2>/dev/null; then
    echo "node exited" >&2
    break
  fi
  grep -aq '\[ DIRECT BAG \] finished' "$NODE_LOG" && DIRECT_DONE=1
  processed=$(grep -aFc '[ PIPELINE ] processed VIO frame:' "$NODE_LOG" 2>/dev/null || true)
  processed=$(printf '%s' "$processed" | tr -cd '0-9')
  [ -z "$processed" ] && processed=0
  poses=0
  [ -f "$TRAJ" ] && poses=$(wc -l < "$TRAJ")
  if [ "$processed" -ge "$EXPECTED_PROCESSED" ] && [ "$poses" -ge "$MIN_POSES" ]; then
    break
  fi
  if [ "$DIRECT_DONE" -eq 1 ]; then
    if [ "$processed" -eq "$LAST_PROCESSED" ]; then
      STALL_COUNT=$((STALL_COUNT + 1))
      [ "$STALL_COUNT" -ge 4 ] && break
    else
      STALL_COUNT=0
    fi
    LAST_PROCESSED=$processed
  fi
  sleep 5
done

# Wait for natural exit (node self-terminates after draining; savePCD runs).
for _ in $(seq 1 240); do
  if ! kill -0 "$NODE_PID" 2>/dev/null; then echo "node exited"; break; fi
  sleep 5
done
if kill -0 "$NODE_PID" 2>/dev/null; then
  echo "node still alive after 20min, killing" >&2
  kill -9 "$NODE_PID" 2>/dev/null || true
fi
poses=0
[ -f "$TRAJ" ] && poses=$(wc -l < "$TRAJ")
echo "done: processed=$processed poses=$poses"
