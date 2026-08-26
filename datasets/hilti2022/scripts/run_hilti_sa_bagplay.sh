#!/usr/bin/env bash
# SA-LIVO on one HILTI'22 sequence via ros2 bag play (same replay mode as the
# FAST-LIVO2 baseline runner, so results are directly comparable).
set -Eeuo pipefail

BAGDIR="$1"
SEQ="$2"
SUFFIX="$3"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
WS="${SA_LIVO_WS:-/home/zxy/slam/fast_livo_ws}"
PKG_SRC="${SA_LIVO_SRC:-$REPO_ROOT/sa_livo}"
MAIN_YAML="${SA_HILTI_MAIN_YAML:-$PKG_SRC/config/hilti_2022.yaml}"
CAM_YAML="${SA_HILTI_CAMERA_YAML:-$PKG_SRC/config/camera_hilti_2022.yaml}"
QOS="${SA_HILTI_QOS:-$REPO_ROOT/datasets/hilti2022/qos_overrides_hilti.yaml}"
TRAJ="$PKG_SRC/Log/result/${SEQ}_${SUFFIX}.txt"
EXTRA="${SA_HILTI_EXTRA_ARGS:-}"

STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/sa_livo_bagplay"
mkdir -p "$STATE_DIR" "$PKG_SRC/Log/result"
NODE_LOG="$STATE_DIR/node_${SEQ}_${SUFFIX}.log"
BAG_LOG="$STATE_DIR/bag_${SEQ}_${SUFFIX}.log"

exec 9>"$STATE_DIR/sa_bagplay.lock"
flock -n 9 || { echo "refusing to start: another SA bagplay replay holds the lock" >&2; exit 1; }

export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 MKL_NUM_THREADS=4
unset PYTHONPATH
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID=96
export ROS_LOG_DIR="$STATE_DIR/roslog"
mkdir -p "$ROS_LOG_DIR"

rm -f "$NODE_LOG" "$BAG_LOG" "$TRAJ"

stop_pid() {
  local pid="$1" name="$2" target="$1"
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 100); do kill -0 "$pid" 2>/dev/null || break; sleep 0.2; done
    if kill -0 "$pid" 2>/dev/null; then
      echo "$name did not exit; KILL" >&2
      kill -9 "$pid" 2>/dev/null || true
    fi
  fi
  wait "$pid" 2>/dev/null || true
}

NODE_PID=""
on_exit() {
  trap - EXIT INT TERM
  [[ -n "$NODE_PID" ]] && stop_pid "$NODE_PID" mapping-node
}
trap on_exit EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid ros2 run sa_livo fastlivo_mapping --ros-args \
  --params-file "$MAIN_YAML" \
  --params-file "$CAM_YAML" \
  -p evo.seq_name:="${SEQ}_${SUFFIX}" \
  -p evo.pose_output_en:=true \
  $EXTRA >"$NODE_LOG" 2>&1 &
NODE_PID=$!
sleep 4
kill -0 "$NODE_PID" || { echo "mapping node failed: $NODE_LOG" >&2; exit 1; }

set +e
ros2 bag play "$BAGDIR" --rate 0.5 \
  --disable-keyboard-controls \
  --qos-profile-overrides-path "$QOS" >"$BAG_LOG" 2>&1
bag_rc=$?
set -e
(( bag_rc == 0 )) || { echo "bag player failed ($bag_rc): $BAG_LOG" >&2; exit 1; }

# drain: wait until LIO frame count is stable for 10 s
last=-1; stable=0
for _ in $(seq 1 240); do
  n=$(grep -aFc 'SA-LIVO Joint Update Time' "$NODE_LOG" 2>/dev/null || true)
  if [ "$n" = "$last" ]; then stable=$((stable+1)); else stable=0; last="$n"; fi
  [ "$stable" -ge 10 ] && break
  sleep 1
done
stop_pid "$NODE_PID" mapping-node
NODE_PID=""
trap - EXIT INT TERM

if [ ! -s "$TRAJ" ]; then
  echo "no trajectory written" >&2
  exit 1
fi
echo "trajectory: $TRAJ ($(wc -l <"$TRAJ") poses)"
