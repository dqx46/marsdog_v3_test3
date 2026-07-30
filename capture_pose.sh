#!/bin/bash
# 交互式姿势捕获：拖动示教 → z=坐下 / p=趴下
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"
export PYTHONPATH="$DIR/src:$DIR/mocap_to_real:${PYTHONPATH:-}"
export PYTHONNOUSERSITE=1

PY=python3
if [ -x "${HOME}/miniforge3/envs/marsdog/bin/python" ]; then
  PY="${HOME}/miniforge3/envs/marsdog/bin/python"
elif [ -x "/home/cat/miniforge3/envs/marsdog/bin/python" ]; then
  PY="/home/cat/miniforge3/envs/marsdog/bin/python"
fi
exec "$PY" -m marsdog_control.apps.tools.calibration.capture_pose "$@"
