#!/bin/bash
# Marsdog 行走主入口 —— 推荐使用重构后的新包入口。
#   ./run_walk.sh [walk 参数...]      例: ./run_walk.sh --no-gamepad --no-tail
#   步频: ./run_walk.sh --gait-period 1.0   或  ./run_walk.sh --gait-hz 1.15
# 兼容旧入口(等价实现): python3 mocap_to_real/walk.py [参数...]
#
# 优先使用 conda 环境 marsdog (含 pinocchio); 否则回退系统 python3。
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
exec "$PY" -m marsdog_control.apps.walk "$@"
