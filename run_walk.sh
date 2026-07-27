#!/bin/bash
# Marsdog 行走主入口 —— 推荐使用重构后的新包入口。
#   ./run_walk.sh [walk 参数...]      例: ./run_walk.sh --no-gamepad --no-tail
# 兼容旧入口(等价实现): python3 mocap_to_real/walk.py [参数...]
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"
export PYTHONPATH="$DIR/src:$DIR/mocap_to_real:${PYTHONPATH:-}"
exec python3 -m marsdog_control.apps.walk "$@"
