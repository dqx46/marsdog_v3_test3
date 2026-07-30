#!/bin/bash
# 用 marsdog conda 环境跑任意命令，例:
#   ./run_with_env.sh python mocap_to_real/static_test.py
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"
export PYTHONPATH="$DIR/src:$DIR/mocap_to_real:${PYTHONPATH:-}"
export PYTHONNOUSERSITE=1
PY="${HOME}/miniforge3/envs/marsdog/bin/python"
[ -x "$PY" ] || PY="/home/cat/miniforge3/envs/marsdog/bin/python"
if [ ! -x "$PY" ]; then
  echo "未找到 conda 环境 marsdog: $PY" >&2
  exit 1
fi
# 若第一个参数是 *.py / -m / 模块风格，用 env python 执行；否则原样 exec
if [ $# -eq 0 ]; then
  echo "用法: $0 <命令...>" >&2
  exit 2
fi
case "$1" in
  python|python3) shift; exec "$PY" "$@" ;;
  *) exec "$@" ;;
esac
