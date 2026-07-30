#!/bin/bash
# 一键安装 Marsdog conda 环境 (pinocchio / scipy / osqp / qpsolvers …)
#
# 用法:
#   cd /path/to/marsdog_v3_test3
#   ./scripts/setup_env.sh
#
# 装好后:
#   source ~/miniforge3/bin/activate marsdog
#   ./run_walk.sh --help
#
# 说明:
# - Miniforge 装到 $HOME/miniforge3（可改 MF_PREFIX）
# - 默认走清华 conda-forge 镜像，aarch64 板子也能装 pinocchio
# - 不要把 miniforge3 提交进 git；保留本机目录则下次几乎不用重下
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MF_PREFIX="${MF_PREFIX:-$HOME/miniforge3}"
ENV_NAME="${ENV_NAME:-marsdog}"
ENV_FILE="${ROOT}/environment.yml"
ARCH="$(uname -m)"

echo "[setup] repo=$ROOT"
echo "[setup] prefix=$MF_PREFIX  env=$ENV_NAME  arch=$ARCH"

# ── 1) Miniforge ──────────────────────────────────────────────────
if [ ! -x "$MF_PREFIX/bin/mamba" ] && [ ! -x "$MF_PREFIX/bin/conda" ]; then
  echo "[setup] 未找到 Miniforge，开始下载安装…"
  TMP_SH="$(mktemp /tmp/Miniforge3-XXXXXX.sh)"
  URL_TUNA="https://mirrors.tuna.tsinghua.edu.cn/github-release/conda-forge/miniforge/LatestRelease/Miniforge3-Linux-${ARCH}.sh"
  URL_GH="https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-${ARCH}.sh"
  if ! curl -fL --connect-timeout 20 --max-time 300 -o "$TMP_SH" "$URL_TUNA"; then
    echo "[setup] 清华镜像失败，改试 GitHub…"
    curl -fL --connect-timeout 20 --max-time 600 -o "$TMP_SH" "$URL_GH"
  fi
  bash "$TMP_SH" -b -p "$MF_PREFIX"
  rm -f "$TMP_SH"
else
  echo "[setup] 已有 Miniforge: $MF_PREFIX"
fi

MAMBA="$MF_PREFIX/bin/mamba"
CONDA="$MF_PREFIX/bin/conda"
if [ ! -x "$MAMBA" ]; then
  MAMBA="$CONDA"
fi

# ── 2) 国内镜像（可跳过: SKIP_MIRROR=1）──────────────────────────
if [ "${SKIP_MIRROR:-0}" != "1" ]; then
  cat > "$HOME/.condarc" <<'EOF'
channels:
  - conda-forge
show_channel_urls: true
channel_priority: strict
custom_channels:
  conda-forge: https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud
EOF
  echo "[setup] 已写入 ~/.condarc (清华 conda-forge)"
fi

# ── 3) 创建 / 更新环境 ──────────────────────────────────────────
export PYTHONPATH=
if "$CONDA" env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  echo "[setup] 环境 $ENV_NAME 已存在 → 按 environment.yml 更新"
  "$MAMBA" env update -n "$ENV_NAME" -f "$ENV_FILE" --prune
else
  echo "[setup] 创建环境 $ENV_NAME"
  "$MAMBA" env create -n "$ENV_NAME" -f "$ENV_FILE"
fi

# ── 4) 自检 ──────────────────────────────────────────────────────
PY="$MF_PREFIX/envs/$ENV_NAME/bin/python"
export PYTHONNOUSERSITE=1
"$PY" - <<'PY'
import pinocchio as pin, numpy, scipy, osqp, qpsolvers, serial, yaml
print("[ok] pinocchio", pin.__version__)
print("[ok] numpy", numpy.__version__)
print("[ok] scipy", scipy.__version__)
print("[ok] osqp", osqp.__version__)
print("[ok] qpsolvers", qpsolvers.__version__)
print("[ok] pyserial/yaml")
PY

echo
echo "[done] 激活:  source $MF_PREFIX/bin/activate $ENV_NAME"
echo "[done] 行走:  cd $ROOT && ./run_walk.sh --help"
echo "[done] 体检:  cd $ROOT/mocap_to_real && PYTHONPATH=$ROOT/src:\$PYTHONPATH python static_test.py"
