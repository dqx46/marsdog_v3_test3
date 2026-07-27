#!/bin/bash
# 本地/CI 质量门禁 —— 提交/推送前跑这一个脚本就够了。
#
#   ./scripts/check.sh              跑全部四道闸门
#   ./scripts/check.sh --fast       跳过 coverage(离线单测已经跑两遍很慢时用)
#
# 四道闸门(任一失败, 非零退出):
#   1. compileall  — 全仓字节码编译, 零依赖, 抓语法错误/编码问题
#   2. pyflakes    — 只对"核心路径"严格清零(未用变量/未定义名/f-string 空占位符)
#                    apps/tools/**、apps/tools/legacy_apps/**、manual_tests/** 是
#                    历史工具脚本仓库噪音, 不在此门禁范围(REFACTOR_STATUS.md 有说明)
#   3. unittest    — tests/ 离线单元 + tests/parity/ 逐字节金样对照
#   4. coverage    — 用 (3) 的运行轨迹跑一次 coverage, 只报告不设死线(还没有历史
#                    基线, 硬性阈值会先卡住而不是先保护) —— 报告写到 coverage_html/
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$DIR"
export PYTHONPATH="$DIR/src:$DIR/mocap_to_real:${PYTHONPATH:-}"

FAST=0
if [[ "${1:-}" == "--fast" ]]; then FAST=1; fi

# 核心路径清单: 稳态热路径 + 装配 + 硬件抽象 + 安全 + 配置; 不含工具/legacy shim。
CORE_PATHS=(
  src/marsdog_control/runtime
  src/marsdog_control/control
  src/marsdog_control/motion
  src/marsdog_control/hardware
  src/marsdog_control/safety
  src/marsdog_control/config
  src/marsdog_control/core
  src/marsdog_control/input
  src/marsdog_control/io
  src/marsdog_control/apps/walk_cli.py
  src/marsdog_control/compat.py
  tests
)

echo "── [1/4] compileall (语法/编码, 全仓零依赖) ──"
python3 -m compileall -q src mocap_to_real tests manual_tests

echo "── [2/4] pyflakes (核心路径严格清零) ──"
if python3 -c "import pyflakes" >/dev/null 2>&1; then
  # pyflakes 没有 --exclude; tests/Motor_test/ 是手动绘图工具而非测试, 单独过滤掉。
  python3 -m pyflakes "${CORE_PATHS[@]}" 2>&1 | grep -v '^tests/Motor_test/' > /tmp/_check_pyflakes.out || true
  if [[ -s /tmp/_check_pyflakes.out ]]; then
    echo "[FAIL] 核心路径 pyflakes 告警(见上), 门禁范围详见脚本头注释:"
    cat /tmp/_check_pyflakes.out
    exit 1
  fi
  echo "  核心路径零告警。"
else
  echo "  [SKIP] pyflakes 未安装 (pip3 install --user pyflakes 后重跑本脚本可启用)"
fi

echo "── [3/4] 离线单元 + parity ──"
python3 -m unittest discover -s tests -p "test_*.py"
python3 -m unittest discover -s tests/parity -p "test_*.py"

if [[ "$FAST" == "1" ]]; then
  echo "── [4/4] coverage: --fast 跳过 ──"
  echo "[OK] 全部闸门通过 (未测 coverage)。"
  exit 0
fi

echo "── [4/4] coverage (只报告, 暂无历史基线不设死线) ──"
if python3 -c "import coverage" >/dev/null 2>&1; then
  python3 -m coverage run --source=src/marsdog_control \
    -m unittest discover -s tests -p "test_*.py" >/dev/null
  python3 -m coverage run -a --source=src/marsdog_control \
    -m unittest discover -s tests/parity -p "test_*.py" >/dev/null
  python3 -m coverage report --skip-empty | tail -n 5
  python3 -m coverage html -d coverage_html >/dev/null
  echo "  详细报告: coverage_html/index.html"
else
  echo "  [SKIP] coverage 未安装 (pip3 install --user coverage 后重跑本脚本可启用)"
fi

echo "[OK] 全部闸门通过。"
