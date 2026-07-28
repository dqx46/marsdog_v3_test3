#!/usr/bin/env bash
# Estimator-only WBC soft-trot smoke (real-parity control path).
# Does not tune gait params — only verifies the deploy-relevant stack runs.
#
#   ./scripts/estimator_wbc_smoke.sh [duration_s]
#
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$DIR"
export PYTHONPATH="${DIR}/src:${PYTHONPATH:-}"

DURATION="${1:-12}"
PY="${PYTHON:-}"
if [[ -z "$PY" ]]; then
  if [[ -x /home/z/miniforge3/envs/gmr/bin/python ]]; then
    PY=/home/z/miniforge3/envs/gmr/bin/python
  else
    PY=python3
  fi
fi

echo "── estimator WBC smoke (${DURATION}s) via ${PY} ──"
"$PY" -m marsdog_control.apps.sim.sim_walk \
  --wbc --headless --duration "$DURATION" \
  --base-estimate-mode estimator

test -f telemetry_summary.json
test -f telemetry.csv

MODE="$("$PY" -c "import json; print(json.load(open('telemetry_summary.json')).get('estimate_mode',''))")"
if [[ "$MODE" != "estimator" ]]; then
  echo "FAIL: estimate_mode=${MODE@Q} (want estimator)" >&2
  exit 1
fi

echo "── OK: telemetry_summary.json (estimate_mode=estimator) ──"
"$PY" -c "import json; s=json.load(open('telemetry_summary.json')); print(f\"roll_p95={s['roll_p95_deg']:.1f}deg mismatch={s['contact_mismatch_pct']:.1f}% vx_est-cmd={s['vx_est_minus_cmd_mean']:+.3f}\")"

BASELINE_DIR="$DIR/docs/baselines"
mkdir -p "$BASELINE_DIR"
if [[ "${UPDATE_BASELINE:-0}" == "1" ]]; then
  cp -f telemetry_summary.json "$BASELINE_DIR/sim_wbc_estimator_summary.json"
  echo "Wrote $BASELINE_DIR/sim_wbc_estimator_summary.json"
else
  echo "Tip: UPDATE_BASELINE=1 $0  # refresh docs/baselines/sim_wbc_estimator_summary.json"
fi
