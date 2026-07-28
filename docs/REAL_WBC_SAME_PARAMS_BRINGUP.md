# Real bring-up: same params as sim WBC soft-trot

Goal: first real walk uses **identical** `NATURAL_SOFT_TROT_WBC` numbers as the
validated MuJoCo run, then fine-tune from telemetry diffs. Do **not** invent a
second “safe” gait preset until the same-param baseline is recorded.

## Frozen control path (sim ↔ real)

| Item | Value |
|---|---|
| Preset | `NATURAL_SOFT_TROT_WBC` (applied when `--natural-soft-trot --wbc`) |
| Estimate | `--base-estimate-mode estimator` (required on real; default in `sim_walk`) |
| Force stack | `--wbc --no-vmc` |
| Truth in control | **Forbidden** (`truth` is debug-only; lateral damp must not mix MuJoCo `vel_xyz`) |

## Commands

### Spot-turn (abduction-led in-place)

```bash
# Headless check
PYTHONPATH=src python -m marsdog_control.apps.sim.sim_walk --wbc --headless \
  --duration 14 --vx 0 --turn 0.85

# Viewer
PYTHONPATH=src python -m marsdog_control.apps.sim.sim_walk --wbc --vx 0 --turn 0.8
```

Expect: `amp=0`, hip abduction relocates swing feet, yaw accumulates, roll stays modest.

Artifacts (repo root unless redirected):

- `telemetry.csv` / `telemetry.json` — full ring
- `telemetry_summary.json` — compact metrics for real diff

Copy a known-good summary to `docs/baselines/` after a clean run:

```bash
cp telemetry_summary.json docs/baselines/sim_wbc_estimator_summary.json
```

### Real first WBC session (same flags)

```bash
./run_walk.sh --natural-soft-trot --wbc --no-vmc --base-estimate-mode estimator
# optional: --no-tail ; keep gamepad for estop once standing is OK
```

Stand / soft path without WBC first if bring-up is cold:

```bash
./run_walk.sh --no-gamepad --no-tail --no-log
```

## Pre-flight checklist

- [ ] Tether / hoist; clear estop / power cut
- [ ] Battery OK; CAN / IMU nodes present; tarsus manually zeroed
- [ ] Same git commit as sim baseline (or note delta)
- [ ] Log directory writable; plan to save `telemetry_*` or walk CSV
- [ ] Operator: stand → tiny stick → abort if roll/current spikes

## Same-param compare metrics

After a real run, compare against `docs/baselines/sim_wbc_estimator_summary.json`:

| Key | Why |
|---|---|
| `roll_peak_deg` / `roll_p95_deg` | Attitude budget |
| `vx_cmd_mean` / `vx_est_mean` / `vx_est_minus_cmd_mean` | Speed track / fake brake |
| `contact_mismatch_pct` | Schedule vs measured contact |
| `amp_*` / `period_mean_s` / `speed_frac_mean` | Confirm same schedule |
| `q_err_rms_mean_deg` | Joint tracking under WBC kp scale |
| `fz_peak_n` / `dtau_p95` | Impact / torque churn |
| `mpc_ok_pct` / `wbc_ok_pct` | Solver health |
| `estimate_mode` | Must be `estimator` |

Fine-tune only after this table is filled once with **unchanged** gait numbers.
