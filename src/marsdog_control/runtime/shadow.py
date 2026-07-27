"""Shadow comparison helpers for RuntimePipeline takeover.

The legacy loop remains the actuator of record while this helper compares the
candidate pipeline output against the command frame produced by the Board path.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping

from marsdog_control.core.types import ControlOutput, MotorCommandFrame


@dataclass
class ShadowDiff:
    ok: bool
    max_q_error: float = 0.0
    max_dq_error: float = 0.0
    max_kp_error: float = 0.0
    max_kd_error: float = 0.0
    max_tau_error: float = 0.0
    mismatched_ids: list[int] = field(default_factory=list)


def _max_diff(a: Mapping[int, float], b: Mapping[int, float], ids) -> tuple[float, list[int]]:
    max_err = 0.0
    bad = []
    for mid in ids:
        err = abs(float(a.get(mid, 0.0)) - float(b.get(mid, 0.0)))
        max_err = max(max_err, err)
        if err > 1e-9:
            bad.append(mid)
    return max_err, bad


def compare_output_to_board(output: ControlOutput, command: MotorCommandFrame,
                            *, tolerance: float = 1e-9) -> ShadowDiff:
    """Compare pipeline ControlOutput intent to the Board command snapshot."""
    ids = set(output.target.q) | set(command.target_q)
    max_q, bad_q = _max_diff(output.target.q, command.target_q, ids)
    max_dq, bad_dq = _max_diff(output.target.dq, command.target_dq, ids)
    max_tau, bad_tau = _max_diff(output.trq_ff or {}, command.torque_ff, ids)

    # kp/kd are not carried by ControlOutput directly; they are Board-resolved
    # command facts. Keep them in the report so callers can compare two Board
    # command frames when shadowing a full executor path.
    mismatched = sorted(set(bad_q) | set(bad_dq) | set(bad_tau))
    return ShadowDiff(
        ok=(max_q <= tolerance and max_dq <= tolerance and max_tau <= tolerance),
        max_q_error=max_q,
        max_dq_error=max_dq,
        max_tau_error=max_tau,
        mismatched_ids=mismatched,
    )


def compare_command_frames(expected: MotorCommandFrame, actual: MotorCommandFrame,
                           *, tolerance: float = 1e-9) -> ShadowDiff:
    ids = set(expected.target_q) | set(actual.target_q)
    max_q, bad_q = _max_diff(expected.target_q, actual.target_q, ids)
    max_dq, bad_dq = _max_diff(expected.target_dq, actual.target_dq, ids)
    max_kp, bad_kp = _max_diff(expected.kp, actual.kp, ids)
    max_kd, bad_kd = _max_diff(expected.kd, actual.kd, ids)
    max_tau, bad_tau = _max_diff(expected.torque_ff, actual.torque_ff, ids)
    mismatched = sorted(set(bad_q) | set(bad_dq) | set(bad_kp) | set(bad_kd) | set(bad_tau))
    return ShadowDiff(
        ok=(
            max_q <= tolerance and max_dq <= tolerance and max_kp <= tolerance
            and max_kd <= tolerance and max_tau <= tolerance
        ),
        max_q_error=max_q,
        max_dq_error=max_dq,
        max_kp_error=max_kp,
        max_kd_error=max_kd,
        max_tau_error=max_tau,
        mismatched_ids=mismatched,
    )


__all__ = ["ShadowDiff", "compare_output_to_board", "compare_command_frames"]
