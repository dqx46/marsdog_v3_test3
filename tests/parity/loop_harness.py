"""Boot the real `walk.main` control loop offline and record its command stream.

This is the reference generator for full-loop parity: with fake drivers, a fake
clock, and a bounded number of iterations, the *actual legacy loop* runs end to
end (startup -> calibrate -> stand -> loop) on a dev box with no robot, and we
capture every ``send_all`` it issues. That recorded stream is the golden the
refactored ``RuntimePipeline`` must reproduce byte-for-byte — a mathematical
equivalence proof that needs no hardware.
"""

from __future__ import annotations

import os
import sys
import time as _real_time

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_HERE, _ROOT, os.path.join(_ROOT, "src"), os.path.join(_ROOT, "mocap_to_real")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.compat import ensure_legacy_path, install_offline_stubs  # noqa: E402

from fake_hardware import (  # noqa: E402
    FakeDm, FakeEvo, FakeImu, FakeIncos, FakeKeyReader, FakeLz, FakeTail,
)


class _FakeClock:
    """Monotonic-ish clock that only advances when someone sleeps.

    Reads are stable between sleeps (so multiple time.time() calls inside one
    loop iteration agree), and each frequency sleep at the end of a loop tick
    steps the clock by the requested amount -> deterministic phase progression.
    """

    def __init__(self, start=1000.0):
        self.t = start

    def time(self):
        return self.t

    def monotonic(self):
        return self.t

    def sleep(self, dt):
        if dt and dt > 0:
            self.t += dt

    def __getattr__(self, name):
        return getattr(_real_time, name)


class _Recorder:
    def __init__(self, n_ticks):
        self.n_ticks = n_ticks
        self.ticks = 0
        self.recording = False
        self.sends = []


def _normalize_map(m):
    if not m:
        return None
    return {int(k): round(float(v), 9) for k, v in m.items()}


def run_legacy_loop(extra_argv=None, n_ticks=8):
    """Run walk.main offline for ~n_ticks loop iterations; return the send stream."""
    install_offline_stubs()
    ensure_legacy_path()

    import walk  # noqa: E402  (imported after stubs/path are ready)
    import marsdog_control.runtime.walk_services as walk_services  # noqa: E402

    rec = _Recorder(n_ticks)
    clock = _FakeClock()

    saved = {}

    def _save(name):
        saved[name] = getattr(walk, name)

    for _n in ("MotorLz", "MotorEvo", "MotorDamiao", "MotorIncos", "ImuWT901", "KeyReader",
               "TailController", "time", "bark_with_mouth"):
        _save(_n)
    saved_exists = os.path.exists

    # Parity seam is anchored on the real I/O owner (WalkServices). The old thin
    # walk.send_all / walk.read_state module delegators have been removed; main now
    # injects svc.send_all / svc.read_state (bound to the patched class), so patching
    # these class methods proves the component itself is byte-equivalent.
    Services = walk_services.WalkServices
    real_read_state = Services.read_state
    real_send_all = Services.send_all

    def fake_read_state(self, lz, evo, dm, incos, imu, online):
        # First call = loop has started -> begin recording loop sends only.
        if rec.ticks >= rec.n_ticks:
            rec.recording = False  # stop before cleanup ramps (don't record those)
            raise KeyboardInterrupt  # clean, bounded stop via walk's own handler
        rec.recording = True
        rec.ticks += 1
        return real_read_state(self, lz, evo, dm, incos, imu, online)

    def fake_send_all(self, lz, evo, dm, incos, targets, kp_scale=1.0,
                      use_joint_gains=True, **kwargs):
        if rec.recording:
            rec.sends.append({
                "targets": _normalize_map(targets),
                "kp_scale": round(float(kp_scale), 9),
                "velocities": _normalize_map(kwargs.get("velocities")),
                "kp_phase": _normalize_map(kwargs.get("kp_phase")),
                "trq_ff": _normalize_map(kwargs.get("trq_ff")),
                "dm_lead": bool(kwargs.get("dm_reference_lead_active", False)),
            })
        return real_send_all(self, lz, evo, dm, incos, targets, kp_scale=kp_scale,
                             use_joint_gains=use_joint_gains, **kwargs)

    def fake_exists(path):
        # Pretend the configured device nodes exist so startup proceeds.
        if path in (walk.IMU_DEVICE, walk.DM_CAN_DEVICE, walk.LZ_SERIAL_DEVICE,
                    walk.LZ_CAN1_DEVICE, walk.EVO_CAN0_DEVICE, walk.GAMEPAD_DEVICE):
            return True
        return saved_exists(path)

    try:
        walk.MotorLz = FakeLz
        walk.MotorEvo = FakeEvo
        walk.MotorDamiao = FakeDm
        walk.MotorIncos = FakeIncos
        walk.ImuWT901 = FakeImu
        walk.KeyReader = FakeKeyReader
        walk.TailController = FakeTail
        walk.bark_with_mouth = lambda *a, **k: None
        walk.time = clock
        Services.read_state = fake_read_state
        Services.send_all = fake_send_all
        os.path.exists = fake_exists

        argv = ["walk", "--no-gamepad", "--no-log", "--no-tail", "--no-auto-trim"]
        if extra_argv:
            argv += list(extra_argv)
        saved_argv = sys.argv
        try:
            sys.argv = argv
            args = walk.parse_args()
            import contextlib
            with open(os.devnull, "w") as _devnull:
                with contextlib.redirect_stdout(_devnull):
                    walk.main(args)
        finally:
            sys.argv = saved_argv
    finally:
        for _n, _v in saved.items():
            setattr(walk, _n, _v)
        Services.read_state = real_read_state
        Services.send_all = real_send_all
        os.path.exists = saved_exists

    return rec.sends


if __name__ == "__main__":
    stream = run_legacy_loop(n_ticks=5)
    print(f"recorded {len(stream)} loop sends")
    for i, s in enumerate(stream):
        n = len(s["targets"]) if s["targets"] else 0
        print(f"  tick {i}: {n} targets, kp_scale={s['kp_scale']}")
