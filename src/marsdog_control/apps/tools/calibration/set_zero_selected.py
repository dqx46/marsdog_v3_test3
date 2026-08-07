#!/usr/bin/env python3
"""指定电机：失能 → 手动摆位 → 写入零位（Flash，达妙除外）。

流程:
  1. 只打开目标电机所在总线（冷启，不热启保位）
  2. 将该总线上所有电机关力矩/失能（避免邻关节顶住导致掰不动）
  3. 实时显示目标角度；摆好后回车
  4. 把当前位置写成零点（复用 set_zero_all 品牌协议）

用法:
  ./run_with_env.sh python -m marsdog_control.apps.tools.calibration.set_zero_selected --ids 2,5
  ./run_with_env.sh python -m marsdog_control.apps.tools.calibration.set_zero_selected --ids 11
  ./run_with_env.sh python -m marsdog_control.apps.tools.calibration.set_zero_selected --ids 4,8

⚠️  灵足/EVO/因克斯会永久写 Flash。达妙无掉电记忆。
"""

from __future__ import annotations

import argparse
import math
import select
import sys
import time

from marsdog_control.apps.tools.calibration.set_zero_all import (
    OLD_FW_IDS,
    set_zero_dm,
    set_zero_evo,
    set_zero_incos,
    set_zero_lz,
)
# Device paths: same SSOT as run_walk (get_device_config → bus_config).
from marsdog_control.config.devices import get_device_config

_dev = get_device_config()
BAUD = _dev.baud
DM_CAN_DEVICE = _dev.dm_can
EVO_CAN0_DEVICE = _dev.evo_can
INCOS_CAN_DEVICE = _dev.incos_can
LZ_CAN1_DEVICE = _dev.lz_can_a
LZ_SERIAL_DEVICE = _dev.lz_can_b
from marsdog_control.config.joints import (
    DM_CAN_IDS,
    DM_MASTER_ID_BY_SLAVE,
    INCOS_CAN_IDS,
    JOINT_BY_ID,
    LZ_CAN_A_IDS,
    LZ_CAN_B_IDS,
)
from marsdog_control.hardware.motors.damiao import MotorDamiao
from marsdog_control.hardware.motors.evo import MEVO_KNOWN_IDS, MotorEvo
from marsdog_control.hardware.motors.incos import (
    DEFAULT_CAN_TIMEOUT_MS,
    QUERY_POSITION,
    MotorIncos,
)
from marsdog_control.hardware.motors.lingzu import (
    RS05_CAN_IDS,
    RS05_SERIAL_IDS,
    MotorLz,
)


def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi


def _parse_ids(s: str) -> list[int]:
    ids = []
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        ids.append(int(part))
    if not ids:
        raise ValueError("空 ID 列表")
    return ids


def _need_buses(joints) -> set[str]:
    need = set()
    for j in joints:
        if j.mtype == "lz":
            if j.motor_id in LZ_CAN_A_IDS:
                need.add("lz_a")
            if j.motor_id in LZ_CAN_B_IDS:
                need.add("lz_b")
        elif j.mtype == "evo":
            need.add("evo")
        elif j.mtype == "incos":
            need.add("incos")
        elif j.mtype == "dm":
            need.add("dm")
    return need


def _free_lz_bus(lz, ids, *, pulses: int = 5) -> None:
    """Repeated Disable(0x4) so hot-park MIT cannot keep holding."""
    for _ in range(pulses):
        for mid in ids:
            try:
                lz.disable(mid, clear_fault=True)
            except Exception:
                pass
            time.sleep(0.003)
        time.sleep(0.02)


def _free_evo_bus(evo, *, pulses: int = 5) -> None:
    for _ in range(pulses):
        for mid in MEVO_KNOWN_IDS:
            try:
                evo.enter_rest_state(mid)
            except Exception:
                pass
            time.sleep(0.003)
        time.sleep(0.03)


def _free_dm_bus(dm, *, pulses: int = 3) -> None:
    for _ in range(pulses):
        for mid in DM_CAN_IDS:
            try:
                dm.disable(mid)
            except Exception:
                pass
            time.sleep(0.01)


def _free_incos_bus(incos) -> None:
    """Restore CAN timeout, stop keepalive, zero-gain — let firmware drop torque."""
    # 取消 walk 退出时的 timeout=0 park，否则会一直执行上次 MIT。
    for mid in list(incos._active_ids):
        if not incos.is_connected[mid - 1]:
            continue
        try:
            incos.set_can_timeout_ms(mid, DEFAULT_CAN_TIMEOUT_MS, verify=False)
        except Exception:
            pass
        time.sleep(0.002)
    try:
        incos.stop_keepalive()
    except Exception:
        pass
    for mid in list(incos._active_ids):
        if not incos.is_connected[mid - 1]:
            continue
        try:
            # 清掉 _last_cmd 里的保持增益
            q = incos.get_position(mid)
            incos.mit_control(mid, q, 0.0, 0.0, 0.0, 0.0)
            incos.is_enabled[mid - 1] = False
        except Exception:
            pass
        time.sleep(0.002)
    # 默认 500ms 超时后固件掉力；多等一会确保可掰
    time.sleep(0.7)


def _free_opened_buses(need, lz, evo, incos, dm) -> None:
    print("\n[free] 松开已打开总线上的全部电机（含邻关节，避免顶住）...")
    if lz is not None:
        ids = []
        if "lz_a" in need:
            ids.extend(RS05_CAN_IDS)
        if "lz_b" in need:
            ids.extend(RS05_SERIAL_IDS)
        _free_lz_bus(lz, ids)
        print(f"  灵足 disable×5: {ids}")
    if evo is not None:
        _free_evo_bus(evo)
        print(f"  泉智博 REST×5: {MEVO_KNOWN_IDS}")
    if incos is not None:
        _free_incos_bus(incos)
        print(f"  因克斯 停保活+零增益+timeout→{DEFAULT_CAN_TIMEOUT_MS}ms: "
              f"{list(incos._active_ids)}")
    if dm is not None:
        _free_dm_bus(dm)
        print(f"  达妙 disable: {DM_CAN_IDS}")


def _pulse_free_targets(online, need, lz, evo, incos, dm) -> None:
    """Keep targets (and bus neighbors) free while user poses."""
    if lz is not None:
        ids = []
        if "lz_a" in need:
            ids.extend(RS05_CAN_IDS)
        if "lz_b" in need:
            ids.extend(RS05_SERIAL_IDS)
        for mid in ids:
            try:
                lz.disable(mid)
            except Exception:
                pass
    if evo is not None:
        for mid in MEVO_KNOWN_IDS:
            try:
                evo.enter_rest_state(mid)
            except Exception:
                pass
    if dm is not None:
        for mid in DM_CAN_IDS:
            try:
                dm.disable(mid)
            except Exception:
                pass
    # 因克斯：不重新 MIT；只 query 刷新角度（保活已停）
    if incos is not None:
        for j in online:
            if j.mtype != "incos":
                continue
            try:
                incos.query_parameter(j.motor_id, QUERY_POSITION)
            except Exception:
                pass


def _read_pos(j, lz, evo, incos, dm) -> float:
    mid = j.motor_id
    if j.mtype == "lz":
        return float(lz.get_position(mid))
    if j.mtype == "evo":
        return float(evo.get_position(mid))
    if j.mtype == "incos":
        return float(incos.get_position(mid))
    if j.mtype == "dm":
        return float(dm.get_position(mid))
    return float("nan")


def _is_online(j, lz, evo, incos, dm) -> bool:
    mid = j.motor_id
    if j.mtype == "lz":
        return bool(lz is not None and lz.is_connected[mid - 1])
    if j.mtype == "evo":
        return bool(evo is not None and evo.is_connected[mid - 1])
    if j.mtype == "incos":
        return bool(incos is not None and incos.is_connected[mid - 1])
    if j.mtype == "dm":
        if dm is None:
            return False
        online, _p, _e, _l = dm.probe(mid)
        return bool(online)
    return False


def _shutdown(lz, evo, incos, dm):
    """关主机 IO；退出保持使能（摆位阶段的失能是刻意的，退出不再二次 disable）。"""
    for obj in (incos, lz, evo, dm):
        if obj is None:
            continue
        try:
            if obj is dm:
                obj.end()
            else:
                obj.end(disable=False)
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(
        description="指定电机失能 → 手动摆位 → 设零（写 Flash，达妙除外）")
    ap.add_argument(
        "--ids", required=True,
        help="电机 ID，逗号分隔，如 2,3 或 11 或 4,8")
    ap.add_argument(
        "--yes", action="store_true",
        help="跳过最终写 Flash 确认（摆位后的回车仍保留）")
    ap.add_argument(
        "--dry-run", action="store_true",
        help="演习：失能与读角照常，不写零位")
    args = ap.parse_args()

    try:
        target_ids = _parse_ids(args.ids)
    except ValueError as exc:
        print(f"[ERROR] {exc}")
        return 2

    joints = []
    for mid in target_ids:
        j = JOINT_BY_ID.get(mid)
        if j is None or j.bus == "none":
            print(f"[ERROR] 未知/未接线电机 ID: {mid}")
            return 2
        joints.append(j)

    need = _need_buses(joints)
    print("=" * 60)
    print("  指定电机失能 → 摆位 → 设零")
    print(f"  目标: {[(j.motor_id, j.name, j.mtype) for j in joints]}")
    print(f"  总线: {sorted(need)}")
    if args.dry_run:
        print("  模式: DRY-RUN（不写 Flash）")
    print("=" * 60)

    lz = evo = incos = dm = None

    # 热启 clear_fault=False：开启不发 disable/REST；摆位阶段再主动松开目标/邻关节
    if "lz_b" in need or "lz_a" in need:
        lz = MotorLz()
    if "lz_b" in need:
        print(f"\n[init] 灵足 Serial ({LZ_SERIAL_DEVICE})...")
        lz.init_serial(LZ_SERIAL_DEVICE, BAUD, clear_fault=False)
    if "lz_a" in need:
        print(f"[init] 灵足 CAN1 ({LZ_CAN1_DEVICE})...")
        if lz is None:
            lz = MotorLz()
        lz.init_can1_serial(LZ_CAN1_DEVICE, BAUD, clear_fault=False)
    if "evo" in need:
        print(f"[init] 泉智博 ({EVO_CAN0_DEVICE})...")
        evo = MotorEvo()
        evo.init_serial(EVO_CAN0_DEVICE, BAUD, clear_fault=False)
    if "incos" in need:
        print(f"[init] 因克斯 ({INCOS_CAN_DEVICE})...")
        incos = MotorIncos()
        # 打开全部 IncOS：取消其它腿上 timeout=0 park，否则会顶住运动链
        if not incos.begin(INCOS_CAN_DEVICE, INCOS_CAN_IDS, BAUD):
            print("[ERROR] 因克斯总线无应答")
            _shutdown(lz, evo, incos, dm)
            return 1
    if "dm" in need:
        print(f"[init] 达妙 ({DM_CAN_DEVICE})...")
        dm = MotorDamiao()
        if not dm.begin(DM_CAN_DEVICE, BAUD):
            print("[ERROR] 达妙总线打开失败")
            _shutdown(lz, evo, incos, dm)
            return 1
        time.sleep(0.4)
        for mid in DM_CAN_IDS:
            dm.add_motor(mid, master_id=DM_MASTER_ID_BY_SLAVE.get(mid))

    time.sleep(0.2)
    online = []
    for j in joints:
        if _is_online(j, lz, evo, incos, dm):
            online.append(j)
            print(f"  [online] M{j.motor_id:2d} {j.name} [{j.mtype}] "
                  f"{rad2deg(_read_pos(j, lz, evo, incos, dm)):+.1f}°")
        else:
            print(f"  [offline] M{j.motor_id:2d} {j.name} — 跳过")

    if not online:
        print("[ERROR] 目标电机均离线")
        _shutdown(lz, evo, incos, dm)
        return 1

    _free_opened_buses(need, lz, evo, incos, dm)

    print("\n[disable] 目标电机应已可徒手掰动:")
    for j in online:
        note = ""
        if j.mtype == "incos":
            note = " (已停保活，超时掉力)"
        elif j.motor_id in OLD_FW_IDS:
            note = " (老固件 RS00)"
        elif j.mtype == "lz":
            note = " (Disable 0x4；角度可能暂不刷新，以手感为准)"
        print(f"  M{j.motor_id:2d} {j.name}: 自由{note}")

    print("\n[wait] 摆位中。摆好后按回车写入零位；Ctrl+C 取消。")
    try:
        while True:
            _pulse_free_targets(online, need, lz, evo, incos, dm)
            parts = []
            for j in online:
                q = _read_pos(j, lz, evo, incos, dm)
                parts.append(f"M{j.motor_id}:{rad2deg(q):+6.1f}°")
            sys.stdout.write("\r  " + "  ".join(parts) + "    ")
            sys.stdout.flush()
            r, _, _ = select.select([sys.stdin], [], [], 0.15)
            if r:
                sys.stdin.readline()
                break
    except KeyboardInterrupt:
        print("\n[cancel] 用户取消，失能退出")
        _shutdown(lz, evo, incos, dm)
        return 130

    print("\n[pos] 即将写入的当前位置:")
    for j in online:
        print(f"  M{j.motor_id:2d} {j.name:18s}  {rad2deg(_read_pos(j, lz, evo, incos, dm)):+8.2f}°")

    if not args.dry_run and not args.yes:
        print("\n" + "!" * 60)
        print("  ⚠️  即将永久写入 Flash 零位（达妙除外）！")
        print("!" * 60)
        ans = input("  直接回车确认，输入其它内容取消: ")
        if ans != "":
            print("  已取消。")
            _shutdown(lz, evo, incos, dm)
            return 0

    # 设零前因克斯需短暂恢复通信（set_zero 会发协议帧）
    if incos is not None:
        try:
            incos.start_keepalive()
        except Exception:
            pass

    print("\n[set_zero] 写入零位...\n")
    ok_n = 0
    for j in online:
        mid = j.motor_id
        if j.mtype == "lz":
            ok = set_zero_lz(lz, mid, j.name, args.dry_run)
        elif j.mtype == "evo":
            ok = set_zero_evo(evo, mid, j.name, args.dry_run)
        elif j.mtype == "incos":
            ok = set_zero_incos(incos, mid, j.name, args.dry_run)
        elif j.mtype == "dm":
            ok = set_zero_dm(dm, mid, j.name, args.dry_run)
        else:
            ok = False
        if ok:
            ok_n += 1

    print(f"\n[done] {ok_n}/{len(online)} 成功。建议掉电重上后用 static_test 确认 ≈0°。")
    _shutdown(lz, evo, incos, dm)
    return 0 if ok_n == len(online) else 1


if __name__ == "__main__":
    raise SystemExit(main())
