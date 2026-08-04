#!/usr/bin/env python3
"""为承重/腰部电机设置当前位置为零点（永久写入 Flash，达妙除外）。

覆盖当前整机品牌:
  - 灵足 (RS02/EL05/RS00): SetOrigin(0x06) → SaveParams(0x16)
  - 泉智博 PA43:           SetZeroPosition(0xFE，自动写 Flash)
  - 因克斯 EC-A2806:       ENCOS §7.2 广播 0x7FF / 0x03（写零点）
  - 达妙 S2325:            set_zero(0xFE) — **无掉电记忆**，仅当前上电有效

默认跳过: 头/颈 (15–18)。尾巴无关节电机条目。后腿预留 tarsus (bus=none) 跳过。

⚠️  警告: 灵足/EVO/因克斯会永久修改 Flash 零位偏置！
    请确保机器人已摆放到目标零位姿态（URDF 初始站立姿势）后再执行。

用法:
  ./run_with_env.sh python -m marsdog_control.apps.tools.calibration.set_zero_all
  ./run_with_env.sh python -m marsdog_control.apps.tools.calibration.set_zero_all --dry-run
  ./run_with_env.sh python -m marsdog_control.apps.tools.calibration.set_zero_all --ids 2,3,6,7
"""

from __future__ import annotations

import argparse
import math
import time

from marsdog_control.config.bus_config import (
    BAUD,
    DM_CAN_DEVICE,
    EVO_CAN0_DEVICE,
    INCOS_CAN_DEVICE,
    LZ_CAN1_DEVICE,
    LZ_SERIAL_DEVICE,
)
from marsdog_control.config.joints import (
    DM_MASTER_ID_BY_SLAVE,
    INCOS_CAN_IDS,
    JOINT_BY_ID,
    JOINT_MAP,
)
from marsdog_control.hardware.motors.damiao import MotorDamiao
from marsdog_control.hardware.motors.evo import MotorEvo
from marsdog_control.hardware.motors.incos import MotorIncos
from marsdog_control.hardware.motors.lingzu import MotorLz

# 头 / 颈 — 机械结构特殊，单独标定。尾巴当前无 JOINT_MAP 电机。
SKIP_IDS = {15, 16, 17, 18}  # head_pitch/yaw/roll, neck_pitch

# 老固件 RS00 后小腿: set_origin 无效，用 add_offset(0x702B)
OLD_FW_IDS = {11, 14}  # rl_calf, rr_calf


def rad2deg(r):
    return r * 180.0 / math.pi


def set_zero_lz(lz, motor_id, name, dry_run):
    """灵足: Enable → kp=0稳定 → SetOrigin → SaveParams → 验证。"""
    is_old_fw = motor_id in OLD_FW_IDS
    mode_str = "偏置模式(老固件)" if is_old_fw else "标准模式"
    print(f"  Motor {motor_id:2d} ({name}) [灵足/{mode_str}]", end="", flush=True)
    if dry_run:
        print("  [DRY-RUN 跳过]")
        return True

    pos_before = math.degrees(lz.position[motor_id - 1])

    lz.enable(motor_id)
    time.sleep(0.15)

    cur_rad = lz.position[motor_id - 1]
    for _ in range(15):
        lz.mit_control(motor_id, cur_rad, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.003)
    time.sleep(0.05)

    if is_old_fw:
        lz.set_zero_via_offset(motor_id, cur_rad)
        time.sleep(0.15)
        lz.disable(motor_id)
        print(f"  ✓ add_offset 写入完成 (pos={pos_before:+.1f}°，重启电机后生效)")
        return True

    lz.set_origin(motor_id)
    time.sleep(0.08)
    lz.save_params(motor_id)
    time.sleep(0.15)

    for _ in range(10):
        lz.mit_control(motor_id, 0.0, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.005)
    time.sleep(0.15)
    lz.disable(motor_id)

    pos_after = math.degrees(lz.position[motor_id - 1])
    ok = abs(pos_after) < 10.0
    if ok:
        print(f"  ✓ SetOrigin+Flash OK  ({pos_before:+.1f}° → {pos_after:+.1f}°)")
    else:
        print(f"  ✗ 验证失败  ({pos_before:+.1f}° → {pos_after:+.1f}°)")
    return ok


def set_zero_evo(evo, motor_id, name, dry_run):
    """泉智博: SetZeroPosition (自动写 Flash)。"""
    print(f"  Motor {motor_id:2d} ({name}) [泉智博]", end="", flush=True)
    if dry_run:
        print("  [DRY-RUN 跳过]")
        return True

    pos_before = rad2deg(evo.get_position(motor_id))
    ok = evo.set_zero_position(motor_id)
    time.sleep(0.1)
    if ok:
        print(f"  ✓ SetZero OK (Flash)  was={pos_before:+.1f}°")
    else:
        print("  ✗ SetZero 失败（电机离线？）")
    return ok


def set_zero_incos(incos, motor_id, name, dry_run):
    """因克斯 ENCOS §7.2 当前位置设零。"""
    print(f"  Motor {motor_id:2d} ({name}) [因克斯]", end="", flush=True)
    if dry_run:
        print("  [DRY-RUN 跳过]")
        return True

    pos_before = rad2deg(incos.get_position(motor_id))
    ok = incos.set_zero_position(motor_id)
    time.sleep(0.1)
    pos_after = rad2deg(incos.get_position(motor_id))
    if ok:
        print(f"  ✓ SetZero OK  ({pos_before:+.1f}° → {pos_after:+.1f}°)")
    else:
        print(f"  ✗ SetZero 失败/无ACK  ({pos_before:+.1f}° → {pos_after:+.1f}°)")
    return ok


def set_zero_dm(dm, motor_id, name, dry_run):
    """达妙: 控制帧 0xFE。无掉电记忆，仅当前上电有效。"""
    print(f"  Motor {motor_id:2d} ({name}) [达妙/无掉电记忆]", end="", flush=True)
    if dry_run:
        print("  [DRY-RUN 跳过]")
        return True

    pos_before = rad2deg(dm.get_position(motor_id))
    dm.enable(motor_id)
    time.sleep(0.05)
    dm.set_zero(motor_id)
    time.sleep(0.1)
    online, pos, _err, _link = dm.probe(motor_id)
    pos_after = rad2deg(pos)
    ok = online and abs(pos_after) < 15.0
    if ok:
        print(f"  ✓ set_zero OK  ({pos_before:+.1f}° → {pos_after:+.1f}°)  "
              f"— 掉电会丢，下次上电前仍需掰硬限位")
    else:
        print(f"  ✗ set_zero 失败  online={online} "
              f"({pos_before:+.1f}° → {pos_after:+.1f}°)")
    return ok


def _connected(j, lz, evo, incos, dm):
    mid = j.motor_id
    if j.mtype == "lz":
        return bool(lz.is_connected[mid - 1]), lz.get_position(mid)
    if j.mtype == "evo":
        return bool(evo.is_connected[mid - 1]), evo.get_position(mid)
    if j.mtype == "incos":
        if incos is None:
            return False, 0.0
        return bool(incos.is_connected[mid - 1]), incos.get_position(mid)
    if j.mtype == "dm":
        if dm is None:
            return False, 0.0
        online, pos, _err, _link = dm.probe(mid)
        return bool(online), pos
    return False, 0.0


def main():
    ap = argparse.ArgumentParser(description="设置全部电机零位（含因克斯）")
    ap.add_argument("--id", type=int, default=None, help="只设置单个电机 ID")
    ap.add_argument(
        "--ids", type=str, default=None,
        help="设置多个电机 ID，逗号分隔，如 2,3,6,7")
    ap.add_argument("--dry-run", action="store_true", help="演习，不实际执行")
    ap.add_argument(
        "--no-dm", action="store_true",
        help="跳过达妙 tarsus（默认会设，但掉电不记忆）")
    args = ap.parse_args()

    print("=" * 60)
    print("  Marsdog 全关节设零脚本（灵足/EVO/因克斯/达妙）")
    print("=" * 60)

    if args.dry_run:
        print("\n⚠️  DRY-RUN 模式：不会实际执行任何操作\n")

    lz = MotorLz()
    evo = MotorEvo()
    incos = MotorIncos()
    dm = None

    print(f"\n[init] 1/5 灵足 Serial ({LZ_SERIAL_DEVICE} — 后腿/头/腰)...")
    lz.init_serial(LZ_SERIAL_DEVICE, BAUD)

    print(f"[init] 2/5 灵足 CAN1   ({LZ_CAN1_DEVICE} — 前髋+head_roll)...")
    lz.init_can1_serial(LZ_CAN1_DEVICE, BAUD)

    print(f"[init] 3/5 泉智博 CAN0 ({EVO_CAN0_DEVICE} — 后腿hip+颈腰)...")
    evo.init_serial(EVO_CAN0_DEVICE, BAUD)

    print(f"[init] 4/5 因克斯 CAN  ({INCOS_CAN_DEVICE} — 前外展+小腿)...")
    if not incos.begin(INCOS_CAN_DEVICE, INCOS_CAN_IDS, BAUD):
        print("  [WARN] 因克斯总线无应答，将跳过 ID 2/3/6/7")
        try:
            incos.end()
        except Exception:
            pass
        incos = None

    if not args.no_dm:
        print(f"[init] 5/5 达妙 u2can ({DM_CAN_DEVICE} — 前 tarsus)...")
        dm = MotorDamiao()
        if dm.begin(DM_CAN_DEVICE, BAUD):
            time.sleep(0.8)
            from marsdog_control.config.joints import DM_CAN_IDS
            for mid in DM_CAN_IDS:
                dm.add_motor(mid, master_id=DM_MASTER_ID_BY_SLAVE.get(mid))
        else:
            print("  [WARN] 达妙适配器打开失败，将跳过 ID 4/8")
            dm = None
    else:
        print("[init] 5/5 达妙 跳过 (--no-dm)")

    # ── 筛选目标关节 ─────────────────────────────────────────────
    if args.ids is not None:
        target_ids = [int(x.strip()) for x in args.ids.split(",")]
        joints = []
        for mid in target_ids:
            j = JOINT_BY_ID.get(mid)
            if j is None:
                print(f"[ERROR] 未知 motor ID: {mid}")
                _shutdown(lz, evo, incos, dm)
                return
            joints.append(j)
    elif args.id is not None:
        j = JOINT_BY_ID.get(args.id)
        if j is None:
            print(f"[ERROR] 未知 motor ID: {args.id}")
            _shutdown(lz, evo, incos, dm)
            return
        joints = [j]
    else:
        joints = [
            j for j in JOINT_MAP
            if j.motor_id not in SKIP_IDS and j.bus != "none"
            and not (args.no_dm and j.mtype == "dm")
        ]
        skipped = [j.name for j in JOINT_MAP if j.motor_id in SKIP_IDS]
        print(f"[skip] 头/颈已排除: {skipped}")

    print("\n[pos] 等待反馈稳定 0.5s ...")
    time.sleep(0.5)

    print("\n[pos] 当前各关节位置（将被设为新零点）:")
    print(f"  {'ID':>3s}  {'名称':18s}  {'型号':16s}  {'品牌':6s}  {'当前位置':>10s}  {'在线'}")
    print("  " + "-" * 70)

    online_joints = []
    for j in joints:
        connected, pos = _connected(j, lz, evo, incos, dm)
        status = "✓" if connected else "✗ 离线"
        print(f"  {j.motor_id:3d}  {j.name:18s}  {j.model:16s}  {j.mtype:6s}  "
              f"{rad2deg(pos):8.2f}°   {status}")
        if connected:
            online_joints.append(j)

    if not online_joints:
        print("\n[ERROR] 没有在线电机，退出。")
        _shutdown(lz, evo, incos, dm)
        return

    offline = len(joints) - len(online_joints)
    print(f"\n  在线: {len(online_joints)}/{len(joints)}"
          + (f"，离线 {offline} 个将跳过" if offline else ""))

    if not args.dry_run:
        print("\n" + "!" * 60)
        print("  ⚠️   即将永久写入 Flash 零位偏置（达妙除外）！")
        print("  请确认机器人已摆到目标姿态（URDF 初始姿势）。")
        print("!" * 60)
        ans = input("\n  直接按回车确认执行，输入任意内容后回车取消: ")
        if ans != "":
            print("  已取消。")
            _shutdown(lz, evo, incos, dm)
            return

    print("\n[set_zero] 开始设零...\n")
    ok_count = 0
    for j in online_joints:
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
            print(f"  Motor {mid:2d} ({j.name}) [未知品牌 {j.mtype}] 跳过")
            ok = False
        if ok:
            ok_count += 1

    print(f"\n[done] 成功: {ok_count}/{len(online_joints)}")
    if ok_count == len(online_joints):
        print("  ✓ 全部目标电机零位设置完成！")
        print("  建议掉电重上后再跑 static_test / go_zero 确认读数接近 0°。")
        print("  达妙 tarsus 无掉电记忆：每次上电前仍需掰到硬限位。")
    else:
        fail = len(online_joints) - ok_count
        print(f"  ⚠ {fail} 个电机设置失败，请检查通信后重试。")

    _shutdown(lz, evo, incos, dm)


def _shutdown(lz, evo, incos, dm):
    try:
        lz.end()
    except Exception:
        pass
    try:
        evo.end()
    except Exception:
        pass
    if incos is not None:
        try:
            incos.end()
        except Exception:
            pass
    if dm is not None:
        try:
            dm.end()
        except Exception:
            pass


if __name__ == "__main__":
    main()
