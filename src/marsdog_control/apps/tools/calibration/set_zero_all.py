#!/usr/bin/env python3
"""为所有电机设置当前位置为零点（永久写入 Flash）。

操作说明:
  - 灵足 (RS02/EL05/RS00): 发 SetOrigin(0x06) → SaveParams(0x16 写Flash)
  - 泉智博 PA43:            发 SetZeroPosition(0xFE，自动写Flash)

⚠️  警告: 此操作永久修改电机 Flash 中的零位偏置！
    请确保机器人已摆放到目标零位姿态（URDF 初始站立姿势）后再执行。

用法:
  python set_zero_all.py            # 交互确认后设置全部电机
  python set_zero_all.py --id 7     # 只设置单个电机
  python set_zero_all.py --dry-run  # 演习：只打印不执行
"""

import sys, os, time, math, argparse
from marsdog_control.config.joints import JOINT_MAP, JOINT_BY_ID
from marsdog_control.hardware.motors.lingzu import MotorLz
from marsdog_control.hardware.motors.evo import MotorEvo

# 头部和脖子电机不参与设零（机械结构特殊，单独标定）
SKIP_IDS = {13, 14, 15, 16}  # head_pitch, head_yaw, head_roll, neck_pitch

# 老固件 RS00 电机：set_origin 无效，需用 add_offset(0x702B) 写偏置，重启后生效
OLD_FW_IDS = {9, 12}  # rl_calf, rr_calf

def rad2deg(r): return r * 180.0 / math.pi


def set_zero_lz(lz, motor_id, name, dry_run):
    """灵足: Enable → kp=0稳定 → SetOrigin → SaveParams → 验证。
    老固件电机(OLD_FW_IDS)用 add_offset 写偏置模式，重启后生效。"""
    is_old_fw = motor_id in OLD_FW_IDS
    mode_str = "偏置模式(老固件)" if is_old_fw else "标准模式"
    print(f"  Motor {motor_id:2d} ({name}) [灵足/{mode_str}]", end="", flush=True)
    if dry_run:
        print("  [DRY-RUN 跳过]")
        return True

    pos_before = math.degrees(lz.position[motor_id - 1])

    # 使能 → 等 mode=2
    lz.enable(motor_id)
    time.sleep(0.15)

    # 发若干 kp=0 帧让电机稳在当前位置
    cur_rad = lz.position[motor_id - 1]
    for _ in range(15):
        lz.mit_control(motor_id, cur_rad, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.003)
    time.sleep(0.05)

    if is_old_fw:
        # 老固件：写 add_offset 参数，重启后生效
        lz.set_zero_via_offset(motor_id, cur_rad)
        time.sleep(0.15)
        lz.disable(motor_id)
        print(f"  ✓ add_offset 写入完成 (pos={pos_before:+.1f}°，重启电机后生效)")
        return True
    else:
        # 新固件：set_origin 立即生效
        lz.set_origin(motor_id)
        time.sleep(0.08)
        lz.save_params(motor_id)
        time.sleep(0.15)

        # 发 kp=0 等反馈更新
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
            print(f"  ✗ 验证失败  ({pos_before:+.1f}° → {pos_after:+.1f}°)  "
                  f"— 可能是老固件，改用 --offset 模式")
        return ok


def set_zero_evo(evo, motor_id, name, dry_run):
    """泉智博: SetZeroPosition (自动写Flash)"""
    print(f"  Motor {motor_id:2d} ({name}) [泉智博]", end="", flush=True)
    if dry_run:
        print("  [DRY-RUN 跳过]")
        return True

    ok = evo.set_zero_position(motor_id)
    time.sleep(0.1)

    if ok:
        print("  ✓ SetZero OK (Flash 已写入)")
    else:
        print("  ✗ SetZero 失败（电机离线？）")
    return ok


def main():
    ap = argparse.ArgumentParser(description="设置全部电机零位")
    ap.add_argument("--id",      type=int, default=None, help="只设置单个电机 ID")
    ap.add_argument("--ids",     type=str, default=None, help="设置多个电机 ID，逗号分隔，如 13,14,15,16")
    ap.add_argument("--dry-run", action="store_true",    help="演习，不实际执行")
    args = ap.parse_args()

    print("=" * 60)
    print("  Marsdog 全关节设零脚本")
    print("=" * 60)

    if args.dry_run:
        print("\n⚠️  DRY-RUN 模式：不会实际执行任何操作\n")

    # ── 初始化总线（全部 USB-CAN 串口模式）─────────────────────────
    from marsdog_control.config.bus_config import LZ_CAN1_DEVICE, EVO_CAN0_DEVICE, LZ_SERIAL_DEVICE, BAUD
    lz  = MotorLz()
    evo = MotorEvo()

    print(f"\n[init] 1/3 灵足 Serial ({LZ_SERIAL_DEVICE} — 后腿/头/腰)...")
    lz.init_serial(LZ_SERIAL_DEVICE, BAUD)

    print(f"[init] 2/3 灵足 CAN1   ({LZ_CAN1_DEVICE} — 前腿+head_roll)...")
    lz.init_can1_serial(LZ_CAN1_DEVICE, BAUD)

    print(f"[init] 3/3 泉智博 CAN0 ({EVO_CAN0_DEVICE} — 后腿hip+颈腰)...")
    evo.init_serial(EVO_CAN0_DEVICE, BAUD)

    # ── 筛选目标关节 ─────────────────────────────────────────────
    if args.ids is not None:
        target_ids = [int(x.strip()) for x in args.ids.split(",")]
        joints = []
        for mid in target_ids:
            j = JOINT_BY_ID.get(mid)
            if j is None:
                print(f"[ERROR] 未知 motor ID: {mid}")
                lz.end(); evo.end(); return
            joints.append(j)
    elif args.id is not None:
        j = JOINT_BY_ID.get(args.id)
        if j is None:
            print(f"[ERROR] 未知 motor ID: {args.id}")
            lz.end(); evo.end(); return
        joints = [j]
    else:
        joints = [j for j in JOINT_MAP if j.motor_id not in SKIP_IDS]
        print(f"[skip] 头部/脖子电机已排除: "
              f"{[j.name for j in JOINT_MAP if j.motor_id in SKIP_IDS]}")

    # ── 打印当前位置 ─────────────────────────────────────────────
    print("\n[pos] 等待反馈稳定 0.5s ...")
    time.sleep(0.5)

    print("\n[pos] 当前各关节位置（将被设为新零点）:")
    print(f"  {'ID':>3s}  {'名称':18s}  {'型号':5s}  {'总线':6s}  {'当前位置':>10s}  {'在线'}")
    print("  " + "-" * 58)

    online_joints = []
    for j in joints:
        mid = j.motor_id
        if j.mtype == "lz":
            connected = lz.is_connected[mid - 1]
            pos = lz.get_position(mid) if connected else 0.0
        else:
            connected = evo.is_connected[mid - 1]
            pos = evo.get_position(mid) if connected else 0.0

        status = "✓" if connected else "✗ 离线"
        print(f"  {mid:3d}  {j.name:18s}  {j.model:5s}  {j.bus:6s}  "
              f"{rad2deg(pos):8.2f}°   {status}")
        if connected:
            online_joints.append(j)

    if not online_joints:
        print("\n[ERROR] 没有在线电机，退出。")
        lz.end(); evo.end(); return

    offline = len(joints) - len(online_joints)
    print(f"\n  在线: {len(online_joints)}/{len(joints)}"
          + (f"，离线 {offline} 个将跳过" if offline else ""))

    # ── 确认 ─────────────────────────────────────────────────────
    if not args.dry_run:
        print("\n" + "!" * 60)
        print("  ⚠️   即将永久写入 Flash 零位偏置！")
        print("  请确认机器人已摆到目标姿态（URDF 初始姿势）。")
        print("!" * 60)
        ans = input("\n  直接按回车确认执行，输入任意内容后回车取消: ")
        if ans != "":
            print("  已取消。")
            lz.end(); evo.end(); return

    # ── 逐电机设零 ───────────────────────────────────────────────
    print("\n[set_zero] 开始设零...\n")
    ok_count = 0
    for j in online_joints:
        mid = j.motor_id
        if j.mtype == "lz":
            ok = set_zero_lz(lz, mid, j.name, args.dry_run)
        else:
            ok = set_zero_evo(evo, mid, j.name, args.dry_run)
        if ok:
            ok_count += 1

    # ── 结果 ─────────────────────────────────────────────────────
    print(f"\n[done] 成功: {ok_count}/{len(online_joints)}")
    if ok_count == len(online_joints):
        print("  ✓ 全部电机零位设置完成！")
        print("  重新上电后新零位生效，建议再次运行 static_test.py 确认位置读数接近 0°。")
    else:
        fail = len(online_joints) - ok_count
        print(f"  ⚠ {fail} 个电机设置失败，请检查通信后重试。")

    lz.end()
    evo.end()


if __name__ == "__main__":
    main()
