#!/usr/bin/env python3
"""将达妙 S2325 (前腿 tarsus, ID 4/8) 从当前控制模式切换为 MIT 模式并保存到 flash.

用法:
    python3 dm_switch_mode.py            # 切换 DM_CAN_IDS (4,8) 到 MIT_MODE
    python3 dm_switch_mode.py --id 4     # 只切换单个电机
    python3 dm_switch_mode.py --read-only  # 只读当前 CTRL_MODE, 不修改

流程 (对照达妙官方例程 damiao.h::switchControlMode / save_motor_param):
  1. disable 电机 (0xFD)
  2. 读取当前 CTRL_MODE (RID=10) 寄存器值并打印
  3. 写入 MIT_MODE(=1) 并校验
  4. save_motor_param() 保存到 flash (电机内部会先 disable, 然后等待写入完成)
  5. 再次读取 CTRL_MODE 确认

注意: 两个电机的实测 MasterID 并不相同 (见 joint_config.DM_MASTER_ID_BY_SLAVE),
本脚本按每个电机各自的 MasterID 分别匹配反馈帧, 且操作严格串行 (一次只对一个
slave_id 发指令并等回复)。
"""
import argparse
import sys
import os
import time
from marsdog_control.hardware.motors.damiao import (MotorDamiao, MIT_MODE, REG_CTRL_MODE,
                           _CTRL_MODE_NAMES)
from marsdog_control.config.bus_config import DM_CAN_DEVICE, BAUD
from marsdog_control.config.joints import DM_CAN_IDS, DM_MASTER_ID_BY_SLAVE, JOINT_BY_ID


def mode_name(v):
    if v is None:
        return "未知(读取超时)"
    return f"{v} ({_CTRL_MODE_NAMES.get(v, '未知模式')})"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, action="append", dest="ids",
                     help="只操作指定的 slave_id (可多次指定), 默认操作全部 DM_CAN_IDS")
    ap.add_argument("--read-only", action="store_true",
                     help="只读取当前 CTRL_MODE, 不做任何修改")
    ap.add_argument("--mode", type=int, default=MIT_MODE,
                     help=f"目标模式值, 默认 {MIT_MODE} (MIT_MODE)")
    args = ap.parse_args()

    ids = args.ids or list(DM_CAN_IDS)

    dm = MotorDamiao()
    print(f"打开 {DM_CAN_DEVICE} @ {BAUD} ...")
    if not dm.begin(DM_CAN_DEVICE, BAUD):
        print(f"✗ 打开 {DM_CAN_DEVICE} 失败, 检查设备路径/权限")
        sys.exit(1)
    time.sleep(1.5)  # u2can CDC-ACM 初始化

    for sid in ids:
        dm.add_motor(sid, master_id=DM_MASTER_ID_BY_SLAVE.get(sid))

    ok_all = True
    for sid in ids:
        j = JOINT_BY_ID.get(sid)
        name = j.name if j else "?"
        mst = DM_MASTER_ID_BY_SLAVE.get(sid)
        mst_str = f"0x{mst:02X}" if mst is not None else "未知(默认slave+0x10)"
        print(f"\n── Motor {sid} ({name}) MasterID={mst_str} ──")

        dm.disable(sid)
        time.sleep(0.05)

        cur = dm.get_control_mode(sid)
        print(f"  当前 CTRL_MODE = {mode_name(cur)}")

        if args.read_only:
            continue

        if cur == args.mode:
            print(f"  已经是目标模式 {mode_name(args.mode)}, 跳过写入")
        else:
            print(f"  写入 CTRL_MODE = {mode_name(args.mode)} ...")
            ok = dm.switch_control_mode(sid, args.mode)
            if ok:
                print("  ✓ 写入并校验成功")
            else:
                print("  ✗ 写入/校验失败! (电机可能未接线或链路异常)")
                ok_all = False
                continue

        print("  保存参数到 flash (save_motor_param) ...")
        dm.save_motor_param(sid)
        time.sleep(0.2)

        final = dm.get_control_mode(sid)
        print(f"  保存后回读 CTRL_MODE = {mode_name(final)}")
        if final != args.mode:
            print("  ✗ 保存后回读值与目标不一致, 请检查!")
            ok_all = False
        else:
            print("  ✓ 已持久化为 MIT_MODE")

    dm.end()

    if args.read_only:
        print("\n(只读模式, 未做任何修改)")
    elif ok_all:
        print("\n全部电机已切换到 MIT 模式并保存成功。")
        print("下一步: 重新给电机上电, 再运行 static_test.py / probe 确认仍在线,")
        print("然后即可用 control_mit(slave_id, kp, kd, q, dq, tau) 控制。")
    else:
        sys.exit(2)


if __name__ == "__main__":
    main()
