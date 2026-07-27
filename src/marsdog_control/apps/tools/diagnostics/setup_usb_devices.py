#!/usr/bin/env python3
"""扫描 USB 转 CAN / IMU，生成 udev 规则，创建固定设备名。

问题: CH340 没有唯一序列号，插拔后 /dev/ttyUSB* 编号会变 (重启/重新插拔顺序
不同都会导致编号漂移)。
方案:
  - 达妙 u2can 适配器 USB VID:PID 独有 (2e88:4603)，直接按此匹配，
    与插口/插拔顺序完全无关。
  - 其余 CH340 适配器 (灵足 CAN-A / CAN-B、泉智博 EVO、因克斯独立) 外观完全相同、
    无唯一序列号，通过在总线上发指令探测各自专属的电机 ID 来区分角色，
    再绑定到 USB Hub 物理口 (ID_PATH) 生成固定别名:
      /dev/marsdog_lz_can_a
      /dev/marsdog_lz_can_b
      /dev/marsdog_incos_can
      /dev/marsdog_evo_can
      /dev/marsdog_dm_can
      /dev/marsdog_imu

用法:
  # 1. 五路 USB-CAN + IMU 全部接好、机器人上电
  python3 setup_usb_devices.py

  # 2. 确认识别结果无误后安装规则 (需要 sudo)
  sudo python3 setup_usb_devices.py --install

  # 3. 重新插拔 / 重启后验证 (只要没换物理插口就一直有效)
  ls -l /dev/marsdog_*
  python3 static_test.py

  # 4. 如果换了物理插口 (不是换编号，是真的换了USB口位置)，重新执行第 1-2 步
  #    重新生成绑定即可，无需改任何代码。
"""

import argparse
import getpass
import grp
import json
import os
import shutil
import subprocess
import sys
import time
from marsdog_control.apps.tools.diagnostics.usb_probe import (
    ROLE_NAMES,
    SYMLINK_BY_ROLE,
    render_udev_rules,
    scan_all_devices,
)

RULES_PATH = "/etc/udev/rules.d/99-marsdog-usb.rules"
# Deploy artifacts stay under mocap_to_real/ (bus_config reads the same map file).
from marsdog_control.compat import legacy_dir as _legacy_dir
_DEPLOY = str(_legacy_dir())
LOCAL_RULES = os.path.join(_DEPLOY, "udev", "99-marsdog-usb.rules")
DEVICE_MAP_FILE = os.path.join(_DEPLOY, "usb_device_map.json")


def warn_if_no_tty_permission():
    if os.geteuid() == 0:
        return
    user = getpass.getuser()
    groups = {g.gr_name for g in grp.getgrall() if user in g.gr_mem}
    gid_groups = {grp.getgrgid(gid).gr_name for gid in os.getgroups()}
    groups.update(gid_groups)
    if "dialout" in groups:
        return
    print("提示: 当前用户不在 dialout 组，可能无法打开 /dev/ttyUSB*。")
    print("      如看到 Permission denied，请执行:")
    print(f"        sudo usermod -aG dialout {user}")
    print("      然后注销/重启后再运行本脚本。\n")


def save_device_map(found):
    mapping = {}
    for role in SYMLINK_BY_ROLE:
        info = found.get(role)
        if not info or not info.get("dev"):
            continue
        # 优先存 by-path，避免 ttyUSB 编号漂移；换物理插口后重新运行本脚本。
        id_path = info.get("id_path") or ""
        if id_path:
            # ID_PATH 对 ttyUSB 通常对应 by-path 的 "...-port0"
            candidates = [
                f"/dev/serial/by-path/{id_path}-port0",
                f"/dev/serial/by-path/{id_path}",
            ]
            mapping[role] = next((p for p in candidates if os.path.exists(p)), info["dev"])
        else:
            mapping[role] = info["dev"]
    with open(DEVICE_MAP_FILE, "w", encoding="utf-8") as f:
        json.dump(mapping, f, indent=2, ensure_ascii=False)
        f.write("\n")
    print(f"[save] 设备映射缓存: {DEVICE_MAP_FILE}")
    return mapping


def print_scan_results(found, unknown):
    print("=" * 68)
    print("  Marsdog USB 设备识别结果")
    print("=" * 68)

    for role, label in ROLE_NAMES.items():
        info = found.get(role)
        symlink = f"/dev/{SYMLINK_BY_ROLE[role]}"
        if info:
            print(f"\n  ✓ {label}")
            print(f"    当前设备: {info['dev']}")
            print(f"    固定别名: {symlink}")
            print(f"    ID_PATH : {info.get('id_path', '(无)')}")
            print(f"    USB     : {info.get('id_vendor')}:{info.get('id_product')}")
            scores = info.get("scores")
            if scores:
                print(f"    探测得分: {scores}")
            fixed_from = info.get("imu_baud_fixed_from")
            if fixed_from:
                print(f"    ⚠ 检测到波特率跑回 {fixed_from}，已自动改回并保存为 115200")
        else:
            print(f"\n  ✗ {label}  — 未识别")

    if unknown:
        print("\n  ? 未识别设备:")
        for info in unknown:
            extra = ""
            if info.get("duplicate_of"):
                extra = f" (与 {info['duplicate_of']} 角色冲突)"
            print(f"    {info['dev']}  role={info.get('role')}  "
                  f"ID_PATH={info.get('id_path', '?')}{extra}")

    print()


def install_rules(rules_text, found=None):
    os.makedirs(os.path.dirname(LOCAL_RULES), exist_ok=True)
    with open(LOCAL_RULES, "w", encoding="utf-8") as f:
        f.write(rules_text)

    print(f"[save] 本地副本: {LOCAL_RULES}")

    with open(RULES_PATH, "w", encoding="utf-8") as f:
        f.write(rules_text)
    print(f"[save] 系统规则: {RULES_PATH}")

    subprocess.run(["udevadm", "control", "--reload-rules"], check=True)
    # 批量 trigger 有时对已存在的设备节点不生效 (udev 认为无变化而跳过)，
    # 这里额外逐个设备单独 trigger 一次，确保符号链接一定生成。
    subprocess.run(
        ["udevadm", "trigger", "--action=add", "--subsystem-match=tty"], check=True
    )
    if found:
        for info in found.values():
            dev = info.get("dev")
            if dev:
                tty_name = os.path.basename(dev)
                sys_path = f"/sys/class/tty/{tty_name}"
                if os.path.exists(sys_path):
                    subprocess.run(
                        ["udevadm", "trigger", "--action=add", sys_path],
                        check=False,
                    )
    time.sleep(0.5)
    print("[done] udev 规则已重载")

    print("\n验证符号链接:")
    for role, name in SYMLINK_BY_ROLE.items():
        path = f"/dev/{name}"
        if os.path.exists(path):
            print(f"  {path} -> {os.path.realpath(path)}")
        else:
            print(f"  {path}  (尚未出现，请重新插拔 USB 或重启)")


def main():
    parser = argparse.ArgumentParser(description="识别并固定 Marsdog USB-CAN 设备名")
    parser.add_argument(
        "--install",
        action="store_true",
        help="将识别结果写入 /etc/udev/rules.d/ 并重载 udev (需要 root)",
    )
    args = parser.parse_args()

    if args.install and os.geteuid() != 0:
        print("安装 udev 规则需要 root 权限，请使用:")
        print("  sudo python3 setup_usb_devices.py --install")
        return 1

    warn_if_no_tty_permission()

    print("正在扫描串口并探测总线特征电机/IMU，请稍候...\n")
    found, unknown = scan_all_devices()
    print_scan_results(found, unknown)

    required = ("evo_can", "lz_can_a", "lz_can_b", "incos_can", "dm_can", "imu")
    missing = [r for r in required if r not in found]
    if missing:
        print("警告: 以下必需总线未识别:", ", ".join(missing))
        print("请确认机器人已上电、USB 线已接好，然后重试。")
        if not args.install:
            return 1

    rules_text = render_udev_rules(found)
    save_device_map(found)
    print("将生成的 udev 规则:")
    print("-" * 68)
    print(rules_text)
    print("-" * 68)

    if not args.install:
        print("\n下一步: 若识别结果正确，执行")
        print("  sudo python3 setup_usb_devices.py --install")
        return 0 if not missing else 1

    install_rules(rules_text, found)
    print("\n完成! bus_config.py 已配置为使用 /dev/marsdog_* 固定路径。")
    print("以后只要各模块插在相同 USB Hub 物理口，插拔顺序不再影响程序。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
