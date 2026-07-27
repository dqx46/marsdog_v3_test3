#!/bin/bash
# 修复串口权限，并安装 marsdog 固定设备名
# 用法: bash fix_usb_ports.sh

set -e
cd "$(dirname "$0")"
RULES_SRC="udev/99-marsdog-usb.rules"
RULES_DST="/etc/udev/rules.d/99-marsdog-usb.rules"

echo "==> 当前 USB 串口:"
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-path/ 2>/dev/null || true
echo
ls -l /dev/input/by-id/*joystick 2>/dev/null || true
echo

echo "==> 将 $USER 加入 dialout 组"
sudo usermod -aG dialout "$USER"

echo "==> 安装已核对的 udev 规则 -> $RULES_DST"
sudo cp "$RULES_SRC" "$RULES_DST"
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty
# 逐个 trigger，确保符号链接生成
for tty in /sys/class/tty/ttyUSB* /sys/class/tty/ttyACM*; do
  [[ -e "$tty" ]] && sudo udevadm trigger --action=add "$tty" || true
done
sleep 0.8

echo "==> 立即放宽当前串口权限 (本会话可直接测)"
sudo chmod 666 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true

echo "==> /dev/marsdog_* 别名:"
ls -l /dev/marsdog_* 2>/dev/null || echo "  尚未出现，请重新插拔 USB 后再 ls /dev/marsdog_*"

echo
echo "==> bus_config 解析:"
python3 -c "
from bus_config import *
import os
for n,p in [('LZ_A',LZ_CAN_A_DEVICE),('LZ_B',LZ_CAN_B_DEVICE),('EVO',EVO_CAN_DEVICE),
            ('DM',DM_CAN_DEVICE),('IMU',IMU_DEVICE),('PAD',GAMEPAD_DEVICE)]:
    print(f'  {n:4s} {p} -> {os.path.realpath(p) if os.path.exists(p) else \"MISSING\"}')
"

echo
echo "完成。"
echo "  - 若刚加入 dialout：注销重登后永久生效"
echo "  - 机器人上电后建议再跑: sudo python3 setup_usb_devices.py --install"
echo "    （用电机应答确认三路 CH340 角色；若没换物理插口可跳过）"
echo "  - 快速自检: python3 static_test.py"
