#!/usr/bin/env python3
"""纯静态通信测试 — 不使能电机、不发运动指令。

真源：``marsdog_control.apps.tools.diagnostics.static_test``（Phase J）。
兼容入口：``mocap_to_real/static_test.py``（薄启动器）。

五条 USB-CAN 串口总线拓扑:
  总线 A  (灵足 CAN-A):     灵足 ID 1,2,5,6,17
  总线 A2 (因克斯独立):     因克斯 ID 3,7
  总线 B  (灵足 CAN-B):     ID 10,11,13,14,15,16,21 — 后腿从关节 + 头部 + waist_roll
  总线 C  (泉智博 EVO):     ID 9,12,18,19,20        — 后腿 hip + 颈腰
  总线 D  (达妙 u2can):     ID 4,8                  — 前腿 tarsus (S2325)

测试内容:
  1. 逐总线打开串口
  2. 灵足(CAN-A, CAN-B): 用 ReadParam(mechPos 0x7019) 探测
  3. 因克斯(独立总线): 用 E1 01 位置查询探测
  4. 泉智博(EVO): 发 RestState(0xFD) 探测
  5. 达妙(u2can): 发 refresh_motor_status 探测
  6. 被动监听 IMU 数据帧
  7. 检查 PS2 手柄接收器和喇叭设备路径
  8. 汇总在线电机及外设状态
  9. 固定 2D 俯视示意图标红离线电机 (可用 --no-plot 关闭)
"""

import argparse
import os
import time
import struct

from marsdog_control.hardware.motors.can_serial import CanSerial, CAN_EFF_FLAG
from marsdog_control.hardware.motors.damiao import MotorDamiao
from marsdog_control.config.bus_config import (
    LZ_CAN_A_DEVICE, LZ_CAN_B_DEVICE, INCOS_CAN_DEVICE,
    EVO_CAN_DEVICE, DM_CAN_DEVICE, BAUD,
    IMU_DEVICE, IMU_BAUD, GAMEPAD_DEVICE,
    SPEAKER_ALSA_DEVICE,
)
from marsdog_control.config.joints import (
    JOINT_BY_ID, LZ_CAN_A_IDS, LZ_CAN_B_IDS,
    EVO_CAN_IDS, DM_CAN_IDS, INCOS_CAN_IDS,
    DM_MASTER_ID_BY_SLAVE,
)
from marsdog_control.apps.tools.diagnostics.motor_status_viz import (
    default_plot_path,
    render_motor_status_figure,
)

# ── 常量 ──────────────────────────────────────────────────────────
P_MIN, P_MAX = -12.57, 12.57

CMD_READ_PARAM = 0x11
IDX_MECH_POS = 0x7019

CMD_REST_STATE = 0xFD
MEVO_THETA_MIN, MEVO_THETA_MAX = -12.5, 12.5
INCOS_QUERY_POSITION = bytes([0xE1, 0x01])


def rad2deg(r):
    return r * 180.0 / 3.14159265


def uint_to_float(v, x_min, x_max, bits):
    maxv = (1 << bits) - 1
    return v / maxv * (x_max - x_min) + x_min


# ── WT901 IMU 探测 ─────────────────────────────────────────────────

def probe_imu(device, baud, read_s=0.5):
    """纯被动监听 IMU 数据帧 (0x55 帧头), 不发任何指令。"""
    try:
        import serial
    except ImportError:
        # Fallback: legacy usb_probe (still in mocap_to_real) when pyserial absent.
        try:
            from marsdog_control.compat import ensure_legacy_path
            ensure_legacy_path()
            from usb_probe import probe_imu as _ok
        except ImportError:
            return False, "未安装 python3-serial 且无 usb_probe 回退", {}
        if not _ok(device, timeout=max(0.5, read_s), baud=baud):
            return False, "未读到有效 WT901 帧 (且未安装 python3-serial)", {}
        return True, "在线 (termios 探测, 无姿态解析; 请 apt install python3-serial)", {}

    try:
        s = serial.Serial(device, baud, timeout=read_s)
    except Exception as e:
        return False, str(e), {}

    time.sleep(0.1)
    s.reset_input_buffer()
    data = s.read(300)
    s.close()

    if len(data) < 11:
        return False, f"仅收到 {len(data)} 字节 (太少)", {}

    header_count = data.count(0x55)
    if header_count < 2:
        return False, f"收到 {len(data)} 字节但无有效 0x55 帧头 (波特率可能不对)", {}

    # 解析找到的帧: 55 <type> <8 bytes data> <checksum>
    frames = {}
    i = 0
    while i < len(data) - 10:
        if data[i] == 0x55:
            ftype = data[i + 1]
            payload = data[i + 2:i + 10]
            frames[ftype] = payload
            i += 11
        else:
            i += 1

    type_names = {0x50: "time", 0x51: "accel", 0x52: "gyro",
                  0x53: "angle", 0x54: "mag", 0x59: "quaternion"}
    parsed = {}
    if 0x53 in frames:
        p = frames[0x53]
        roll = struct.unpack_from("<h", p, 0)[0] / 32768.0 * 180.0
        pitch = struct.unpack_from("<h", p, 2)[0] / 32768.0 * 180.0
        yaw = struct.unpack_from("<h", p, 4)[0] / 32768.0 * 180.0
        parsed["roll"], parsed["pitch"], parsed["yaw"] = roll, pitch, yaw

    frame_types = [type_names.get(t, f"0x{t:02X}") for t in frames.keys()]
    return True, f"帧类型: {frame_types}", parsed


# ── 灵足 RS05 探测: ReadParam(mechPos) ────────────────────────────

def probe_rs05_serial(serial, motor_id):
    """通过串口发送 ReadParam(0x7019)，不改变电机状态。"""
    frame_id = (CMD_READ_PARAM << 24) | (0x00 << 16) | (0xFD << 8) | motor_id
    buf = bytearray(8)
    struct.pack_into("<H", buf, 0, IDX_MECH_POS)

    serial.flush()
    if not serial.send_msg(bytes(buf), 8, frame_id):
        return False, 0.0

    deadline = time.monotonic() + 0.050
    while time.monotonic() < deadline:
        result = serial.read_msg()
        if result is None:
            time.sleep(0.001)
            continue
        can_id, dlc, data = result
        if not (can_id & CAN_EFF_FLAG):
            continue
        resp_id = (can_id >> 8) & 0xFF
        if resp_id == motor_id and len(data) >= 8:
            pos_rad = struct.unpack_from("<f", data, 4)[0]
            return True, pos_rad
    return False, 0.0


def probe_incos(serial, motor_id):
    """在因克斯独立 USB-CAN 上发 E1 01 位置查询，不发送运动控制。"""
    serial.flush()
    if not serial.send_msg(INCOS_QUERY_POSITION, len(INCOS_QUERY_POSITION), motor_id):
        return False, 0.0, ""

    deadline = time.monotonic() + 0.050
    while time.monotonic() < deadline:
        result = serial.read_msg()
        if result is None:
            time.sleep(0.001)
            continue
        can_id, dlc, data = result
        if can_id & CAN_EFF_FLAG:
            continue
        if (can_id & 0x7FF) != motor_id or len(data) < 2:
            continue
        frame_type = data[0] >> 5
        fault = data[0] & 0x1F
        if frame_type == 5 and len(data) >= 6 and data[1] == 1:
            deg = struct.unpack(">f", bytes(data[2:6]))[0]
            return True, deg * 3.14159265 / 180.0, f"fault={fault}"
        if frame_type == 1 and len(data) == 8:
            pos_raw = (data[1] << 8) | data[2]
            pos = uint_to_float(pos_raw, -12.5, 12.5, 16)
            return True, pos, f"fault={fault}"
    return False, 0.0, ""


# ── 泉智博 MotorEvo 探测: RestState(0xFD) ─────────────────────────

def probe_evo(serial, motor_id):
    """通过串口发送 RestState 命令探测泉智博电机。不会使能电机。"""
    data = bytes([0xFF] * 7 + [CMD_REST_STATE])

    serial.flush()
    if not serial.send_msg(data, 8, motor_id):
        return False, 0.0, "", 0, 0

    deadline = time.monotonic() + 0.050
    while time.monotonic() < deadline:
        result = serial.read_msg()
        if result is None:
            time.sleep(0.001)
            continue
        can_id, dlc, rdata = result
        if can_id & CAN_EFF_FLAG:
            continue
        if len(rdata) < 8:
            continue
        if rdata[0] > 0x05:
            continue
        if (can_id & 0x7FF) == motor_id:
            pos_raw = (rdata[1] << 8) | rdata[2]
            pos_rad = uint_to_float(pos_raw, MEVO_THETA_MIN, MEVO_THETA_MAX, 16)
            fault = rdata[6]
            temp = rdata[7]
            status_names = {0: "Rest", 1: "Servo", 2: "PTM",
                            3: "Velocity", 4: "Torque", 5: "Torque4"}
            status_str = status_names.get(rdata[0], f"0x{rdata[0]:02X}")
            return True, pos_rad, status_str, fault, temp
    return False, 0.0, "", 0, 0


# ── 主测试 ────────────────────────────────────────────────────────

def _parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Marsdog 五总线 + IMU 静态通信测试（不使能电机）")
    p.add_argument(
        "--no-plot", action="store_true",
        help="不生成电机状态 2D 示意图")
    p.add_argument(
        "--show", action="store_true",
        help="出图后尝试弹出窗口（无显示环境则仅保存）")
    p.add_argument(
        "--plot-path", default=None,
        help="示意图 PNG 路径（默认仓库根 static_test_status.png）")
    return p.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    print("=" * 70)
    print("  Marsdog 五总线 + IMU 静态通信测试")
    print("  ※ 不使能电机、不发运动指令")
    print("=" * 70)

    results = {}

    # ── 总线 A: 灵足 CAN-A (前腿髋/大腿 + head_roll) ───────────────
    print(f"\n[1/6] {LZ_CAN_A_DEVICE} → CAN-A (灵足前腿髋/大腿/head_roll)")
    print(f"      预期灵足: {LZ_CAN_A_IDS}")
    can_a = CanSerial()
    if can_a.begin(LZ_CAN_A_DEVICE, BAUD):
        print(f"      串口已打开 @ {BAUD}")
        time.sleep(0.1)
        print("      -- 灵足 LZ --")
        for mid in LZ_CAN_A_IDS:
            j = JOINT_BY_ID.get(mid)
            name = j.name if j else "?"
            model = j.model if j else "?"
            ok, pos = probe_rs05_serial(can_a, mid)
            if ok:
                deg = rad2deg(pos)
                print(f"      ✓ Motor {mid:2d} ({name:18s}) [{model:8s}]  ONLINE  "
                      f"pos = {deg:8.2f}°  ({pos:7.4f} rad)")
                results[mid] = (True, "lz_can_a", deg, "")
            else:
                print(f"      ✗ Motor {mid:2d} ({name:18s}) [{model:8s}]  OFFLINE")
                results[mid] = (False, "lz_can_a", 0, "")
            time.sleep(0.010)
    else:
        print(f"      ✗ {LZ_CAN_A_DEVICE} 打开失败!")
        for mid in LZ_CAN_A_IDS:
            results[mid] = (False, "lz_can_a", 0, "bus_fail")

    # ── 总线 A2: 因克斯独立 USB-CAN (前腿小腿) ─────────────────────
    print(f"\n[2/6] {INCOS_CAN_DEVICE} → 因克斯独立 CAN (前腿小腿)")
    print(f"      预期因克斯: {INCOS_CAN_IDS}")
    can_incos = CanSerial()
    if can_incos.begin(INCOS_CAN_DEVICE, BAUD):
        print(f"      串口已打开 @ {BAUD}")
        time.sleep(0.1)
        print("      -- 因克斯 ENCOS --")
        for mid in INCOS_CAN_IDS:
            j = JOINT_BY_ID.get(mid)
            name = j.name if j else "?"
            model = j.model if j else "?"
            ok, pos, extra = probe_incos(can_incos, mid)
            if ok:
                deg = rad2deg(pos)
                print(f"      ✓ Motor {mid:2d} ({name:18s}) [{model:8s}]  ONLINE  "
                      f"pos = {deg:8.2f}°  ({pos:7.4f} rad)  {extra}")
                results[mid] = (True, "incos_can", deg, extra)
            else:
                print(f"      ✗ Motor {mid:2d} ({name:18s}) [{model:8s}]  OFFLINE")
                results[mid] = (False, "incos_can", 0, "")
            time.sleep(0.010)
    else:
        print(f"      ✗ {INCOS_CAN_DEVICE} 打开失败!")
        for mid in INCOS_CAN_IDS:
            results[mid] = (False, "incos_can", 0, "bus_fail")

    # ── 总线 B: 灵足 CAN-B (后腿从关节 + 头部 + waist_roll) ────────
    print(f"\n[3/6] {LZ_CAN_B_DEVICE} → 灵足 CAN-B (后腿+头/腰)")
    print(f"      预期电机: {LZ_CAN_B_IDS}")
    can_b = CanSerial()
    if can_b.begin(LZ_CAN_B_DEVICE, BAUD):
        print(f"      串口已打开 @ {BAUD}")
        time.sleep(0.1)
        for mid in LZ_CAN_B_IDS:
            j = JOINT_BY_ID.get(mid)
            name = j.name if j else "?"
            model = j.model if j else "?"
            ok, pos = probe_rs05_serial(can_b, mid)
            if ok:
                deg = rad2deg(pos)
                print(f"      ✓ Motor {mid:2d} ({name:18s}) [{model:8s}]  ONLINE  "
                      f"pos = {deg:8.2f}°  ({pos:7.4f} rad)")
                results[mid] = (True, "lz_can_b", deg, "")
            else:
                print(f"      ✗ Motor {mid:2d} ({name:18s}) [{model:8s}]  OFFLINE")
                results[mid] = (False, "lz_can_b", 0, "")
            time.sleep(0.010)
    else:
        print(f"      ✗ {LZ_CAN_B_DEVICE} 打开失败!")
        for mid in LZ_CAN_B_IDS:
            results[mid] = (False, "lz_can_b", 0, "bus_fail")

    # ── 总线 C: 泉智博 EVO (后腿hip + 颈腰) ───────────────────────
    print(f"\n[4/6] {EVO_CAN_DEVICE} → 泉智博 MotorEvo")
    print(f"      预期电机: {EVO_CAN_IDS}")
    can_evo = CanSerial()
    if can_evo.begin(EVO_CAN_DEVICE, BAUD):
        print(f"      串口已打开 @ {BAUD}")
        time.sleep(0.1)
        for mid in EVO_CAN_IDS:
            j = JOINT_BY_ID.get(mid)
            name = j.name if j else "?"
            model = j.model if j else "?"
            ret = probe_evo(can_evo, mid)
            if ret[0]:
                ok, pos, status, fault, temp = ret
                deg = rad2deg(pos)
                extra = f"status={status}  fault={fault}  temp={temp}°C"
                print(f"      ✓ Motor {mid:2d} ({name:18s}) [{model:8s}]  ONLINE  "
                      f"pos = {deg:8.2f}°  {extra}")
                results[mid] = (True, "evo_can", deg, extra)
            else:
                print(f"      ✗ Motor {mid:2d} ({name:18s}) [{model:8s}]  OFFLINE")
                results[mid] = (False, "evo_can", 0, "")
            time.sleep(0.010)
    else:
        print(f"      ✗ {EVO_CAN_DEVICE} 打开失败!")
        for mid in EVO_CAN_IDS:
            results[mid] = (False, "evo_can", 0, "bus_fail")

    # ── 总线 D: 达妙 u2can (前腿 tarsus S2325) ────────────────────
    print(f"\n[5/6] {DM_CAN_DEVICE} → 达妙 S2325 (前腿 tarsus)")
    print(f"      预期电机: {DM_CAN_IDS}")
    dm = MotorDamiao()
    if dm.begin(DM_CAN_DEVICE, BAUD):
        print(f"      串口已打开 @ {BAUD}, 等待适配器初始化...")
        time.sleep(1.5)  # u2can (CDC-ACM) 需要较长初始化时间
        for mid in DM_CAN_IDS:
            # 每个电机的实测 MasterID 不同, 见 joint_config.DM_MASTER_ID_BY_SLAVE
            dm.add_motor(mid, master_id=DM_MASTER_ID_BY_SLAVE.get(mid))
        for mid in DM_CAN_IDS:
            j = JOINT_BY_ID.get(mid)
            name = j.name if j else "?"
            model = j.model if j else "?"
            online, pos, err, link_ok = dm.probe(mid)
            if online:
                deg = rad2deg(pos)
                extra = f"err={err}" if err else ""
                print(f"      ✓ Motor {mid:2d} ({name:18s}) [{model:8s}]  ONLINE  "
                      f"pos = {deg:8.2f}°  ({pos:7.4f} rad)  {extra}")
                results[mid] = (True, "dm_can", deg, extra)
            elif link_ok:
                print(f"      ✗ Motor {mid:2d} ({name:18s}) [{model:8s}]  OFFLINE "
                      f"(适配器链路正常, 电机未上电/未接线)")
                results[mid] = (False, "dm_can", 0, "link_ok_no_motor")
            else:
                print(f"      ✗ Motor {mid:2d} ({name:18s}) [{model:8s}]  OFFLINE "
                      f"(适配器无响应!)")
                results[mid] = (False, "dm_can", 0, "link_dead")
            time.sleep(0.010)
    else:
        print(f"      ✗ {DM_CAN_DEVICE} 打开失败!")
        for mid in DM_CAN_IDS:
            results[mid] = (False, "dm_can", 0, "bus_fail")

    # ── IMU: WT901G4K ─────────────────────────────────────────────
    print(f"\n[6/6] {IMU_DEVICE} → WT901G4K IMU (被动监听, 不发指令)")
    imu_ok, imu_msg, imu_data = probe_imu(IMU_DEVICE, IMU_BAUD)
    if imu_ok:
        print(f"      ✓ IMU ONLINE  {imu_msg}")
        if imu_data:
            print(f"      当前姿态: roll={imu_data.get('roll', 0):+7.2f}°  "
                  f"pitch={imu_data.get('pitch', 0):+7.2f}°  "
                  f"yaw={imu_data.get('yaw', 0):+7.2f}°")
    else:
        print(f"      ✗ IMU OFFLINE  ({imu_msg})")

    # ── 其他外设路径检查 ─────────────────────────────────────────
    print(f"\n[附加] PS2 手柄接收器: {GAMEPAD_DEVICE}")
    print(f"      {'✓ 设备节点存在' if os.path.exists(GAMEPAD_DEVICE) else '✗ 设备节点不存在'}")
    print(f"[附加] 喇叭 ALSA 设备: {SPEAKER_ALSA_DEVICE}")
    print("      播放测试请运行: python3 speaker_test.py")

    # ── 汇总 ─────────────────────────────────────────────────────
    print("\n" + "=" * 70)
    print("  汇总")
    print("=" * 70)
    print(f"  {'ID':>3s}  {'名称':18s}  {'型号':8s}  {'总线':10s}  {'状态':7s}  {'位置':>10s}")
    print("  " + "-" * 66)

    online_count = 0
    total = len(results)
    for mid in sorted(results.keys()):
        j = JOINT_BY_ID.get(mid)
        name = j.name if j else "?"
        model = j.model if j else "?"
        on, bus, deg, extra = results[mid]
        status = "ONLINE" if on else "OFFLINE"
        pos_str = f"{deg:8.2f}°" if on else "   ---"
        print(f"  {mid:3d}  {name:18s}  {model:8s}  {bus:10s}  {status:7s}  {pos_str}")
        if on:
            online_count += 1

    print("  " + "-" * 66)
    print(f"  电机在线: {online_count}/{total}")
    print(f"  IMU:      {'ONLINE' if imu_ok else 'OFFLINE'}")
    print(f"  手柄:     {'ONLINE' if os.path.exists(GAMEPAD_DEVICE) else 'OFFLINE'}")
    print(f"  喇叭:     {SPEAKER_ALSA_DEVICE}")
    if online_count == total and imu_ok:
        print("  ✓ 全部电机 + IMU 通信正常!")
    else:
        if online_count < total:
            offline = [mid for mid, v in results.items() if not v[0]]
            offline_named = [f"{mid}({JOINT_BY_ID[mid].name})" for mid in offline if mid in JOINT_BY_ID]
            print(f"  ✗ 离线电机: {offline_named}")
        if not imu_ok:
            print(f"  ✗ IMU 离线: {imu_msg}")
    print()

    if not args.no_plot:
        out = args.plot_path or default_plot_path()
        path = render_motor_status_figure(
            results, out_path=out, show=args.show)
        if path:
            print(f"  电机状态示意图: {path}")
            if offline_ids := [mid for mid, v in results.items() if not v[0]]:
                print(f"  (红点 = 离线: {offline_ids})")
            print()

    # cleanup
    can_a.end()
    can_incos.end()
    can_b.end()
    can_evo.end()
    dm.end()


if __name__ == "__main__":
    raise SystemExit(main() or 0)
