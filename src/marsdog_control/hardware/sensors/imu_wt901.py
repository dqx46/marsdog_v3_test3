"""WT901 系列 IMU 驱动 (Python, 纯串口协议)

协议: WIT Motion 标准二进制, 每帧 11 字节
  [0x55] [type] [D0L D0H D1L D1H D2L D2H D3L D3H] [checksum]
  type=0x51 加速度, type=0x52 角速度, type=0x53 姿态角

支持 WT901G4K / WT901C / WT901B 等全系列.
"""

import time
import math
import struct
import threading

# [解耦] 真实实现已下沉到此 src 模块; 保持逐字一致的扁平 import(serial 本地垫片 +
# bus_config), 由 ensure_legacy_path() 把 mocap_to_real 置于 sys.path 前部保证解析。
from marsdog_control.compat import ensure_legacy_path as _ensure_legacy_path
_ensure_legacy_path()

import serial

from marsdog_control.config.bus_config import IMU_DEVICE, IMU_BAUD


def _ema_alpha(dt_s, tau_s):
    """按真实采样间隔计算一阶低通的新样本权重。"""
    if tau_s <= 0.0:
        return 1.0
    dt_s = max(1e-6, min(1.0, dt_s))
    return dt_s / (tau_s + dt_s)


class ImuWT901:
    """非阻塞读取 WT901 IMU, 独立线程解析."""

    def __init__(self, port=IMU_DEVICE, baud=IMU_BAUD,
                 angle_tau_s=0.025, gyro_tau_s=0.015):
        self.port = port
        self.baud = baud
        self._ser = None
        self._running = False
        self._thread = None
        self._lock = threading.Lock()

        # 原始数据 (SI 单位)
        self.acc = [0.0, 0.0, 0.0]       # m/s^2
        self.gyro = [0.0, 0.0, 0.0]      # rad/s
        self.angle = [0.0, 0.0, 0.0]     # rad (roll, pitch, yaw)

        # 滤波后输出
        self.roll = 0.0    # rad
        self.pitch = 0.0   # rad
        self.yaw = 0.0     # rad
        self.gyro_roll = 0.0   # rad/s
        self.gyro_pitch = 0.0  # rad/s
        self.raw_roll = 0.0
        self.raw_pitch = 0.0
        self.raw_gyro_roll = 0.0
        self.raw_gyro_pitch = 0.0

        # 校准偏置
        self._roll_offset = 0.0
        self._pitch_offset = 0.0

        # 按采样 dt 定义的低通时间常数；不再让滤波特性随 RRATE 改变。
        self.angle_tau_s = max(0.0, float(angle_tau_s))
        self.gyro_tau_s = max(0.0, float(gyro_tau_s))
        self._angle_filter_ready = False
        self._gyro_filter_ready = False

        # 统计
        self.update_count = 0
        self.acc_update_count = 0
        self.gyro_update_count = 0
        self.angle_update_count = 0
        self.acc_timestamp = 0.0
        self.gyro_timestamp = 0.0
        self.angle_timestamp = 0.0
        self.last_update_time = 0.0
        self._connected = False

    @property
    def connected(self):
        return self._connected

    def begin(self) -> bool:
        try:
            self._ser = serial.Serial(
                self.port, self.baud,
                timeout=0.002, write_timeout=0.1)
            self._ser.reset_input_buffer()
            self._running = True
            self._thread = threading.Thread(
                target=self._read_loop, daemon=True)
            self._thread.start()
            time.sleep(0.3)
            self._connected = self.update_count > 0
            return self._connected
        except Exception as e:
            print(f"[IMU] 无法打开 {self.port}: {e}")
            return False

    def calibrate(self, duration=1.0):
        """静态校准: 采集 duration 秒取平均作为零偏."""
        if not self._connected:
            return
        time.sleep(0.2)
        samples_r, samples_p = [], []
        t0 = time.monotonic()
        last_seq = -1
        while time.monotonic() - t0 < duration:
            with self._lock:
                seq = self.angle_update_count
                if seq != last_seq:
                    samples_r.append(self.angle[0])
                    samples_p.append(self.angle[1])
                    last_seq = seq
            time.sleep(0.002)
        if samples_r:
            self._roll_offset = sum(samples_r) / len(samples_r)
            self._pitch_offset = sum(samples_p) / len(samples_p)
            print(f"[IMU] 校准完成: roll_offset={math.degrees(self._roll_offset):.2f}° "
                  f"pitch_offset={math.degrees(self._pitch_offset):.2f}°")

    def close(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._ser and self._ser.is_open:
            self._ser.close()

    def _read_loop(self):
        buf = bytearray()
        while self._running:
            try:
                data = self._ser.read(128)
                if not data:
                    continue
                buf.extend(data)
                while len(buf) >= 11:
                    idx = buf.find(0x55)
                    if idx < 0:
                        buf.clear()
                        break
                    if idx > 0:
                        del buf[:idx]
                    if len(buf) < 11:
                        break
                    frame = bytes(buf[:11])
                    checksum = sum(frame[:10]) & 0xFF
                    if checksum != frame[10]:
                        del buf[:1]
                        continue
                    self._parse_frame(frame)
                    del buf[:11]
            except Exception:
                time.sleep(0.01)

    def _parse_frame(self, frame: bytes):
        ptype = frame[1]
        d = struct.unpack_from('<hhhh', frame, 2)
        now = time.monotonic()

        # IMU 原始轴 → 世界坐标系映射 (实测校正):
        #   world_roll  = -IMU_X   (左高右低为正)
        #   world_pitch = +IMU_Y   (抬头为正)
        #   world_yaw   = +IMU_Z   (左转为正)

        with self._lock:
            if ptype == 0x51:
                raw_x = d[0] / 32768.0 * 16.0 * 9.81
                raw_y = d[1] / 32768.0 * 16.0 * 9.81
                raw_z = d[2] / 32768.0 * 16.0 * 9.81
                self.acc[0] = -raw_x
                self.acc[1] = +raw_y
                self.acc[2] = +raw_z
                self.acc_timestamp = now
                self.acc_update_count += 1
            elif ptype == 0x52:
                raw_x = d[0] / 32768.0 * 2000.0 * math.pi / 180.0
                raw_y = d[1] / 32768.0 * 2000.0 * math.pi / 180.0
                raw_z = d[2] / 32768.0 * 2000.0 * math.pi / 180.0
                self.gyro[0] = -raw_x   # world roll rate
                self.gyro[1] = +raw_y   # world pitch rate
                self.gyro[2] = +raw_z   # world yaw rate
                self.raw_gyro_roll = self.gyro[0]
                self.raw_gyro_pitch = self.gyro[1]
                # Clip + dt-LPF: 先截断冲击尖峰，再按时间常数低通。
                _GYRO_CLIP = 5.0  # rad/s (真实99th%=1.4, 留3.6x余量)
                gr = max(-_GYRO_CLIP, min(_GYRO_CLIP, self.gyro[0]))
                gp = max(-_GYRO_CLIP, min(_GYRO_CLIP, self.gyro[1]))
                if self._gyro_filter_ready and self.gyro_timestamp > 0.0:
                    a = _ema_alpha(now - self.gyro_timestamp, self.gyro_tau_s)
                    self.gyro_roll += a * (gr - self.gyro_roll)
                    self.gyro_pitch += a * (gp - self.gyro_pitch)
                else:
                    self.gyro_roll, self.gyro_pitch = gr, gp
                    self._gyro_filter_ready = True
                self.gyro_timestamp = now
                self.gyro_update_count += 1
            elif ptype == 0x53:
                raw_x = d[0] / 32768.0 * math.pi  # IMU roll (deg→rad)
                raw_y = d[1] / 32768.0 * math.pi  # IMU pitch
                raw_z = d[2] / 32768.0 * math.pi  # IMU yaw
                self.angle[0] = -raw_x  # world roll
                self.angle[1] = +raw_y  # world pitch
                self.angle[2] = +raw_z  # world yaw
                raw_roll = self.angle[0] - self._roll_offset
                raw_pitch = self.angle[1] - self._pitch_offset
                self.raw_roll = raw_roll
                self.raw_pitch = raw_pitch
                if self._angle_filter_ready and self.angle_timestamp > 0.0:
                    a = _ema_alpha(now - self.angle_timestamp, self.angle_tau_s)
                    self.roll += a * (raw_roll - self.roll)
                    self.pitch += a * (raw_pitch - self.pitch)
                else:
                    self.roll, self.pitch = raw_roll, raw_pitch
                    self._angle_filter_ready = True
                self.yaw = self.angle[2]
                self.angle_timestamp = now
                self.angle_update_count += 1
                self.update_count += 1
                self.last_update_time = now
                self._connected = True

    def frame_ages(self, now=None):
        """返回 acc/gyro/angle 三类数据年龄（秒），从未收到则为 inf。"""
        now = time.monotonic() if now is None else now
        with self._lock:
            def age(ts):
                return max(0.0, now - ts) if ts > 0.0 else float("inf")
            return {
                "acc": age(self.acc_timestamp),
                "gyro": age(self.gyro_timestamp),
                "angle": age(self.angle_timestamp),
            }


if __name__ == "__main__":
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else IMU_DEVICE
    print(f"测试 IMU @ {port}")
    imu = ImuWT901(port, IMU_BAUD)
    if not imu.begin():
        print("IMU 未连接或无数据")
        sys.exit(1)
    print("等待数据...")
    time.sleep(0.5)
    print(f"收到 {imu.update_count} 帧")
    imu.calibrate(1.0)
    print("\n实时数据 (Ctrl+C 退出):")
    try:
        while True:
            print(f"\r  Roll={math.degrees(imu.roll):+6.2f}°  "
                  f"Pitch={math.degrees(imu.pitch):+6.2f}°  "
                  f"Yaw={math.degrees(imu.yaw):+6.1f}°  "
                  f"GyroR={math.degrees(imu.gyro_roll):+6.1f}°/s  "
                  f"cnt={imu.update_count}", end="   ")
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    imu.close()
    print("\n完成")
