"""USB-CAN serial bridge (AT frame protocol, SJA1000 encoding).

200Hz 优化版：
- 预分配 TX 缓冲区，避免每帧 bytearray()
- send_bulk()：多帧一次性 write()，减少系统调用
- read_msg() 纯非阻塞（O_NONBLOCK），0 延迟
- drain_all()：一次 read() 拿尽所有到达字节
"""

import os
import time
import termios
import select

CAN_EFF_FLAG = 0x80000000
DIFF_EXTEND_S = 0x7FF

_FRAME_HEADER = (ord('A'), ord('T'))


class CanSerial:

    def __init__(self):
        self._fd = -1
        self._rxbuf = bytearray(4096)   # 更大缓冲，减少 read() 次数
        self._rxpos = 0
        self._txbuf = bytearray(20)     # 预分配，避免每帧 bytearray()
        self._bulk_buf = bytearray(0)   # send_bulk 动态拼接

    # ------------------------------------------------------------------

    @staticmethod
    def _to_speed(baud: int):
        _MAP = {
            9600: termios.B9600, 19200: termios.B19200,
            38400: termios.B38400, 57600: termios.B57600,
            115200: termios.B115200, 230400: termios.B230400,
            460800: termios.B460800, 921600: termios.B921600,
        }
        return _MAP.get(baud, termios.B115200)

    def _open_serial(self, device: str, baud: int) -> bool:
        try:
            fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            print(f"[CanSerial] open {device}: {e}")
            return False
        attrs = termios.tcgetattr(fd)
        sp = self._to_speed(baud)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = sp
        attrs[5] = sp
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        self._fd = fd
        return True

    def _enter_at_mode(self) -> bool:
        if self._fd < 0:
            return False

        def try_cmd(cmd: bytes) -> bool:
            termios.tcflush(self._fd, termios.TCIOFLUSH)
            os.write(self._fd, cmd)
            buf = b""
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.3:
                r, _, _ = select.select([self._fd], [], [], 0.01)
                if r:
                    try:
                        chunk = os.read(self._fd, 32)
                        if chunk:
                            buf += chunk
                        if b"OK" in buf:
                            return True
                    except BlockingIOError:
                        pass
            return False

        if try_cmd(b"AT+AT\r\n"):
            return True
        if try_cmd(b"AT+CG\r\n") and try_cmd(b"AT+AT\r\n"):
            return True
        return False

    def begin(self, device: str, baud: int = 921600) -> bool:
        if not self._open_serial(device, baud):
            return False
        if self._enter_at_mode():
            print(f"[CanSerial] {device} @{baud}, AT mode OK")
        else:
            print(f"[CanSerial] {device} @{baud}, open OK (assume AT mode)")
        return True

    def end(self):
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1

    # ------------------------------------------------------------------
    # 发送：单帧 & 批量

    def _build_frame(self, buf: bytearray, data: bytes, dlc: int, can_id: int) -> int:
        """将一个 CAN 帧编码到 buf[0:total]，返回总字节数。buf 至少 20 字节。"""
        buf[0] = 0x41  # 'A'
        buf[1] = 0x54  # 'T'

        if can_id > DIFF_EXTEND_S:
            cid = can_id & 0x1FFFFFFF
            buf[2] = (cid >> 21) & 0xFF
            buf[3] = (cid >> 13) & 0xFF
            buf[4] = (cid >> 5) & 0xFF
            buf[5] = ((cid & 0x1F) << 3) | 0x04
        else:
            cid = can_id & 0x7FF
            buf[2] = (cid >> 3) & 0xFF
            buf[3] = (cid & 0x07) << 5
            buf[4] = 0x00
            buf[5] = 0x00

        buf[6] = dlc
        for i in range(dlc):
            buf[7 + i] = data[i] if i < len(data) else 0
        buf[7 + dlc] = 0x0D   # '\r'
        buf[7 + dlc + 1] = 0x0A  # '\n'
        return 9 + dlc

    def send_msg(self, data: bytes, dlc: int, can_id: int) -> bool:
        """发送单帧（重用预分配 _txbuf）。"""
        if self._fd < 0:
            return False
        n = self._build_frame(self._txbuf, data, dlc, can_id)
        try:
            return os.write(self._fd, self._txbuf[:n]) == n
        except OSError:
            return False

    def send_bulk(self, frames) -> bool:
        """批量发送多帧，frames = [(data, dlc, can_id), ...]，一次 write()。
        
        减少系统调用次数，在多电机批量下发时比逐帧 send_msg 快 2-4×。
        """
        if self._fd < 0:
            return False
        # 每帧最大 20 字节，预估总大小
        total_est = len(frames) * 20
        if len(self._bulk_buf) < total_est:
            self._bulk_buf = bytearray(total_est)

        pos = 0
        tmp = bytearray(20)
        for data, dlc, can_id in frames:
            n = self._build_frame(tmp, data, dlc, can_id)
            self._bulk_buf[pos:pos + n] = tmp[:n]
            pos += n
        written_total = 0
        try:
            while written_total < pos:
                w = os.write(self._fd, self._bulk_buf[written_total:pos])
                if w == 0:
                    break
                written_total += w
            return written_total == pos
        except OSError:
            return False

    # ------------------------------------------------------------------
    # 接收：纯非阻塞

    def _parse_frame(self):
        """从内部缓冲区解析一帧，返回 (can_id, dlc, data) 或 None。"""
        buf = self._rxbuf
        pos = self._rxpos

        # 找帧头 'AT'
        start = -1
        for i in range(pos - 1):
            if buf[i] == 0x41 and buf[i + 1] == 0x54:
                start = i
                break
        if start < 0:
            self._rxpos = 0
            return None
        if start > 0:
            remaining = pos - start
            self._rxbuf[:remaining] = self._rxbuf[start:pos]
            self._rxpos = remaining
            pos = remaining

        if pos < 7:
            return None
        dlc = buf[6]
        if dlc > 8:
            self._rxpos = 0
            return None
        need = 7 + dlc + 2
        if pos < need:
            return None

        is_ext = bool(buf[5] & 0x04)
        if is_ext:
            can_id = CAN_EFF_FLAG | (
                (buf[2] << 21) | (buf[3] << 13) | (buf[4] << 5) | (buf[5] >> 3)
            )
        else:
            can_id = (buf[2] << 3) | (buf[3] >> 5)

        data = bytes(buf[7:7 + dlc])

        remaining = pos - need
        self._rxbuf[:remaining] = self._rxbuf[need:pos]
        self._rxpos = remaining
        return (can_id, dlc, data)

    def _fill_rxbuf(self):
        """一次性读取所有可用字节到 rxbuf（非阻塞）。返回读到的字节数。"""
        if self._fd < 0:
            return 0
        space = len(self._rxbuf) - self._rxpos
        if space <= 0:
            # 缓冲区满，丢弃最旧的一半
            half = len(self._rxbuf) // 2
            self._rxbuf[:half] = self._rxbuf[half:half * 2]
            self._rxpos = half
            space = half
        try:
            chunk = os.read(self._fd, space)
            if chunk:
                n = len(chunk)
                self._rxbuf[self._rxpos:self._rxpos + n] = chunk
                self._rxpos += n
                return n
        except OSError:
            pass
        return 0

    def read_msg(self):
        """非阻塞：先解析缓冲区，不够再读一次，返回 (can_id, dlc, data) 或 None。"""
        if self._fd < 0:
            return None
        result = self._parse_frame()
        if result:
            return result
        self._fill_rxbuf()
        return self._parse_frame()

    def read_msg_blocking(self, timeout: float = 0.005):
        """阻塞读，最多等 timeout 秒，用于初始化时的请求-应答。"""
        if self._fd < 0:
            return None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            r = self.read_msg()
            if r is not None:
                return r
            # 有数据等待则立刻重试，否则短暂让出 CPU
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            r2, _, _ = select.select([self._fd], [], [], min(0.001, remaining))
            if r2:
                self._fill_rxbuf()
        return None

    def drain_all(self):
        """读取并丢弃当前 OS 缓冲区所有字节，并清空 rxbuf（用于 flush）。"""
        if self._fd < 0:
            return
        try:
            while True:
                chunk = os.read(self._fd, 4096)
                if not chunk:
                    break
        except OSError:
            pass
        self._rxpos = 0

    def flush(self):
        """清空接收缓冲区。"""
        if self._fd < 0:
            return
        termios.tcflush(self._fd, termios.TCIFLUSH)
        self._rxpos = 0
