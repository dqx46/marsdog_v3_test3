"""SocketCAN wrapper for can0 / can1 interfaces.

Architecture follows the proven marsdog_firmware reference:
- Separate TX raw socket (non-blocking send, large SO_SNDBUF)
- Separate RX raw socket (short SO_RCVTIMEO for recv thread)
"""

import socket
import struct
import os

CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF

CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

DIFF_EXTEND = 0x7FF


class CanBus:

    def __init__(self):
        self._tx_sock = None
        self._rx_sock = None

    def begin(self, interface: str, baud: int = 1000000) -> bool:
        os.system(f"ip link set down {interface} 2>/dev/null")
        ret = os.system(f"ip link set up {interface} type can bitrate {baud}")
        if ret != 0:
            print(f"[CanBus] Failed to set up {interface}")
            return False
        return self._open_sockets(interface)

    def _open_sockets(self, interface: str) -> bool:
        SOL_CAN_RAW = 101
        CAN_RAW_LOOPBACK = 3
        try:
            tx = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            tx.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
            tx.setsockopt(SOL_CAN_RAW, CAN_RAW_LOOPBACK, struct.pack("i", 0))
            tx.bind((interface,))
            self._tx_sock = tx

            rx = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            tv = struct.pack("ll", 0, 5000)  # 5ms recv timeout
            rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO, tv)
            rx.bind((interface,))
            self._rx_sock = rx
            return True
        except OSError as e:
            print(f"[CanBus] socket error on {interface}: {e}")
            return False

    def end(self):
        if self._tx_sock:
            self._tx_sock.close()
            self._tx_sock = None
        if self._rx_sock:
            self._rx_sock.close()
            self._rx_sock = None

    def send_msg_rx(self, data: bytes, dlc: int, can_id: int) -> bool:
        """Send via the RX socket (for init-time send-and-recv on same socket)."""
        if self._rx_sock is None:
            return False
        frame_id = can_id
        if can_id > DIFF_EXTEND:
            frame_id = can_id | CAN_EFF_FLAG
        padded = (data + b'\x00' * 8)[:8]
        frame = struct.pack(CAN_FRAME_FMT, frame_id, dlc, padded)
        try:
            self._rx_sock.send(frame)
            return True
        except OSError:
            return False

    def send_msg(self, data: bytes, dlc: int, can_id: int) -> bool:
        if self._tx_sock is None:
            return False
        frame_id = can_id
        if can_id > DIFF_EXTEND:
            frame_id = can_id | CAN_EFF_FLAG
        padded = (data + b'\x00' * 8)[:8]
        frame = struct.pack(CAN_FRAME_FMT, frame_id, dlc, padded)
        try:
            self._tx_sock.send(frame)
            return True
        except OSError:
            return False

    def read_msg(self):
        """Returns (can_id, dlc, data_bytes) or None on timeout."""
        if self._rx_sock is None:
            return None
        try:
            raw = self._rx_sock.recv(CAN_FRAME_SIZE)
            if len(raw) < CAN_FRAME_SIZE:
                return None
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
            return (can_id, dlc, data[:dlc])
        except (OSError, BlockingIOError):
            return None

    def read_msg_nonblock(self):
        """Non-blocking read: returns (can_id, dlc, data) or None immediately."""
        if self._rx_sock is None:
            return None
        try:
            self._rx_sock.setblocking(False)
            raw = self._rx_sock.recv(CAN_FRAME_SIZE)
            if len(raw) < CAN_FRAME_SIZE:
                return None
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
            return (can_id, dlc, data[:dlc])
        except (OSError, BlockingIOError):
            return None
        finally:
            self._rx_sock.setblocking(True)
            tv = struct.pack("ll", 0, 5000)
            self._rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO, tv)

    def flush(self):
        if self._rx_sock is None:
            return
        self._rx_sock.setblocking(False)
        try:
            while True:
                try:
                    self._rx_sock.recv(CAN_FRAME_SIZE)
                except (OSError, BlockingIOError):
                    break
        finally:
            self._rx_sock.setblocking(True)
            tv = struct.pack("ll", 0, 5000)
            self._rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO, tv)
