"""Small pyserial-compatible wrapper used when python3-serial is unavailable.

Only the subset used by this project is implemented: Serial(...), read(),
write(), flush(), reset_input_buffer(), close(), is_open, and in_waiting.
"""

import array
import fcntl
import os
import select
import termios
import time


class SerialException(OSError):
    pass


class Serial:
    _BAUD_MAP = {
        4800: termios.B4800,
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
        460800: termios.B460800,
        921600: termios.B921600,
    }

    def __init__(self, port, baudrate=9600, timeout=None, write_timeout=None, **_kwargs):
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = timeout
        self.write_timeout = write_timeout
        self._fd = -1

        try:
            self._fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            self._configure()
        except OSError as exc:
            self._fd = -1
            raise SerialException(str(exc)) from exc

    def _configure(self):
        attrs = termios.tcgetattr(self._fd)
        speed = self._BAUD_MAP.get(self.baudrate)
        if speed is None:
            raise SerialException(f"unsupported baudrate: {self.baudrate}")

        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = speed
        attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
        termios.tcflush(self._fd, termios.TCIOFLUSH)

    @property
    def is_open(self):
        return self._fd >= 0

    @property
    def in_waiting(self):
        if self._fd < 0:
            return 0
        buf = array.array("i", [0])
        try:
            fcntl.ioctl(self._fd, termios.FIONREAD, buf, True)
            return int(buf[0])
        except OSError:
            return 0

    def read(self, size=1):
        if self._fd < 0:
            raise SerialException("port is closed")
        size = max(0, int(size))
        if size == 0:
            return b""

        chunks = bytearray()
        deadline = None if self.timeout is None else time.monotonic() + float(self.timeout)
        while len(chunks) < size:
            wait = None
            if deadline is not None:
                wait = max(0.0, deadline - time.monotonic())
                if wait == 0.0 and not chunks:
                    break
                if wait == 0.0:
                    break
            r, _, _ = select.select([self._fd], [], [], wait)
            if not r:
                break
            try:
                chunk = os.read(self._fd, size - len(chunks))
            except BlockingIOError:
                continue
            except OSError as exc:
                raise SerialException(str(exc)) from exc
            if not chunk:
                break
            chunks.extend(chunk)
        return bytes(chunks)

    def write(self, data):
        if self._fd < 0:
            raise SerialException("port is closed")
        view = memoryview(bytes(data))
        written = 0
        deadline = None
        if self.write_timeout is not None:
            deadline = time.monotonic() + float(self.write_timeout)
        while written < len(view):
            wait = None
            if deadline is not None:
                wait = max(0.0, deadline - time.monotonic())
                if wait == 0.0:
                    break
            _, w, _ = select.select([], [self._fd], [], wait)
            if not w:
                break
            try:
                n = os.write(self._fd, view[written:])
            except BlockingIOError:
                continue
            except OSError as exc:
                raise SerialException(str(exc)) from exc
            if n <= 0:
                break
            written += n
        return written

    def flush(self):
        if self._fd >= 0:
            termios.tcdrain(self._fd)

    def reset_input_buffer(self):
        if self._fd >= 0:
            termios.tcflush(self._fd, termios.TCIFLUSH)

    def reset_output_buffer(self):
        if self._fd >= 0:
            termios.tcflush(self._fd, termios.TCOFLUSH)

    def close(self):
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _tb):
        self.close()
        return False
