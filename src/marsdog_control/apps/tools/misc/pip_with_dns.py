#!/usr/bin/env python3
"""pip install using UDP DNS (bypasses broken systemd-resolved stub).

Use when `host pypi.org` fails with SERVFAIL but ping 8.8.8.8 works.

Example:
    python3 pip_with_dns.py install --user matplotlib \\
        -i https://pypi.tuna.tsinghua.edu.cn/simple \\
        --trusted-host pypi.tuna.tsinghua.edu.cn
"""

from __future__ import annotations

import random
import socket
import struct
import sys

DNS_SERVERS = ["192.168.5.1", "8.8.8.8", "114.114.114.114"]
CACHE: dict[str, str] = {}


def dns_a(name: str, server: str, timeout: float = 3.0) -> str | None:
    tid = random.randint(0, 65535)
    labels = b"".join(bytes([len(p)]) + p.encode() for p in name.split(".")) + b"\x00"
    req = struct.pack(">HHHHHH", tid, 0x0100, 1, 0, 0, 0) + labels + struct.pack(">HH", 1, 1)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.sendto(req, (server, 53))
        data, _ = s.recvfrom(512)
    except OSError:
        return None
    finally:
        s.close()
    if len(data) < 12:
        return None
    flags, _qd, an = struct.unpack(">HHH", data[2:8])
    if (flags & 0xF) != 0 or an == 0:
        return None
    i = 12
    while i < len(data) and data[i] != 0:
        i += 1 + data[i]
    i += 5
    for _ in range(an):
        if i >= len(data):
            break
        if data[i] & 0xC0 == 0xC0:
            i += 2
        else:
            while i < len(data) and data[i] != 0:
                i += 1 + data[i]
            i += 1
        if i + 10 > len(data):
            break
        rtype, _rclass, _ttl, rdlen = struct.unpack(">HHIH", data[i:i + 10])
        i += 10
        if rtype == 1 and rdlen == 4 and i + 4 <= len(data):
            return socket.inet_ntoa(data[i:i + 4])
        i += rdlen
    return None


def resolve(host: str) -> str:
    if host in CACHE:
        return CACHE[host]
    try:
        socket.inet_aton(host)
        CACHE[host] = host
        return host
    except OSError:
        pass
    for srv in DNS_SERVERS:
        ip = dns_a(host, srv)
        if ip:
            CACHE[host] = ip
            print(f"[dns] {host} -> {ip} via {srv}", flush=True)
            return ip
    raise OSError(f"DNS failed for {host}")


_orig_getaddrinfo = socket.getaddrinfo


def _patched_getaddrinfo(host, port, family=0, type=0, proto=0, flags=0):
    host_s = host.decode() if isinstance(host, bytes) else host
    try:
        ip = resolve(host_s)
    except OSError:
        return _orig_getaddrinfo(host, port, family, type, proto, flags)
    return _orig_getaddrinfo(ip, port, family, type, proto, flags)


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    socket.getaddrinfo = _patched_getaddrinfo
    from pip._internal.cli.main import main as pip_main
    return pip_main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
