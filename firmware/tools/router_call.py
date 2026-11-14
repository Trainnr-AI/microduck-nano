#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Call one of the firmware's Bridge functions from a shell ON THE BOARD.

A debugging aid with no dependencies (the board's system Python has no msgpack
module): it speaks MessagePack-RPC to the arduino-router socket directly.

    adb push tools/router_call.py /home/arduino/router_call.py
    adb shell python3 /home/arduino/router_call.py '["duck_info"]' '["duck_status"]' '["duck_ping", 1]'

Each argument is a JSON list: the method name, then its parameters. The reply
is printed as [1, id, error, result]. "method ... not available" means the
sketch has not registered it: it is not running, or has not finished setup().
"""
import json
import socket
import struct
import sys

SOCKET = "/var/run/arduino-router.sock"


def pack(o):
    if o is None:
        return b"\xc0"
    if o is True:
        return b"\xc3"
    if o is False:
        return b"\xc2"
    if isinstance(o, int):
        if 0 <= o < 128:
            return bytes([o])
        if -32 <= o < 0:
            return struct.pack("b", o)
        if 0 <= o < 2 ** 32:
            return b"\xce" + struct.pack(">I", o)
        return b"\xd3" + struct.pack(">q", o)
    if isinstance(o, float):
        return b"\xca" + struct.pack(">f", o)
    if isinstance(o, str):
        b = o.encode()
        return (bytes([0xA0 | len(b)]) if len(b) < 32 else b"\xd9" + bytes([len(b)])) + b
    if isinstance(o, (list, tuple)):
        head = bytes([0x90 | len(o)]) if len(o) < 16 else b"\xdc" + struct.pack(">H", len(o))
        return head + b"".join(pack(x) for x in o)
    raise TypeError(f"cannot pack {o!r}")


_NUM = {0xCC: (">B", 1), 0xCD: (">H", 2), 0xCE: (">I", 4), 0xCF: (">Q", 8), 0xD0: (">b", 1), 0xD1: (">h", 2),
        0xD2: (">i", 4), 0xD3: (">q", 8), 0xCA: (">f", 4), 0xCB: (">d", 8)}


def unpack(b, i=0):
    t = b[i]
    if t < 0x80:
        return t, i + 1
    if t >= 0xE0:
        return t - 256, i + 1
    if 0x90 <= t <= 0x9F or t in (0xDC, 0xDD):
        if t <= 0x9F:
            n, i = t & 15, i + 1
        elif t == 0xDC:
            n, i = struct.unpack(">H", b[i + 1:i + 3])[0], i + 3
        else:
            n, i = struct.unpack(">I", b[i + 1:i + 5])[0], i + 5
        out = []
        for _ in range(n):
            v, i = unpack(b, i)
            out.append(v)
        return out, i
    if 0xA0 <= t <= 0xBF:
        n = t & 31
        return b[i + 1:i + 1 + n].decode(errors="replace"), i + 1 + n
    if t == 0xD9:
        n = b[i + 1]
        return b[i + 2:i + 2 + n].decode(errors="replace"), i + 2 + n
    if t == 0xC0:
        return None, i + 1
    if t == 0xC2:
        return False, i + 1
    if t == 0xC3:
        return True, i + 1
    fmt, n = _NUM[t]
    return struct.unpack(fmt, b[i + 1:i + 1 + n])[0], i + 1 + n


def call(method, *args, timeout=3.0):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        s.connect(SOCKET)
        s.sendall(pack([0, 1, method, list(args)]))
        buf = b""
        while True:
            chunk = s.recv(4096)
            if not chunk:
                return "connection closed"
            buf += chunk
            try:
                return unpack(buf)[0]
            except (IndexError, struct.error, KeyError):
                continue  # reply not complete yet


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for spec in sys.argv[1:]:
        parts = json.loads(spec)
        try:
            print(parts[0], "->", call(*parts))
        except Exception as e:  # noqa: BLE001 - a debugging tool: show whatever went wrong
            print(parts[0], "-> failed:", repr(e))
