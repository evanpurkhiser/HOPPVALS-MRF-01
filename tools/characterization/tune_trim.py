#!/usr/bin/env python3
"""Auto-tune the per-motor feedforward trim (motion.ff_trim_r) so the two lift
motors stay synced in closed loop.

The two motors aren't identical — one runs a few percent faster than the other
at the same duty. With identical feedforward, the PI + sync loop has to make up
that constant offset, which shows up as one side hunting behind the other (it
falls behind, the loop speeds it up, it falls behind again...). Trimming the
faster motor's feedforward removes the constant offset so the loop only handles
small disturbances.

This script drives a closed-loop move while recording both encoders, measures
the steady-state L-R count divergence, and runs a secant search on ff_trim_r
until that divergence is ~zero. ff_trim_l is held at 1.0 as the reference; tuning
ff_trim_r over a range that spans 1.0 corrects whichever motor is faster (R fast
-> trim < 1.0, L fast -> trim > 1.0).

Per device: each blind has its own motor pair, so run this once per unit. The
result is saved to that device's NVS (config save). The firmware default is
1.0/1.0.

Prerequisites:
  - Device in WiFi debug mode (flip the HA "Debug Mode" switch) and reachable.
  - Homed + mm_per_rev calibrated is NOT required (this only needs the closed-
    loop speed control, not position), but a move that won't hit an endpoint is:
    by default it lowers, which has room from the top.

Pure standard library so the part that drives a ceiling-mounted blind can't fail
on a missing package.

Usage:
  python3 tune_trim.py --ip 10.0.0.135
  python3 tune_trim.py --ip 10.0.0.135 --rpm 60 --target 6
"""

import argparse
import base64
import os
import socket
import statistics
import struct
import sys
import time


# ── Minimal RFC6455 websocket client (text frames, one cmd in / one reply) ──
class Console:
    def __init__(self, ip, timeout=45):
        self.s = socket.create_connection((ip, 80), timeout=timeout)
        self.s.settimeout(timeout)
        key = base64.b64encode(os.urandom(16)).decode()
        self.s.sendall(
            f"GET /ws HTTP/1.1\r\nHost: {ip}\r\nUpgrade: websocket\r\n"
            f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
            f"Sec-WebSocket-Version: 13\r\n\r\n".encode()
        )
        resp = self.s.recv(1024).decode(errors="replace")
        if "101" not in resp.split("\r\n", 1)[0]:
            raise RuntimeError(f"websocket handshake failed: {resp!r}")

    def _send(self, msg):
        data = msg.encode()
        mask = os.urandom(4)
        frame = bytearray([0x81])
        n = len(data)
        if n < 126:
            frame.append(0x80 | n)
        else:
            frame.append(0x80 | 126)
            frame += struct.pack(">H", n)
        frame += mask
        frame += bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        self.s.sendall(frame)

    def _recv(self):
        def rd(n):
            buf = b""
            while len(buf) < n:
                c = self.s.recv(n - len(buf))
                if not c:
                    raise EOFError("socket closed")
                buf += c
            return buf

        _b0, b1 = rd(2)
        ln = b1 & 0x7F
        if ln == 126:
            ln = struct.unpack(">H", rd(2))[0]
        elif ln == 127:
            ln = struct.unpack(">Q", rd(8))[0]
        return rd(ln).decode(errors="replace")

    def cmd(self, line, settle=0.3):
        self._send(line)
        time.sleep(settle)
        return self._recv()

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


def measure(con, args, trim_r):
    """Set ff_trim_r, home, run a closed-loop move while tracing, and return the
    mean L-R encoder divergence over the cruise portion (counts)."""
    con.cmd("config set motion.ff_trim_l 1.0")
    con.cmd(f"config set motion.ff_trim_r {trim_r:.4f}")
    con.cmd("config save")
    con.cmd("home")  # back to a clean zero
    con._send(f"motion {args.rpm} {args.dir}")  # non-blocking start
    time.sleep(0.3)
    con._recv()
    out = con.cmd(f"trace {args.trace_s} {args.hz}")
    con.cmd("motion stop")

    rows = [
        tuple(int(x) for x in r.split(","))
        for r in out.splitlines()
        if r and r[0].isdigit() and "," in r
    ]
    cruise = [
        left - right for (time, left, right) in rows if time >= args.cruise_start_ms
    ]
    if not cruise:
        return None
    return statistics.mean(cruise)


def tune(con, args):
    pts = []  # (trim_r, mean_L_minus_R)

    def take(trim):
        m = measure(con, args, trim)
        pts.append((trim, m))
        print(f"  ff_trim_r={trim:.3f}  ->  mean(L-R)={m:+.1f} counts")
        return m

    print(
        f"tuning ff_trim_r (rpm={args.rpm}, dir={args.dir}, "
        f"target |mean|<{args.target} counts)"
    )

    # First point at 1.0, then step toward reducing |mean| (R fast -> lower trim,
    # L fast -> higher trim), then secant from there.
    m0 = take(1.0)
    if m0 is None:
        print("no trace data — is the device homed-able and lowering clear?")
        return None
    if abs(m0) < args.target:
        return (1.0, m0)
    second = round(1.0 + (0.08 if m0 > 0 else -0.08), 3)
    take(second)

    best = min(pts, key=lambda p: abs(p[1]))
    for _ in range(args.max_iter):
        if abs(best[1]) < args.target:
            break
        (t0, m0), (t1, m1) = pts[-2], pts[-1]
        if m1 == m0:
            break
        nt = t1 - m1 * (t1 - t0) / (m1 - m0)  # secant -> mean = 0
        nt = max(t1 - 0.06, min(t1 + 0.06, nt))  # limit step size
        nt = max(args.trim_min, min(args.trim_max, round(nt, 3)))  # safety bounds
        if abs(nt - t1) < 0.004:
            break
        m = take(nt)
        if m is not None and abs(m) < abs(best[1]):
            best = (nt, m)
    return best


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--ip", default="10.0.0.135")
    p.add_argument("--rpm", type=int, default=60, help="closed-loop test speed")
    p.add_argument("--dir", choices=["lower", "raise"], default="lower")
    p.add_argument(
        "--trace-s",
        dest="trace_s",
        type=int,
        default=6,
        help="seconds of L/R capture per trial",
    )
    p.add_argument("--hz", type=int, default=50)
    p.add_argument(
        "--cruise-start-ms",
        type=int,
        default=1500,
        help="skip this much of each run as spin-up before averaging",
    )
    p.add_argument(
        "--target",
        type=float,
        default=6.0,
        help="stop once |mean(L-R)| is below this many counts (noise floor)",
    )
    p.add_argument("--max-iter", type=int, default=6)
    p.add_argument("--trim-min", type=float, default=0.78)
    p.add_argument("--trim-max", type=float, default=1.25)
    args = p.parse_args()

    con = Console(args.ip)
    try:
        best = tune(con, args)
        if best is None:
            return 1
        con.cmd(f"config set motion.ff_trim_r {best[0]:.4f}")
        con.cmd("config save")
        con.cmd("home")
        print(
            f"\nBEST: ff_trim_r = {best[0]:.3f}  (mean L-R = {best[1]:+.1f} counts) — saved"
        )
        print("ff_trim_l left at 1.0. This is a per-device value (its own motor pair).")
        return 0
    finally:
        try:
            con.cmd("motion stop")
        except Exception:
            pass
        con.close()


if __name__ == "__main__":
    sys.exit(main())
