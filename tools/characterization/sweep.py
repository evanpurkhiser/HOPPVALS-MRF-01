#!/usr/bin/env python3
"""Open-loop motor characterization sweep for the HV-MRF-01 blind controller.

Drives BOTH motors open-loop (no PI/sync/stall feedback) at a series of fixed
duties, capturing per-motor position and current, and derives velocity (RPM)
and acceleration (RPM/s). For each duty the cycle is:

    home  ->  profile down <down-rotations>  ->  profile up <up-rotations>  ->  home

The goal is to see how the two lift strings track open-loop and to re-fit the
closed-loop tuning (duty_per_rpm, etc.).

This script is intentionally pure standard library: the part that physically
drives a ceiling-mounted blind must not fail on a missing third-party package.
Plotting (plot.py) is a separate step and is the only thing that needs
numpy/matplotlib.

Transport: the device exposes its console over a websocket at ws://<ip>/ws.
One text frame out = one command line; one text frame back = that command's
full captured stdout. Commands are serialized (single-flight) and some block
for many seconds, so the response timeout is generous.

Prerequisite: the device must be in WiFi debug mode (flip the HA "Debug Mode"
switch) and reachable at its IP.

Usage:
    # 1. Safe dry-run: connect, read encoders, print the plan. No driving.
    python3 sweep.py --dry-run

    # 2. Validate a single duty end-to-end before the full sweep.
    python3 sweep.py --duties 30 --label validate

    # 3. Full sweep.
    python3 sweep.py --label full

    # 4. Plot (needs the venv with matplotlib):
    ../.venv/bin/python plot.py --outdir runs/full
    # ...or let sweep.py invoke it:
    python3 sweep.py --label full --plot --plot-python ../.venv/bin/python
"""

from __future__ import annotations

import argparse
import base64
import csv
import os
import socket
import struct
import subprocess
import sys
from dataclasses import dataclass, field

# ── WebSocket client (RFC6455, pure stdlib) ────────────────────────────────


class WsError(Exception):
    pass


class WsClient:
    """Minimal text-only websocket client: connect, send a line, recv a line."""

    def __init__(self, ip: str, timeout: float):
        self.ip = ip
        self.timeout = timeout
        self.sock: socket.socket | None = None
        self._buf = b""

    def connect(self) -> None:
        s = socket.create_connection((self.ip, 80), timeout=self.timeout)
        s.settimeout(self.timeout)
        key = base64.b64encode(os.urandom(16)).decode()
        req = (
            f"GET /ws HTTP/1.1\r\n"
            f"Host: {self.ip}\r\n"
            f"Upgrade: websocket\r\n"
            f"Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            f"Sec-WebSocket-Version: 13\r\n\r\n"
        )
        s.sendall(req.encode())
        resp = self._read_http_headers(s)
        if "101" not in resp.split("\r\n", 1)[0]:
            raise WsError(
                f"handshake failed: {resp.splitlines()[0] if resp else '(empty)'}"
            )
        self.sock = s

    def _read_http_headers(self, s: socket.socket) -> str:
        data = b""
        while b"\r\n\r\n" not in data:
            chunk = s.recv(1024)
            if not chunk:
                raise WsError("connection closed during handshake")
            data += chunk
        head, _, rest = data.partition(b"\r\n\r\n")
        self._buf = rest  # any bytes after headers belong to the ws stream
        return head.decode(errors="replace")

    def close(self) -> None:
        if self.sock:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def _recv_raw(self, n: int) -> bytes:
        while len(self._buf) < n:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise WsError("connection closed")
            self._buf += chunk
        out, self._buf = self._buf[:n], self._buf[n:]
        return out

    def send(self, line: str) -> None:
        if not self.sock:
            raise WsError("not connected")
        payload = line.encode()
        mask = os.urandom(4)
        header = bytearray([0x81])  # FIN + text
        n = len(payload)
        if n < 126:
            header.append(0x80 | n)
        elif n < 65536:
            header.append(0x80 | 126)
            header += struct.pack(">H", n)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", n)
        header += mask
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        self.sock.sendall(bytes(header) + masked)

    def recv(self) -> str:
        """Receive one logical text message (handles fragmentation + control frames)."""
        chunks: list[bytes] = []
        while True:
            b0, b1 = self._recv_raw(2)
            fin = b0 & 0x80
            opcode = b0 & 0x0F
            masked = b1 & 0x80
            ln = b1 & 0x7F
            if ln == 126:
                (ln,) = struct.unpack(">H", self._recv_raw(2))
            elif ln == 127:
                (ln,) = struct.unpack(">Q", self._recv_raw(8))
            mask = self._recv_raw(4) if masked else b""
            payload = self._recv_raw(ln)
            if masked:
                payload = bytes(c ^ mask[i % 4] for i, c in enumerate(payload))

            if opcode == 0x8:  # close
                raise WsError("server closed the connection")
            if opcode == 0x9:  # ping -> pong
                self._send_pong(payload)
                continue
            if opcode == 0xA:  # pong
                continue
            # text (0x1) or continuation (0x0)
            chunks.append(payload)
            if fin:
                return b"".join(chunks).decode(errors="replace")

    def _send_pong(self, payload: bytes) -> None:
        mask = os.urandom(4)
        header = bytearray([0x8A, 0x80 | len(payload)])
        header += mask
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        self.sock.sendall(bytes(header) + masked)

    def command(self, line: str) -> str:
        self.send(line)
        return self.recv()


# ── Profile parsing + derived signals ──────────────────────────────────────

# Smoothing windows (in samples). Position is lightly smoothed before the
# velocity finite-difference; velocity is smoothed again before the
# acceleration difference, since double-differentiating raw counts is very
# noisy. At 50 Hz a window of 5 ≈ 100 ms.
POS_SMOOTH_WIN = 5
VEL_SMOOTH_WIN = 7


@dataclass
class Profile:
    direction: str
    duty: int
    rotations: int
    hz: int
    cpr: int
    done_l: bool
    done_r: bool
    samples: int
    rows: list[dict] = field(default_factory=list)  # parsed + derived per-sample


def parse_profile(text: str) -> Profile:
    lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
    begin = next((ln for ln in lines if ln.startswith("PROFILE_BEGIN")), None)
    end = next((ln for ln in lines if ln.startswith("PROFILE_END")), None)
    if begin is None:
        raise WsError(f"no PROFILE_BEGIN in response:\n{text[:300]}")

    meta = _kv(begin[len("PROFILE_BEGIN") :])
    cpr = int(meta.get("cpr", 896))

    # Data rows are the lines between the header row and PROFILE_END.
    data: list[tuple[int, int, int, int, int]] = []
    for ln in lines:
        if ln[0:1].isdigit():
            parts = ln.split(",")
            if len(parts) == 5:
                try:
                    data.append(tuple(int(p) for p in parts))  # type: ignore
                except ValueError:
                    pass

    done_l = done_r = False
    samples = len(data)
    if end is not None:
        em = _kv(end[len("PROFILE_END") :])
        done_l = em.get("done_l") == "1"
        done_r = em.get("done_r") == "1"
        samples = int(em.get("samples", samples))

    rows = _derive(data, cpr)
    return Profile(
        direction=meta.get("dir", "?"),
        duty=int(meta.get("duty", 0)),
        rotations=int(meta.get("rotations", 0)),
        hz=int(meta.get("hz", 50)),
        cpr=cpr,
        done_l=done_l,
        done_r=done_r,
        samples=samples,
        rows=rows,
    )


def _kv(s: str) -> dict[str, str]:
    out = {}
    for tok in s.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def _moving_avg(xs: list[float], win: int) -> list[float]:
    if win <= 1 or len(xs) < 2:
        return list(xs)
    half = win // 2
    out = []
    for i in range(len(xs)):
        lo = max(0, i - half)
        hi = min(len(xs), i + half + 1)
        out.append(sum(xs[lo:hi]) / (hi - lo))
    return out


def _central_diff(ys: list[float], ts: list[float]) -> list[float]:
    """d(ys)/d(ts), central where possible, one-sided at the ends. ts in seconds."""
    n = len(ys)
    if n < 2:
        return [0.0] * n
    out = [0.0] * n
    for i in range(n):
        if i == 0:
            dt = ts[1] - ts[0]
            out[i] = (ys[1] - ys[0]) / dt if dt else 0.0
        elif i == n - 1:
            dt = ts[-1] - ts[-2]
            out[i] = (ys[-1] - ys[-2]) / dt if dt else 0.0
        else:
            dt = ts[i + 1] - ts[i - 1]
            out[i] = (ys[i + 1] - ys[i - 1]) / dt if dt else 0.0
    return out


def _derive(data: list[tuple], cpr: int) -> list[dict]:
    if not data:
        return []
    t_s = [r[0] / 1000.0 for r in data]
    cl = [float(r[1]) for r in data]
    cr = [float(r[2]) for r in data]
    il = [r[3] for r in data]
    ir = [r[4] for r in data]

    # Work in magnitude-from-start so "rotations travelled" is positive
    # regardless of direction sign.
    cl0, cr0 = cl[0], cr[0]
    pos_l = [abs(c - cl0) for c in cl]
    pos_r = [abs(c - cr0) for c in cr]

    pos_l_s = _moving_avg(pos_l, POS_SMOOTH_WIN)
    pos_r_s = _moving_avg(pos_r, POS_SMOOTH_WIN)

    # counts/s -> output RPM
    k = 60.0 / cpr
    vel_l = [v * k for v in _central_diff(pos_l_s, t_s)]
    vel_r = [v * k for v in _central_diff(pos_r_s, t_s)]

    vel_l_s = _moving_avg(vel_l, VEL_SMOOTH_WIN)
    vel_r_s = _moving_avg(vel_r, VEL_SMOOTH_WIN)
    acc_l = _central_diff(vel_l_s, t_s)
    acc_r = _central_diff(vel_r_s, t_s)

    rows = []
    for i, r in enumerate(data):
        rows.append(
            {
                "t_ms": r[0],
                "count_l": r[1],
                "count_r": r[2],
                "cur_l_ma": il[i],
                "cur_r_ma": ir[i],
                "rot_l": round(pos_l[i] / cpr, 4),
                "rot_r": round(pos_r[i] / cpr, 4),
                "vel_l_rpm": round(vel_l[i], 2),
                "vel_r_rpm": round(vel_r[i], 2),
                "acc_l_rpmps": round(acc_l[i], 1),
                "acc_r_rpmps": round(acc_r[i], 1),
            }
        )
    return rows


CSV_COLUMNS = [
    "t_ms",
    "count_l",
    "count_r",
    "cur_l_ma",
    "cur_r_ma",
    "rot_l",
    "rot_r",
    "vel_l_rpm",
    "vel_r_rpm",
    "acc_l_rpmps",
    "acc_r_rpmps",
]


def save_csv(path: str, prof: Profile) -> None:
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        w.writeheader()
        w.writerows(prof.rows)


def steady_rpm(prof: Profile) -> tuple[float, float]:
    """Median RPM over the middle half of the run (excludes spin-up/brake)."""
    rows = prof.rows
    if len(rows) < 4:
        return 0.0, 0.0
    lo, hi = len(rows) // 4, 3 * len(rows) // 4
    seg = rows[lo:hi] or rows
    return _median([r["vel_l_rpm"] for r in seg]), _median(
        [r["vel_r_rpm"] for r in seg]
    )


def _median(xs: list[float]) -> float:
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return 0.0
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0


# ── Orchestration ───────────────────────────────────────────────────────────


def run_sweep(args) -> int:
    duties = [int(d) for d in args.duties.split(",") if d.strip()]
    client = WsClient(args.ip, timeout=args.timeout)

    print(f"connecting to ws://{args.ip}/ws ...")
    client.connect()
    enc = client.command("enc").strip()
    print(f"connected. encoders: {enc}")

    plan = []
    for d in duties:
        plan += [
            "home",
            f"profile down {d} {args.down_rotations} {args.hz}",
            f"profile up {d} {args.up_rotations} {args.hz}",
            "home",
        ]

    if args.dry_run:
        print("\n--- DRY RUN (no driving) — planned sequence ---")
        for step in plan:
            print(f"  {step}")
        print(f"\noutput dir would be: {args.outdir}")
        client.close()
        return 0

    os.makedirs(args.outdir, exist_ok=True)
    print(f"saving to {args.outdir}")
    summary: list[str] = []

    for d in duties:
        print(f"\n=== duty {d}% ===")
        _step(client, "home")
        for direction, rots in (
            ("down", args.down_rotations),
            ("up", args.up_rotations),
        ):
            cmd = f"profile {direction} {d} {rots} {args.hz}"
            print(f"  -> {cmd}")
            resp = client.command(cmd)
            try:
                prof = parse_profile(resp)
            except WsError as e:
                print(f"  !! parse failed: {e}")
                summary.append(f"duty {d:>2} {direction:<4}: PARSE FAILED")
                continue
            path = os.path.join(args.outdir, f"duty{d:02d}_{direction}.csv")
            save_csv(path, prof)
            srl, srr = steady_rpm(prof)
            rl = prof.rows[-1]["rot_l"] if prof.rows else 0
            rr = prof.rows[-1]["rot_r"] if prof.rows else 0
            warn = "" if (prof.done_l and prof.done_r) else "  <-- did NOT settle"
            print(
                f"     {prof.samples} samples, settled L={prof.done_l} R={prof.done_r}, "
                f"rot L={rl} R={rr}, steady rpm L={srl:.1f} R={srr:.1f}{warn}"
            )
            summary.append(
                f"duty {d:>2} {direction:<4}: L={'ok' if prof.done_l else 'NO'} "
                f"R={'ok' if prof.done_r else 'NO'}  rot L={rl:.2f} R={rr:.2f}  "
                f"rpm L={srl:5.1f} R={srr:5.1f}"
            )
        _step(client, "home")

    client.close()

    print("\n===== SWEEP SUMMARY =====")
    for line in summary:
        print(" ", line)
    print(f"\nCSVs in {args.outdir}")

    if args.plot:
        _invoke_plot(args)
    return 0


def _step(client: WsClient, cmd: str) -> None:
    print(f"  -> {cmd}")
    resp = client.command(cmd).strip()
    last = resp.splitlines()[-1] if resp else ""
    print(f"     {last}")


def _invoke_plot(args) -> None:
    plot_py = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plot.py")
    py = args.plot_python or sys.executable
    print(f"\nplotting via {py} ...")
    try:
        subprocess.run([py, plot_py, "--outdir", args.outdir], check=True)
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        print(f"plot step failed ({e}); run plot.py manually with the venv python.")


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--ip", default="10.0.0.135", help="device IP in WiFi debug mode")
    p.add_argument(
        "--duties",
        default="10,20,30,40,50,60,70,80",
        help="comma-separated duty %% list",
    )
    p.add_argument("--down-rotations", type=int, default=10)
    p.add_argument("--up-rotations", type=int, default=6)
    p.add_argument("--hz", type=int, default=50, help="device sample rate")
    p.add_argument(
        "--timeout", type=float, default=40.0, help="per-command response timeout (s)"
    )
    p.add_argument("--label", default="latest", help="run label -> runs/<label>/")
    p.add_argument("--outdir", default=None, help="override output dir")
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="connect + read enc + print plan; no driving",
    )
    p.add_argument("--plot", action="store_true", help="invoke plot.py at the end")
    p.add_argument(
        "--plot-python",
        default=None,
        help="python for plotting (e.g. ../.venv/bin/python)",
    )
    args = p.parse_args()
    if args.outdir is None:
        args.outdir = os.path.join(here, "runs", args.label)

    try:
        return run_sweep(args)
    except (WsError, OSError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print(
            "\ninterrupted — the device brakes on its own timeout; consider a power cycle.",
            file=sys.stderr,
        )
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
