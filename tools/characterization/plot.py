#!/usr/bin/env python3
"""Plot the open-loop characterization sweep produced by sweep.py.

Reads the per-run CSVs in an output directory (duty<DD>_<down|up>.csv) and
produces:

  * one PNG per run with 4 stacked subplots vs time — position (rotations),
    velocity (RPM), acceleration (RPM/s), current (mA) — with L and R overlaid.
  * summary PNGs across the sweep:
      - steady_rpm_vs_duty.png : steady-state RPM vs duty, per motor, down vs up
      - current_vs_duty.png    : steady-state + peak current vs duty, per motor

"Steady-state" = median over the middle half of each run (excludes the
spin-up and braking transients).

Needs matplotlib (and numpy). Run with the repo venv python, e.g.:
    ../.venv/bin/python plot.py --outdir runs/full
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import re

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

RUN_RE = re.compile(r"duty(\d+)_(down|up)\.csv$")


def load_csv(path: str) -> dict[str, list[float]]:
    cols: dict[str, list[float]] = {}
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for name in r.fieldnames or []:
            cols[name] = []
        for row in r:
            for name in r.fieldnames or []:
                try:
                    cols[name].append(float(row[name]))
                except (ValueError, KeyError):
                    cols[name].append(float("nan"))
    return cols


def median(xs: list[float]) -> float:
    s = sorted(v for v in xs if v == v)  # drop NaN
    n = len(s)
    if n == 0:
        return float("nan")
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0


def steady_slice(n: int) -> slice:
    return slice(n // 4, 3 * n // 4) if n >= 4 else slice(0, n)


def plot_run(path: str, outdir: str) -> None:
    m = RUN_RE.search(os.path.basename(path))
    if not m:
        return
    duty, direction = int(m.group(1)), m.group(2)
    c = load_csv(path)
    if not c.get("t_ms"):
        print(f"  (skip empty {os.path.basename(path)})")
        return
    t = [v / 1000.0 for v in c["t_ms"]]

    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    fig.suptitle(f"profile {direction}  duty {duty}%", fontsize=14, fontweight="bold")

    axes[0].plot(t, c["rot_l"], label="L", color="C0")
    axes[0].plot(t, c["rot_r"], label="R", color="C1")
    axes[0].set_ylabel("position\n(rotations)")

    axes[1].plot(t, c["vel_l_rpm"], label="L", color="C0")
    axes[1].plot(t, c["vel_r_rpm"], label="R", color="C1")
    axes[1].set_ylabel("velocity\n(RPM)")

    axes[2].plot(t, c["acc_l_rpmps"], label="L", color="C0")
    axes[2].plot(t, c["acc_r_rpmps"], label="R", color="C1")
    axes[2].set_ylabel("accel\n(RPM/s)")

    axes[3].plot(t, c["cur_l_ma"], label="L", color="C0")
    axes[3].plot(t, c["cur_r_ma"], label="R", color="C1")
    axes[3].set_ylabel("current\n(mA)")
    axes[3].set_xlabel("time (s)")

    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)

    fig.tight_layout(rect=(0, 0, 1, 0.98))
    out = os.path.join(outdir, f"duty{duty:02d}_{direction}.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"  wrote {os.path.relpath(out)}")


def collect_summary(paths: list[str]) -> dict:
    """{(duty, direction): {rpm_l, rpm_r, cur_l_ss, cur_r_ss, cur_l_pk, cur_r_pk}}"""
    out: dict[tuple[int, str], dict] = {}
    for path in paths:
        m = RUN_RE.search(os.path.basename(path))
        if not m:
            continue
        duty, direction = int(m.group(1)), m.group(2)
        c = load_csv(path)
        n = len(c.get("t_ms", []))
        if n == 0:
            continue
        sl = steady_slice(n)
        out[(duty, direction)] = {
            "rpm_l": median(c["vel_l_rpm"][sl]),
            "rpm_r": median(c["vel_r_rpm"][sl]),
            "cur_l_ss": median(c["cur_l_ma"][sl]),
            "cur_r_ss": median(c["cur_r_ma"][sl]),
            "cur_l_pk": max((v for v in c["cur_l_ma"] if v == v), default=float("nan")),
            "cur_r_pk": max((v for v in c["cur_r_ma"] if v == v), default=float("nan")),
        }
    return out


def _series(summary: dict, direction: str, key: str):
    pts = sorted((d, v[key]) for (d, dr), v in summary.items() if dr == direction)
    return [p[0] for p in pts], [p[1] for p in pts]


def plot_summary(summary: dict, outdir: str) -> None:
    if not summary:
        return

    # Steady-state RPM vs duty.
    fig, ax = plt.subplots(figsize=(9, 6))
    for direction, style in (("down", "-o"), ("up", "--s")):
        for motor, key, color in (("L", "rpm_l", "C0"), ("R", "rpm_r", "C1")):
            x, y = _series(summary, direction, key)
            if x:
                ax.plot(x, y, style, color=color, label=f"{motor} {direction}")
    ax.set_xlabel("duty (%)")
    ax.set_ylabel("steady-state output RPM")
    ax.set_title("Open-loop speed vs duty (slope ≈ 1 / duty_per_rpm)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    p = os.path.join(outdir, "steady_rpm_vs_duty.png")
    fig.tight_layout()
    fig.savefig(p, dpi=120)
    plt.close(fig)
    print(f"  wrote {os.path.relpath(p)}")

    # Current vs duty (steady-state solid, peak dashed).
    fig, ax = plt.subplots(figsize=(9, 6))
    for direction in ("down", "up"):
        for motor, ss_key, pk_key, color in (
            ("L", "cur_l_ss", "cur_l_pk", "C0"),
            ("R", "cur_r_ss", "cur_r_pk", "C1"),
        ):
            x, y = _series(summary, direction, ss_key)
            if x:
                ax.plot(x, y, "-o", color=color, alpha=0.9 if direction == "down" else 0.5,
                        label=f"{motor} {direction} steady")
            xp, yp = _series(summary, direction, pk_key)
            if xp:
                ax.plot(xp, yp, ":^", color=color, alpha=0.6 if direction == "down" else 0.3,
                        label=f"{motor} {direction} peak")
    ax.set_xlabel("duty (%)")
    ax.set_ylabel("current (mA)")
    ax.set_title("Current vs duty (steady-state + peak)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    p = os.path.join(outdir, "current_vs_duty.png")
    fig.tight_layout()
    fig.savefig(p, dpi=120)
    plt.close(fig)
    print(f"  wrote {os.path.relpath(p)}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--outdir", required=True, help="directory of duty<DD>_<dir>.csv files")
    args = p.parse_args()

    paths = sorted(glob.glob(os.path.join(args.outdir, "duty*_*.csv")))
    if not paths:
        print(f"no run CSVs found in {args.outdir}")
        return 1

    print(f"plotting {len(paths)} runs from {args.outdir}")
    for path in paths:
        plot_run(path, args.outdir)
    plot_summary(collect_summary(paths), args.outdir)
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
