# Open-loop motor characterization

Drives both blind motors **open-loop** (no PI / sync / stall feedback) across a
sweep of fixed PWM duties and records, per motor:

- **position** (output-shaft rotations)
- **velocity** (RPM, derived)
- **acceleration** (RPM/s, derived)
- **current** (mA)

The point is to see the raw system response — how the two lift strings track
each other open-loop, where each motor starts moving under load, and the
steady-state speed/current at each duty — so we can re-fit the closed-loop
tuning (`duty_per_rpm`, gains, etc.).

## The cycle

For each duty in the sweep:

```
home  ->  profile down <down-rotations>  ->  profile up <up-rotations>  ->  home
```

`home` re-zeroes both encoders at the top hard stop before each run. The device
brakes each motor as soon as its own encoder reaches the target rotations, or at
a per-run time cap (`max_s`, default 25 s) if it stalls / can't move — so a low
duty that doesn't move just records "did not settle" and the sweep continues.

## Prerequisites

- The device must be in **WiFi debug mode** (flip the Home Assistant
  "Debug Mode" switch) and reachable at its IP (default `10.0.0.135`).
- `sweep.py` is **pure standard library** — no install needed. It drives the
  blind and saves CSVs.
- `plot.py` needs **matplotlib + numpy**, available in the repo venv at
  `tools/.venv`. Use that interpreter for plotting.

## Commands

Run from this directory.

```bash
# 1. Dry run — connect, read encoders, print the planned sequence. NO driving.
python3 sweep.py --dry-run

# 2. Validate a single duty end-to-end first (recommended before the full sweep).
python3 sweep.py --duties 30 --label validate

# 3. Full sweep (default duties 10..80 step 10).
python3 sweep.py --label full

# 4. Plot a finished run directory (use the venv python for matplotlib):
../.venv/bin/python plot.py --outdir runs/full

#    ...or have the sweep plot automatically when it finishes:
python3 sweep.py --label full --plot --plot-python ../.venv/bin/python
```

Useful flags: `--ip`, `--duties 10,20,30`, `--down-rotations`, `--up-rotations`,
`--hz`, `--timeout`.

## Output

Everything lands in `runs/<label>/` (gitignored):

- `duty<DD>_down.csv`, `duty<DD>_up.csv` — per-run data. Columns:
  `t_ms,count_l,count_r,cur_l_ma,cur_r_ma,rot_l,rot_r,vel_l_rpm,vel_r_rpm,acc_l_rpmps,acc_r_rpmps`
- `duty<DD>_<dir>.png` — per-run plot: position / velocity / accel / current vs
  time, L and R overlaid.
- `steady_rpm_vs_duty.png` — steady-state RPM vs duty (per motor, down vs up).
  Its slope is ≈ `1 / duty_per_rpm`.
- `current_vs_duty.png` — steady-state and peak current vs duty, per motor.

## How velocity/acceleration are derived

Encoder counts are converted to output rotations using `cpr` (896 counts/rev,
reported in each run's `PROFILE_BEGIN` line). Differentiating raw counts twice
is very noisy, so:

1. position is smoothed with a centered moving average (window **5** samples,
   ≈100 ms at 50 Hz),
2. velocity = central finite difference of smoothed position, scaled to RPM,
3. velocity is smoothed again (window **7** samples),
4. acceleration = central finite difference of smoothed velocity.

"Steady-state" values in the summary plots are the **median over the middle
half** of each run, excluding the spin-up and braking transients.

## Notes / caveats

- Runs block the websocket (single-flight) for their duration; that's expected.
- The firmware `profile` command drives **both motors together** at the same
  fixed duty. It brakes each side independently once that side reaches the
  requested rotation count, so the later part of a run may have one side stopped
  while the other catches up.
- `profile` resets both encoder counts at the start of each run. After manual
  profiling, run `home` before interpreting `pos`, `goto`, or percentage
  commands as absolute positions again.
- `trace <duration_s> <hz>` is read-only: it streams encoder counts and does not
  drive motors or reset encoders. Use it alongside `motion`, `goto`, or HA
  commands when you want to see what the closed-loop controller actually did.
- If the connection drops mid-run, the device keeps executing the current
  `profile` until its rotation target or `max_s` cap — there's no host
  heartbeat. The runs drive **up** to a hard stop (safe) and **down** within the
  configured rotations; the time cap is the backstop.
- `Ctrl-C` stops the host script, but the device finishes the current run and
  brakes on its own `max_s` cap; power-cycle if you need it to stop immediately.
