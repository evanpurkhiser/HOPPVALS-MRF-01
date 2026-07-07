# Two-Motor Balance & the Once-Per-Revolution Wobble

Investigation notes on keeping the two lift motors synchronized, and on a
residual speed artifact we traced to mechanics rather than firmware. Captured so
the same diagnosis is repeatable on the other units.

## TL;DR

- The two motors are **not identical** — open-loop, the right motor runs
  **~4–9 % faster** than the left at the same duty. Left uncorrected, the speed
  loop has to continuously make up that constant offset, which reads as one
  motor lagging and the loop hunting to catch it up.
- **Fix: per-motor feedforward trim** (`motion.ff_trim_r`). Tuned closed-loop to
  **≈ 0.88** on this unit, which zeroes the steady L−R offset. Automated by
  `tools/characterization/tune_trim.py`. This is a **per-device** value (each
  unit has its own motor pair); the firmware default is 1.0.
- A **separate** artifact remains: an audible **once-per-output-revolution**
  speed oscillation (~3 RPM RMS, ~5 % of cruise) on the **left** motor. We ruled
  out trim, the sync loop, and PI gains as causes. It is **shaft-angle-locked**
  → **mechanical** (eccentric spool / bent shaft / uneven cord lead on the left
  side). Firmware can't remove a ~1 Hz shaft-locked disturbance; it's cosmetic.

## Control architecture (context)

Each motor runs an independent **feedforward + PI speed loop** at 100 Hz
(`firmware/components/motion`):

```
duty = (ff_offset + duty_per_rpm * setpoint) * ff_trim   # feedforward (per-motor trim)
     + Kp * error + Ki * integral                        # feedback (PI)
```

On top, a **cross-coupled sync** biases each motor's *setpoint* by
`k_sync * (count_other − count_me)` — the lagging motor speeds up, the leader
slows, no master. A sync **fault** trips if the encoders diverge past
`sync_fault_limit` counts.

The feedforward was fit from open-loop characterization
(`tools/characterization/sweep.py`): `duty_per_rpm ≈ 0.33` (≈ 3 RPM per % duty)
plus a static breakaway offset (`ff_offset_raise ≈ 10 %`, lower ≈ 0).

## Problem 1 — the two motors don't match (fixed)

### Measurement
Open-loop duty sweep, both motors at the same fixed duty (`profile` command,
`sweep.py`). Right consistently faster than left:

| duty (down) | L RPM | R RPM | R/L |
|---|---|---|---|
| 30 % | 82.7 | 90.1 | 1.09 |
| 40 % | 113.8 | 120.9 | 1.06 |
| 60 % | 174.1 | 182.1 | 1.05 |
| 80 % | 234.0 | 243.1 | 1.04 |

With identical feedforward for both, the right motor systematically leads; the
PI integral + sync loop must continuously manufacture that constant offset, and
the proportional position cross-coupling turns it into a slow hunt.

### Fix — per-motor feedforward trim
`ff_trim_l` / `ff_trim_r` multiply each side's feedforward. Hold one at 1.0 and
trim the faster motor down so both are commanded the right duty for the target
speed; the loop is then left only small disturbances to handle.

Tuned **against the closed-loop metric** (mean L−R encoder divergence over a
steady move), not the open-loop ratio — a secant search to mean ≈ 0:

| `ff_trim_r` | mean L−R (counts) |
|---|---|
| 1.000 | −40 |
| 0.920 | −13 |
| **0.883** | **−1.5** |

Result: **`ff_trim_r ≈ 0.88`** (R needs ~12 % less feedforward — *more* than the
open-loop ~5 % ratio, because the closed-loop offset is dominated by transient
divergence during spin-up + sync dynamics, not just the steady slope mismatch).

Run it with `tune_trim.py` (see [Reproducing](#reproducing) below). **Per
device** — re-run on each unit.

## Problem 2 — once-per-revolution wobble (mechanical, not fixed in firmware)

### Symptom
Audible: the left side **speeds up slightly, overshoots, then slows**, over and
over, during a move. Distinct from Problem 1 (a *constant* offset) — this is a
*periodic* oscillation that the trim does not remove.

### Measurement
FFT of per-motor velocity ripple in the **rotation domain** (cycles per output
revolution), both open- and closed-loop:

| | dominant period | ripple |
|---|---|---|
| L (open-loop, all duties) | **1.00 cyc/rev** | ~2.5 RPM RMS |
| L (closed-loop, 60 RPM) | **0.95 cyc/rev** | ~3.0 RPM RMS (~5 % of cruise) |
| R (closed-loop, 60 RPM) | 0.16 cyc/rev | ~2.1 RPM RMS |

`~1.0 cycle per output revolution`, constant across speed, on the left side =
**locked to output-shaft angle**. That rules out:

- **Encoder eccentricity** — would be once per *motor* rev = 32×/output-rev.
- **Cogging / PWM / electrical** — fixed in time/motor-rate, not output-shaft.

So it is downstream of the 32:1 gearbox: the **output shaft / spool / cord** on
the left side.

### Hypotheses tested and ruled out
| Hypothesis | Test | Result |
|---|---|---|
| Per-motor mismatch (trim) | tuned `ff_trim_r`, mean L−R → 0 | offset gone, **oscillation unchanged** |
| Sync loop chasing/propagating it | `k_sync` 0.02 vs 0 | detrended L−R ripple **identical** (5.7 vs 5.6) → not the sync |
| PI overshoot | Kp/Ki current vs soft vs no-integral | L ripple 3.33 / 3.29 / 2.92 RPM; **no audible change** |

Killing the integral shaved the ripple ~12 % (the PI added a little of its own
hunting), but the bulk is the mechanical disturbance, which no PI setting hides.

### Conclusion & recommendation
**Mechanical, on the left side.** Likely causes, in order:
- **Off-center / wobbling spool** on the output shaft (not concentric).
- **Bent output shaft**, or a **loose coupling / set-screw** letting the spool
  sit slightly off-axis.
- **Uneven cord lead / level-wind** — cord feeding at a varying radius, so load
  pulses once per wrap.

Check by spinning the left side (`spin L fwd 20`) and watching the spool for a
once-per-turn wobble or a pulsing cord radius.

Firmware cannot cleanly cancel a ~1 Hz shaft-locked disturbance; the only "real"
software fix would be a repetitive/iterative-learning controller keyed to
encoder angle (learn and feedforward-cancel the per-rev disturbance) — almost
certainly overkill for a blind. The artifact is **cosmetic**: the two sides
still track to ≈ 1.7 mm, position moves land cleanly, and the offset is zeroed.

## Reproducing

Device must be in WiFi debug mode and reachable (default `10.0.0.135`). All over
the websocket console.

**Open-loop characterization** (raw motor response, the duty→RPM fit):
```
tools/.venv/bin/python tools/characterization/sweep.py --label run --plot \
    --plot-python tools/.venv/bin/python
```

**Per-motor trim auto-tune** (zeroes the steady L−R offset):
```
python3 tools/characterization/tune_trim.py --ip 10.0.0.135
```

**Closed-loop capture for the once-per-rev analysis** — drive with the real
controller and record both encoders:
```
motion 60 lower     # non-blocking
trace 8 50          # 8 s of t_ms,L,R at 50 Hz while it drives
motion stop
```
Differentiate each encoder → per-motor RPM; FFT in the *rotation* domain
(velocity vs accumulated revolutions). A dominant ~1.0 cycle/rev that holds
across speeds is the mechanical, shaft-angle-locked signature.

## 2026-07-07 right-blind tuning notes

Device: right blind (`device.location = right`). Debug-mode WiFi address during
this session was `10.0.0.53` (WiFi MAC `98:a3:16:85:0f:38`). The Zigbee IEEE
for the same unit is `98:a3:16:ff:fe:85:0f:38`.

### Firmware/control changes under test

- `go_to_mm(0)` is now a normal absolute position move. There is no special
  top-capture path for zero; it targets `0` counts and arrives only when both
  encoders are independently within tolerance.
- Position moves bypass the average-position travel-limit stop. Manual velocity
  moves still enforce top/down travel limits.
- Stall detection now uses a windowed expected-progress check based on target RPM
  over `motion.stall_ms`, and resets the window on direction changes.
- `home()` now waits one `motion.home_settle` interval after the final all-motor
  brake, then resets both encoders at the actual resting top. Without this, the
  right blind rebounded about 30-40 counts after per-side homing reset, so an
  immediate `gotopct 0` tried to pull farther into the physical top and faulted.

### Right-blind config during successful verification

These are persisted on the right unit after this session:

| key | value |
|---|---:|
| `motion.ff_off_raise` | `10.00` |
| `motion.ff_off_lower` | `0.00` |
| `motion.goto_min_rpm` | `25` |
| `motion.goto_tol_mm` | `2.0` |
| `motion.goto_slow_mm` | `70.0` |
| `motion.cover_rpm` | `40` |
| `motion.k_sync` | `0.050` |
| `motion.sync_fault` | `400` |
| `motion.stall_ms` | `500` |
| `motion.grace_ms` | `1500` |
| `motion.soft_stop_mm` | `1500.0` |

### Position verification results

After the post-brake home reset fix:

| command | result |
|---|---|
| `home` | final `pos L=0.0mm R=0.0mm`, valid reference |
| `gotopct 0` from top | immediate `goto.arrived`, no motion/fault |
| `gotopct 5` then `gotopct 0` | clean arrivals, no faults |
| `gotopct 10` then `gotopct 0` | clean arrivals, no faults |
| `gotopct 25` then `gotopct 0` | clean arrivals, no faults |
| manual `gotopct 10` | arrived at `L=148.7mm R=149.6mm` |
| manual return to `gotopct 0` | arrived at `L=1.7mm R=2.2mm` |
| manual `gotopct 15` | arrived at `L=223.8mm R=224.6mm` |

The 10% and 25% cycles kept the sides within a few millimetres during motion and
ended within the 2 mm target tolerance after count-to-mm rounding.

### Slow approach observation

Raising `motion.goto_min_rpm` from 15 to 25 fixed the final pull into `0mm`, but
it visibly reduces the slow-down effect. The soft approach still exists, but the
floor now clamps the last part of the ramp at 25 RPM instead of 15 RPM.

This is probably not the right long-term shape. The better model is likely:

- Use extra duty/RPM only for breakaway or stalled-progress recovery.
- Once the motors are already moving, allow the setpoint to drop to a lower
  sustain speed for the final approach.
- Direction matters: lowering can run very slowly at low duty; raising has a much
  higher loaded breakaway threshold near the top.

Likely firmware follow-up: make the go-to slow approach direction-aware. A single
`motion.goto_min_rpm` is too blunt because lowering and raising have very
different physics. We probably want separate floors, or separate approach
profiles, for raise vs lower:

- Lower/down: can keep a low final RPM because gravity assists and even 10% duty
  moves both motors together.
- Raise/up: needs enough breakaway authority to start both motors under lift
  load, but should be able to drop to a lower sustain RPM after motion begins.

So the future model may be closer to `raise_breakaway_rpm/duty` plus
`raise_sustain_min_rpm`, and a lower `lower_min_rpm`, instead of one shared
`goto_min_rpm` for both directions.

### Open-loop profiling: lowering

Command used: `profile down <duty> 1 100 8` from the top area. Important: the
`profile` command resets encoder counts internally and drives both motors
together open-loop at the same duty; it is for local characterization, not an
absolute-position move.

| down duty | duration | first move | avg RPM L | avg RPM R | last-half RPM L | last-half RPM R |
|---:|---:|---:|---:|---:|---:|---:|
| 10% | 2.32s | 40ms | 26.2 | 25.9 | 23.4 | 28.3 |
| 12% | 1.94s | 40ms | 31.4 | 31.0 | 29.4 | 33.8 |
| 15% | 1.50s | 30ms | 40.5 | 40.1 | 38.8 | 43.2 |
| 18% | 1.26s | 20ms | 48.7 | 47.6 | 47.7 | 50.6 |
| 20% | 1.12s | 30ms | 55.0 | 53.8 | 54.5 | 57.9 |

Conclusion: lowering starts easily and is fast even at 10% duty. Gravity assists,
so low duty is enough to produce a stable slow-ish RPM.

### Open-loop profiling: raising from 15%

Clean run sequence for each duty: `home`, `gotopct 15`, wait for arrival, then
`profile up <duty> 1 100 8`. This keeps the physical start point comparable.

| up duty | outcome | duration | first move L | first move R | avg RPM L | avg RPM R |
|---:|---|---:|---:|---:|---:|---:|
| 12% | no useful movement | 7.99s | 80ms | none | 0.1 | 0.0 |
| 15% | no useful movement | 7.99s | 50ms | 70ms | 0.4 | 0.0 |
| 18% | L reached 1 rev, R did not | 7.99s | 40ms | 40ms | 7.5 | 6.8 |
| 20% | both reached 1 rev | 3.14s | 30ms | 40ms | 19.2 | 19.1 |
| 22% | both reached 1 rev | 2.22s | 30ms | 30ms | 27.2 | 27.0 |
| 25% | both reached 1 rev | 1.70s | 30ms | 30ms | 35.6 | 35.5 |

Conclusion: the loaded raise breakaway threshold from rest is around 20% duty on
this right blind. Duties below that may twitch or move one side, but are not
reliable for both motors together.

This does not prove the motors cannot **sustain** lower RPM once already moving.
It mostly measures static breakaway from rest under lift load. A useful next test
is to start raising at 20-25% duty/RPM, then trace whether the controller can
back down to a lower command while maintaining motion. Use `trace` during a
closed-loop `motion <rpm> raise` or `goto` run to capture encoder counts without
resetting them.

### Closed-loop sustain after breakaway

Test sequence: home, move down to a safe start point, command `motion 40 raise`
for a short breakaway interval, then command a lower raise RPM and record with
`trace 5 100`.

From about 30% down:

| commanded raise RPM after 40 RPM breakaway | outcome over 5s | avg RPM L | avg RPM R |
|---:|---|---:|---:|
| 8 | stalled / no sustained motion | 0.0 | 0.1 |
| 10 | sustained, uneven low speed | 6.4 | 6.5 |
| 12 | sustained | 9.0 | 8.9 |
| 15 | sustained | 12.5 | 12.6 |

From about 10% down, closer to the top load region:

| commanded raise RPM after 40 RPM breakaway | outcome over 5s |
|---:|---|
| 12 | sustained, stopped manually around 73-74 mm from top |
| 15 | sustained, stopped manually around 52-53 mm from top |

Conclusion: the motors need roughly 20% duty to **start** raising both sides from
rest under load, but once moving they can sustain a materially lower closed-loop
raise speed. This supports separating breakaway authority from final approach
speed rather than raising the global `goto_min_rpm` floor.

### Cross-blind closed-loop RPM comparison

Test sequence per blind: `home`, `motion 40 lower`, `trace 5 100`, `motion stop`,
then `motion 40 raise`, `trace 4 100`, `motion stop`. RPM below is derived from
1-second trace windows after startup, projected so requested direction is
positive. Trace files from this run were saved under
`/tmp/opencode/hv-mrf-rpm-compare`.

| blind | direction | median L RPM | median R RPM | side avg |
|---|---|---:|---:|---:|
| center | lower | 39.6 | 39.5 | 39.6 |
| left | lower | 39.0 | 38.4 | 38.7 |
| right | lower | 38.8 | 38.9 | 38.8 |
| center | raise | 36.7 | 37.2 | 37.0 |
| left | raise | 37.6 | 38.1 | 37.9 |
| right | raise | 36.4 | 36.7 | 36.5 |

Conclusion: per-blind speeds are already close. Lowering spread is ~0.8 RPM
(~2.1%) across blinds; raising spread is ~1.3 RPM (~3.5%) across blinds. This
does not suggest one blind needs unique tuning just to match the others.

However, raising is consistently slower than the 40 RPM command on every blind
(~36.5-37.9 RPM), while lowering is close to target (~38.7-39.6 RPM). If we want
raise to hit the requested RPM more exactly, that should probably be a shared
raise-feedforward/gain calibration rather than per-blind trim.

### Command gotchas from this session

- `trace <duration_s> <hz>` only records encoder counts. It does not drive
  motors, reset encoders, or change controller state.
- `profile <up|down> <duty> <rotations> [hz] [max_s]` drives both motors
  together open-loop. It resets both encoders before each run and brakes each
  side independently once that side reaches the requested rotation count.
- After running `profile`, run `home` before trusting absolute `pos` again.
- `selftest` has no help subcommand; `selftest help` is parsed as `selftest both`
  and will drive/reset the motors. Do not use it as help while installed.

## This unit's tuned values (per-device, in NVS)

`duty_per_rpm 0.33`, `ff_off_raise 10`, `ff_off_lower 0`, `ff_trim_l 1.0`,
`ff_trim_r 0.88`, `kp 0.10`, `ki 0.20`, `i_max 40`, `k_sync 0.02`,
`mm_per_rev 69.1`, `hard_stop_mm 1600`.

See also [DESIGN.md](DESIGN.md) → "Closed-loop speed control" and "Two-motor
synchronization".
