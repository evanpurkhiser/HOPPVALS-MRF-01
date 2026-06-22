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

## This unit's tuned values (per-device, in NVS)

`duty_per_rpm 0.33`, `ff_off_raise 10`, `ff_off_lower 0`, `ff_trim_l 1.0`,
`ff_trim_r 0.88`, `kp 0.10`, `ki 0.20`, `i_max 40`, `k_sync 0.02`,
`mm_per_rev 69.1`, `hard_stop_mm 1600`.

See also [DESIGN.md](DESIGN.md) → "Closed-loop speed control" and "Two-motor
synchronization".
