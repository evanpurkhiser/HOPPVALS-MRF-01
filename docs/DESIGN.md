# Zigbee Motorized Blinds — Design

## Goals

- Motorize an **IKEA HOPPVALS** cellular (honeycomb) blind using two
  synchronized DC motors with grooved drum spools — one per lift cord.
- Expose it as a standard Zigbee 3.0 Window Covering device so any compliant
  hub (Home Assistant / ZHA, Zigbee2MQTT, deCONZ, SmartThings) works out of the
  box with no custom converters.
- Smooth motion: accelerate at the start and decelerate near the endstops
  rather than slamming the blind from stop to full speed.
- Hold position reliably with the factory tension spring removed (the motor
  becomes the only thing fighting gravity).
- Survive a power loss: remember position so we don't need to recalibrate on
  every reboot.
- Be safe: detect stalls and cut power before mechanical damage. Use the
  motor's own stall behavior to *discover* the top endpoint during
  calibration rather than relying on physical limit switches.

## Non-goals (for v1)

- Battery operation. Blinds are line-powered, so we run as a Zigbee Router
  (not a sleepy End Device). This gives us better mesh behavior for free.
- OTA firmware updates. Planned but not yet implemented — see
  [OTA.md](OTA.md) for the design (Zigbee-triggered, WiFi-delivered from a
  GitHub release). Needs a two-slot partition table swap before it can land.
- Tilt control. These are roller blinds, not venetians — `WindowCoveringType`
  is `Rollershade`, lift only.
- Light/temperature sensing or scheduling. The hub does that.

## Mechanical overview

IKEA HOPPVALS is a cellular (honeycomb) shade. Two lift cords run from the
top rail, down through the cells, and anchor to the bottom rail. Pulling
both cords *up* (winding them onto our drums in the head rail) compresses the
cells and raises the bottom rail. Releasing cord lets gravity expand the
cells and lower the rail.

Stock HOPPVALS has a torsion spring that counterbalances the bottom rail's
weight so the user can leave the blind at any height by hand. **That spring
has been removed** in this project — the motor + gearbox is now the only
thing holding the blind up against gravity. This has two consequences:

1. The motor needs a high-ratio gearbox (or other non-backdriveable feature)
   so the blind can't unwind itself under load when power is off.
2. When stopped at an intermediate position with power on, we electrically
   short-brake the motor (not coast) so it actively resists rotation.

```
   ┌─ head rail (our enclosure) ────────────────────────────────┐
   │                                                            │
   │  ┌─────┐                                          ┌─────┐  │
   │  │ ░░░ │◄── grooved drum spool ──►                │ ░░░ │  │
   │  │  M  │◄── DC motor + encoder ──►                │  M  │  │
   │  └──┬──┘                                          └──┬──┘  │
   │     │   rides on lead screw                          │     │
   │  ═══╧══════════════════════════════════════════════╧═══   │
   │     │   fixed lead screw                            │      │
   └─────│──────────────────────────────────────────────│──────┘
         │ lift cord L                       lift cord R│
         │                                              │
         ▼          ┌─────────────────────┐             ▼
        ╱╲╱╲╱╲╱╲╱╲╱╲│  honeycomb cells    │╱╲╱╲╱╲╱╲╱╲╱╲╱
                    │  (collapse / expand)│
        ╱╲╱╲╱╲╱╲╱╲╱╲│                     │╱╲╱╲╱╲╱╲╱╲╱╲╱
                    └──────────┬──────────┘
                               │
                    ┌──────────┴──────────┐
                    │   bottom rail       │ ◄── weight pulls blind down
                    └─────────────────────┘
```

Each side has:
- A DC gearmotor with a quadrature encoder.
- A grooved drum that spools the lift cord. As cord wraps on, the cells
  above collapse and the bottom rail rises.
- A carriage that slides along a fixed lead screw. As the motor turns, the
  carriage walks along the lead screw so the cord feeds onto the drum at a
  consistent angle (level-wind, like a fishing reel — no piled-up wraps).

There are **no limit switches** — endpoints are found electrically (see
[Endpoint discovery](#endpoint-discovery-no-limit-switches)), so the motor
connector carries only the H-bridge outputs and the encoder.

The two carriages are mechanically independent but must move in lockstep —
if one side raises faster than the other, the bottom rail tilts, the cells
crush unevenly, and the cord on the lagging side goes slack.

### Asymmetric load

Cellular shades have a meaningfully different load profile in each direction:

- **Raising**: motor fights bottom-rail weight + cell stack friction.
  Highest current draw, biggest stiction to overcome at start.
- **Lowering**: gravity does the work; the motor's job is to *meter out*
  cord at a controlled rate. The motor may even be back-driven (the load
  spins the motor rather than the other way around) if PWM duty drops too
  low.

This means the same PWM duty produces wildly different velocities up vs.
down. The closed-loop velocity controller handles this automatically — you
just get to see direction-asymmetric duty cycles in the logs.

## Hardware

| Part | Role |
|---|---|
| Seeed XIAO ESP32-C6 | MCU + Zigbee radio (built-in 802.15.4) |
| 2× DC motor with quadrature encoder | Drum drive + position feedback |
| 2× TI DRV8876PWPR H-bridge | One driver per motor; PH/EN mode + `IPROPI` current sense |
| 12 V DC supply | Board input + motor power (`VM`) |
| AP63205 buck (12 V → 5 V) | Logic power into the XIAO `VBUS`; the XIAO's onboard LDO makes 3.3 V |
| DMP3007 PMOS + PPTC fuse | Reverse-polarity protection + resettable over-current fuse on the 12 V input |

This is the **custom PCB build** (`PCB_DESIGN_OVERVIEW.md` has the full
schematic/layout walkthrough). The original breadboard prototype used a single
**TB6612FNG** dual H-bridge; the board revision swapped it for two DRV8876s to
gain per-motor current feedback (see "Motor drivers & current sense" below for
the why).

### Why two DRV8876s (one per motor)

Each DRV8876 is a single H-bridge with an analog current-sense output
(`IPROPI`), on-chip current regulation, 4.5–37 V motor supply, and 3.3 V-
tolerant logic so the XIAO drives it directly. One chip per motor gives
shorter motor traces, no shared-substrate heating between channels, and an
independent current reading per side — the headline feature we use for
load-aware stall detection.

The drivers run in **PH/EN mode** (`PMODE` strapped to GND): one pin sets
direction (`PH`), one PWM'd pin sets speed (`EN`). Critically for our spring-
less design (the motor must hold position against gravity), PH/EN's `EN` PWM
toggles the bridge between **drive and low-side slow-decay (brake)** — so the
winding is actively shorted during every off-interval, dynamically braking
against overspeed when lowering and holding when stopped. A shared `nSLEEP`
line wakes/sleeps both drivers together.

Driver state truth table (PH/EN mode, DRV8876 datasheet Table 3):

| `nSLEEP` | `EN` | `PH` | OUT1 | OUT2 | Mode |
|---|---|---|---|---|---|
| 1 | PWM | 1 | PWM | L | Forward |
| 1 | PWM | 0 | L | PWM | Reverse |
| 1 | 0 | x | L | L | **Brake** (low-side slow decay — hold against gravity) |
| 0 | x | x | Hi-Z | Hi-Z | Coast (both motors; never mid-travel) |

Which physical direction is "raise" vs "lower" depends on per-side motor
wiring polarity; the firmware's `to_raw()` mapping plus the per-side encoder
sign convention reconcile it. Coast is **shared** — `nSLEEP` low coasts both
motors at once, which is fine since the two cords must always move in lockstep.

## Power architecture

The board runs from a single **12 V DC** input; everything else is derived
on-board:

```
12 V in ─► F1 (PPTC fuse) ─► Q1 (PMOS rev-pol) ─► +12V rail
                                                   ├─► DRV8876 ×2  VM (motors)
                                                   ├─► C8 680 µF bulk
                                                   └─► AP63205 buck ─► +5V ─► XIAO VBUS
                                                                               └─(XIAO LDO)─► +3.3V rail
```

Three domains:
- **Motor (12 V `VM`)**: the two H-bridge outputs only. Each driver has a
  local 100 µF VM bulk; the rail shares a 680 µF bulk near the input.
- **Logic 5 V**: the AP63205 synchronous buck steps 12 V → 5 V into the XIAO's
  `VBUS` pin (the module's intended external-power input).
- **Logic 3.3 V**: produced by the XIAO's *onboard* LDO and tapped off its
  `3V3` pin — it is **not** generated on the board. It powers the DRV8876
  `VREF` dividers, `nFAULT` pull-ups, and the encoder connectors.

Common ground throughout.

### Input protection & decoupling

- **Reverse-polarity**: high-side P-FET (`DMP3007SPS-13`), body diode toward
  the load, gate pulled to GND — blocks current on a reversed input.
- **Fuse**: `BSMD2920` PPTC resettable fuse (4 A hold) in series ahead of the
  FET.
- **Bulk**: 680 µF on the protected 12 V rail + 100 µF local per driver, to
  absorb motor inrush / PWM transients before they dip `VM` enough to brown
  out the buck.

(Full rationale, part numbers, and trace sizing live in
`PCB_DESIGN_OVERVIEW.md`.)

## Pin allocation (XIAO ESP32-C6, front-side header only)

We stay on the 11 front-side header pins — no soldering to the back-side
castellated test pads. All 11 are now in use (the two current-sense lines took
the previously-spare D0/D1). This table is the as-built PCB netlist
(`PCB_DESIGN_OVERVIEW.md`) and is the single source of truth for pins.

| XIAO pin | GPIO | Use | Peripheral |
|---|---|---|---|
| D0 | GPIO0 | MOTOR_R_IPROPI (R current sense) | ADC1 ch 0 |
| D1 | GPIO1 | MOTOR_L_IPROPI (L current sense) | ADC1 ch 1 |
| D2 | GPIO2 | MOTOR_SLEEP (shared driver enable, `nSLEEP`) | GPIO OUT |
| D3 | GPIO21 | ENCODER_R_B | PCNT unit 1 |
| D4 | GPIO22 | ENCODER_R_A | PCNT unit 1 |
| D5 | GPIO23 | ENCODER_L_A | PCNT unit 0 |
| D6 | GPIO16 | ENCODER_L_B | PCNT unit 0 |
| D7 | GPIO17 | MOTOR_R_EN (R speed PWM) | LEDC ch 2 |
| D8 | GPIO19 | MOTOR_R_PH (R direction) | GPIO OUT |
| D9 | GPIO20 | MOTOR_L_EN (L speed PWM) | LEDC ch 1 |
| D10 | GPIO18 | MOTOR_L_PH (L direction) | GPIO OUT |

**Hard-wired on the board (not on GPIOs):** DRV8876 `PMODE` → GND (PH/EN
mode); `VREF` → 1.65 V divider (≈2.95 A current-regulation trip); `IMODE` →
20 kΩ (cycle-by-cycle chopping + auto-retry); `nFAULT` → 3.3 V pull-up only,
**not** routed to the MCU (all 11 GPIOs are used — stall is inferred from
`IPROPI` instead).

**Driver enable:** unlike the breadboard's `STBY`-tied-to-VCC, `nSLEEP` is now
a real GPIO (D2, shared across both drivers). Firmware drives it high at boot
so the bridges are awake to short-brake and hold position.

**Reserved by board:** GPIO3 (RF switch), GPIO9 (BOOT button), GPIO14
(antenna selector), GPIO15 (user LED, used by the `led` component for the
ZCL Identify effect).

**LEDC channel allocation:** ch 0 = user LED (the `led` component); ch 1 =
Motor L `EN`; ch 2 = Motor R `EN`. PWM now rides on the `EN` pins (the DRV8876
has no separate PWM input). All low-speed mode (the C6 has no high-speed mode),
sharing one 25 kHz timer.

### Why PCNT for encoders

The ESP32-C6 has 4 hardware pulse counter units. Each unit decodes a
quadrature encoder in hardware with no CPU involvement (counts up on A→B
transitions, down on B→A). We just poll the counter value or hook the
overflow/threshold interrupt. This is *vastly* better than reading both
encoder pins via GPIO ISRs at high speed — the CPU would spend its life in
interrupt handlers and still miss counts.

## Software architecture

```
┌──────────────────────────────────────────────────────┐
│                    app_main                          │
│  init NVS, init Zigbee stack, spawn tasks            │
└─────┬─────────────────┬─────────────────┬────────────┘
      │                 │                 │
      ▼                 ▼                 ▼
┌──────────┐  ┌──────────────────┐  ┌─────────────┐
│  Zigbee  │  │  Motion control  │  │  Hardware   │
│  task    │◄─┤  task (100 Hz)   │◄─┤  ISRs       │
│ ┌──────┐ │  │ ┌─────────────┐  │  │ ┌─────────┐ │
│ │ZCL   │ │  │ │ trajectory  │  │  │ │BOOT btn │ │
│ │attr  │ │  │ │ generator   │  │  │ │PCNT ovf │ │
│ │cmd   │ │  │ ├─────────────┤  │  │ └─────────┘ │
│ │handl │ │  │ │ per-motor   │  │  │             │
│ │ers   │ │  │ │ PID         │  │  │             │
│ └──────┘ │  │ ├─────────────┤  │  │             │
│          │  │ │ sync logic  │  │  │             │
│          │  │ ├─────────────┤  │  │             │
│          │  │ │ safety      │  │  │             │
│          │  │ └─────────────┘  │  │             │
└──────────┘  └──────────────────┘  └─────────────┘
      ▲                 │                  │
      └─── position ────┘                  ▼
           reports                   PWM + GPIO out
```

### Tasks

| Task | Prio | Tick | Responsibility |
|---|---|---|---|
| `zb_main` | 5 | event-driven | Zigbee stack mainloop, ZCL command handling, attribute reporting |
| `motion` | 6 | 10 ms (100 Hz) | Trajectory gen, per-motor closed-loop control, sync, stall detection |
| `persistence` | 3 | on demand | Write position snapshots to NVS (rate-limited to avoid flash wear) |

ISR work is minimal: the BOOT button just sets a flag and notifies the motion
task. PCNT decoding happens in hardware, and current sensing is polled by the
`current_sense` task — no per-edge interrupts.

### Module layout

C++23 throughout, organized as ESP-IDF components so each module gets its
own `CMakeLists.txt` + namespace + clean public header.

```
main/
└── main.cpp                          app_main, wiring; ~50 LOC
components/
├── zigbee/
│   ├── include/blinds/zigbee.hpp     public API (events, cover handlers)
│   └── src/zigbee.cpp                stack init, ZCL handlers, attr writes
├── motor/
│   ├── include/blinds/motor.hpp      raw motor drive (PH/EN + shared enable)
│   └── src/motor.cpp                 LEDC + GPIO config, debug API
├── encoder/
│   ├── include/blinds/encoder.hpp    quadrature reader
│   └── src/encoder.cpp               PCNT hardware decoder
├── current_sense/
│   ├── include/blinds/current_sense.hpp  IPROPI current reader + overcurrent events
│   └── src/current_sense.cpp             ADC1 oneshot + 100 Hz sampling task
├── motion/                           (next)
│   ├── include/blinds/motion.hpp     speed/position controller API
│   └── src/motion.cpp                PI+FF speed loop, trapezoid trajectory
├── calibration/                      (next)
│   ├── include/blinds/calibration.hpp
│   └── src/calibration.cpp           stall-discovery routine
├── persistence/                      (next)
│   ├── include/blinds/persistence.hpp
│   └── src/persistence.cpp           NVS save/load
├── led/
│   ├── include/blinds/led.hpp        user-LED ZCL Identify driver
│   └── src/led.cpp                   LEDC fade effects
└── cli/
    ├── include/blinds/console.hpp    serial REPL (USB-Serial-JTAG)
    └── src/console.cpp               motor/encoder/current debug commands (incl. `ramp`)
```

Inter-component communication: **events** (`esp_event` bus) for fire-and-
forget notifications, **synchronous handler callbacks** for request/response
commands, **direct function calls** for unidirectional state updates.
Components never include each other's source — only public headers.

## Motion control

### Coordinate convention

Internally we work in **encoder counts**, with 0 = fully open (top) and
`max_counts` = fully closed (bottom). We expose this to Zigbee as a 0–100
percentage where 0 = open, 100 = closed (matching the ZCL spec — yes, this
is counterintuitive).

### Closed-loop speed control (PI + feedforward)

Motors are commanded in **RPM**, not PWM duty. The duty-vs-RPM relationship
was measured to be essentially linear in the operating range (~2.8 RPM per
% duty, no load), which makes feedforward control easy and effective.

Each motor has its own controller running at 100 Hz:

```
ff_duty   = setpoint_rpm * DUTY_PER_RPM           # feedforward
error     = setpoint_rpm - measured_rpm
i_accum  += error * dt                            # integral
i_accum   = clamp(i_accum, -I_MAX, +I_MAX)        # anti-windup
duty      = ff_duty + Kp * error + Ki * i_accum
duty      = clamp(duty, 0, 100)
```

- **Feedforward** (`ff_duty`) does the heavy lifting: knowing the linear
  duty/RPM map, we land at roughly the right duty on the first tick. PI
  then corrects for load, supply voltage drift, and motor variability.
- **P** absorbs transient error.
- **I** eliminates steady-state offset — under real cord load, the motor
  needs slightly more duty than no-load to hit the target. The integral
  finds that offset automatically.
- **D is intentionally absent.** It amplifies encoder quantization noise
  without buying real damping (motor + load inertia already provides that).

Measured RPM = `(Δcount / Δt) / COUNTS_PER_OUTPUT_REV × 60`. We sample
encoder count over the 10 ms control-loop window.

Tuning constants (in `motion/config.hpp`):

| Constant | Initial value | Meaning |
|---|---|---|
| `DUTY_PER_RPM` | ≈ 1 / 2.8 | Feedforward slope, calibrated empirically |
| `Kp` | start at 0.1, ramp up | Proportional gain (units: %duty per RPM error) |
| `Ki` | start at 0.5 | Integral gain |
| `I_MAX` | 30 | Anti-windup clamp on the integral accumulator |
| `CONTROL_HZ` | 100 | Update rate |

### Trajectory generation

The application asks for *position*, but the controller wants *speed*. A
trajectory generator sits between them, producing a smoothly-shaped speed
profile that respects acceleration limits:

```
velocity
   ▲
   │       ┌──────────────┐
v_max ─────│              │─────
   │      /                \
   │     /                  \
   │    /                    \
   │___/                      \___  ─► time
       ◄── accel ──►◄── cruise ──►◄── decel ──►
```

The generator outputs `setpoint_rpm(t)` to the per-motor speed loop. For
short moves (target within `2 × decel_distance`), it uses a triangular
profile (accel straight into decel, no cruise).

Tuning:
- `MOTION_V_MAX` — cruise RPM (e.g. 60 RPM for ~25 s full travel).
- `MOTION_ACCEL` — RPM/sec accel ramp. Smaller = silkier, larger = snappier.
- `MOTION_DECEL` — usually slightly slower than accel so we ease into target.

### Easing — making it feel silky

A bare trapezoid (linear accel ramps) has *constant acceleration* during
the ramp phases, which means **instantaneous jerk** (derivative of accel)
at the four corners: the moment we start ramping, the moment we hit cruise,
the moment we begin decel, the moment we hit zero. On the motor this
manifests as small "thumps" — barely audible but perceptible. We can do
much better.

The trajectory generator is structured so its accel and decel ramps can be
shaped by a swappable **easing function** `f(τ) → v_normalized` where
`τ ∈ [0,1]` is normalized phase time and `v_normalized ∈ [0,1]` is the
speed fraction within that phase. The current speed at phase position τ is:

```
setpoint_rpm = v_min + (v_max - v_min) * f(τ)        # accel phase
setpoint_rpm = v_max - (v_max - v_min) * f(τ)        # decel phase
```

Choices we can swap at runtime:

| Easing | f(τ) | Character |
|---|---|---|
| **Linear** (default trapezoid) | τ | Punchy. Audible jerk at corners. |
| **Cosine ease-in-out** | (1 - cos(πτ)) / 2 | Silky. Velocity smoothly enters/exits the cruise plateau. Zero jerk at corners. Tiny CPU cost. |
| **Cubic ease-in-out** | 3τ² - 2τ³ | Very close to cosine but cheaper (no `cos`). Slightly less smooth in the third derivative. |
| **Quintic ease-in-out** | 6τ⁵ - 15τ⁴ + 10τ³ | Smoother than cubic. Continuous jerk *and* snap. Overkill for blinds but pretty. |
| **CSS-style cubic-bezier** | bezier(p1, p2)(τ) | Fully tunable feel. Same parameterization as CSS animations. |

**Default**: cosine ease-in-out. Quiet, near-zero CPU, no tuning knobs to
get wrong.

#### Spring physics (alternative trajectory generator)

Instead of pre-planning a trajectory of any shape, we can replace the whole
generator with a virtual **critically-damped spring** pulling the soft
target toward the goal:

```
soft_target += velocity * dt
velocity    += (-k * (soft_target - goal) - c * velocity) * dt    # c = 2√k
```

The motor's speed loop chases `soft_target`. The result is the iOS-style
"feels alive" motion: faster when far from the goal, slower as we approach,
no constant-speed cruise phase at all. Very pleasant to watch on a UI
slider; on a blind motor it'd land softly without overshoot.

Caveats:
- The spring asymptotes — it never quite *reaches* the goal. Need a small
  dead-band: "within N counts, snap and stop".
- It doesn't respect a max-speed cap naturally — needs a clamp on velocity.
- Tuning `k` (spring stiffness) is per-motor, per-load. Likely needs a
  re-tune after calibration.

The trajectory generator interface is small enough that we can ship both
the trapezoid-with-easing path and the spring path, and let the user pick
via a runtime config (or just by feel during testing). Same speed-loop
input, different generator on top.

### Two-motor synchronization

The blind has two cords, one per side, each spooled by its own motor. If
the motors don't stay locked together count-for-count, the bottom rail
tilts and the cells crumple — the blind looks visibly wobbly.

Three approaches were considered:

**(A) Master/slave (velocity coupling).** Motor L runs at the commanded
RPM; motor R's setpoint mirrors motor L's *measured* RPM. Simple, one
real controller. **Downside**: R is always reacting to L's last tick, so R
lags by one control period (~10 ms). Over a long move, **position error
accumulates monotonically** unless we also add a position correction term.
Asymmetric — the mechanically faster motor effectively becomes the master.

**(B) Position-sync (shared trajectory, independent loops).** Both motors
share a single trajectory generator producing `target_position(t)`. Each
motor has its own independent PI speed loop chasing that time-based
target. Symmetric — no leader. **Downside**: the motors don't *actively*
pull toward each other. Mild count drift accumulates quietly until the
sync watchdog notices.

**(C) Cross-coupled position-sync (chosen).** Same shared trajectory as
(B), but each motor's RPM setpoint includes a bias term tied to the *other
motor's count*:

```
For each motor i (where j = the other motor):
    base_setpoint = trajectory.velocity(t)
    sync_bias     = K_sync × (count_j - count_i)
    rpm_setpoint  = base_setpoint + sync_bias
```

If motor L is 50 counts behind motor R, L's setpoint instantly nudges up
by `50 × K_sync` RPM; R simultaneously sees `-50 × K_sync` and slows a
touch. The system **actively converges** toward equal counts. Both motors
are equal citizens — no master.

This composes cleanly with the rest of the stack:

```
trajectory.tick()                # outputs target_position(t), target_velocity(t)
    │
    ├── motor_L.set_rpm(velocity + K_sync × (count_R - count_L))
    └── motor_R.set_rpm(velocity + K_sync × (count_L - count_R))

each motor's PI loop runs independently against its own setpoint
```

The PI loops themselves don't need to know about each other; cross-coupling
lives in the trajectory→setpoint stage above. Easing functions and spring
trajectories swap in without touching sync logic.

**Tuning `K_sync`**: start low (≈0.05 RPM per count of error) and increase
until the motors *visibly* snap back into alignment after a perturbation
without ringing. Too high → the two controllers fight each other in
oscillation. Too low → drift slips through.

**Sync-error watchdog (still required).** Independent of the corrective
bias, a hard fault catches gross divergence — e.g. one motor stalled
mechanically while the other continues:

```
if |count_L - count_R| > SYNC_FAULT_LIMIT:
    fault → stop both motors, post fault event
```

Threshold sized for "more counts than ~5 mm of cord travel" — enough
slack that one-tick instantaneous lag from cross-coupling settling doesn't
trip it, tight enough that real mechanical mismatch does.

### Stall detection

If the trajectory says we should be moving but `|Δcounts|/Δt` is below
`STALL_VELOCITY_MIN` for `STALL_TIMEOUT_MS`, we drop `EN` to 0 (brake — we
don't coast, since that would let gravity unwind the cord) and report a fault
via Zigbee. Recovery requires a re-calibration or a hub command.

## Endpoint discovery (no limit switches)

We deliberately don't use physical limit switches. The closed-loop speed
controller + encoder give us enough information to find the endpoints
electrically, which saves four GPIOs, eliminates four microswitches and
their wiring + alignment, and simplifies the mechanical design.

### Top endpoint — stall detection

The top is a *hard* mechanical stop: the cord is fully wound onto the drum
and won't go further. Driving into that condition stalls the motor.

Detection algorithm (during calibration):
1. Set speed-loop target to a low, motor-friendly RPM (e.g. 30 RPM).
2. The PI controller raises duty until that RPM is achieved.
3. As the carriage approaches the stop, friction increases — the PI's
   integral term creeps up to compensate.
4. At the stop, the motor can no longer turn. Measured RPM falls toward 0.
5. If `measured_rpm < STALL_RPM_THRESHOLD` (~5 RPM) for `STALL_DURATION_MS`
   (~200 ms), we declare stall. Cut PWM immediately.
6. Record `encoder::count()` as `top_position`.

This intentionally stresses the motor briefly, so we don't do it on every
boot — only when explicitly recalibrating (see Calibration).

Safety: a hard timeout (e.g. 90 seconds of calibration travel) cuts power if
stall never fires — protects against runaway in case the gearbox somehow
slips.

### Bottom endpoint — count-based

The bottom has no hard mechanical stop. The bottom rail just dangles at the
end of the unwound cord. Stall detection doesn't apply.

Instead the bottom is **calibrated as a count offset from the top**:

```
bottom_position = top_position + TRAVEL_COUNTS
```

Where `TRAVEL_COUNTS` is derived from the known blind drop and drum
geometry: `(blind_drop_mm / drum_circumference_mm) × COUNTS_PER_OUTPUT_REV`.

For our build: 1800 mm drop / (π × 22 mm) circumference ≈ 26 output revs ×
896 counts/rev ≈ **23,300 counts**. We store this constant and use it
directly; no physical bottom calibration sweep required.

In operation, the motion controller refuses any target that would drive the
encoder count outside `[top_position, top_position + TRAVEL_COUNTS]`.

### Why this is OK without redundancy

The encoder-only approach assumes the encoder tracks the drum faithfully —
i.e., no cord slippage, no spool overrun. For a grooved drum + level-wind
carriage system that's a reasonable assumption. Signs of trouble we'd watch
for:

- **Sync error** between L and R motors (`|count_L - count_R| > threshold`)
  — fault, force recalibration.
- **Stall while not near a known endpoint** — something's jammed, fault.
- **User reports the blind is in the wrong place** in HA — trigger
  recalibration via the Mode attribute.

If real-world experience shows the assumption breaks down, we can revisit
adding back a single shared top-only limit switch as a sanity check.

## Calibration

Triggered by writing `Mode.CalibrationMode = 1` from the hub, or by holding
the BOOT button at startup for >3 s.

Sequence:
1. Set speed-loop target to a slow `CALIBRATION_RPM` (≈30 RPM) in the *up*
   direction.
2. Wait for stall detection (see "Top endpoint" above). Hard timeout: 90 s.
3. On stall: cut PWM. Record `encoder::count()` as `top_position` for each
   motor.
4. Compute `bottom_position = top_position + TRAVEL_COUNTS`.
5. Verify `|top_position_L - top_position_R| < SYNC_TOL` — if not, the
   spools aren't matched and we fault.
6. Drive both motors down to ~50% travel as a sanity check, then stop.
7. Persist `top_position_L`, `top_position_R`, `TRAVEL_COUNTS` to NVS. Clear
   `Mode.CalibrationMode`.

The first power-on after flashing forces calibration before the device will
accept move commands. After that, calibration only re-runs when explicitly
requested or when a fault forces it.

## Persistence

NVS namespace `blinds`. Keys:
- `top_l`, `top_r` — per-side `top_position` discovered by stall during
  calibration. Origin for position math.
- `travel_counts` — encoder counts from top to bottom (derived once from
  drum geometry, persisted for reference).
- `pos_last` — last known position percentage (0–100).
- `cal_done` — bool; if false, force calibration before accepting commands.
- `mode` — current Window Covering `Mode` bitmap (motor reversed, LED feedback).

We write `pos_last` only on **stop** (not every tick — that'd burn out the
flash in a week). Flash is rated for ~100k writes per sector; this gives
us decades.

On boot, we load `pos_last` and assume the blind is still there. The
gearbox is non-backdriveable and we short-brake when powered, so the blind
shouldn't move while we're off. If the assumption is wrong — user manually
moved it, slippage between sessions — the next stall during a routine move
will catch the inconsistency and trigger fault → recalibration.

## Zigbee data model

One endpoint (`BLINDS_ENDPOINT_ID = 10`) with these clusters:

| Cluster | ID | Role | Why |
|---|---|---|---|
| Basic | 0x0000 | Server | Manufacturer name, model, firmware ver |
| Identify | 0x0003 | Server | "Blink so I can find you" — required by hubs |
| Groups | 0x0004 | Server | Lets hub put this in a group (e.g. "all blinds") |
| Scenes | 0x0005 | Server | Optional, lets hub save/recall position presets |
| Window Covering | 0x0102 | Server | The actual blind functionality |

Implemented Window Covering attributes:

| Attr | ID | Meaning |
|---|---|---|
| WindowCoveringType | 0x0000 | `Rollershade` (0x00) |
| ConfigStatus | 0x0007 | `Operational | Online | LiftClosedLoop | LiftEncoderControlled` (0x2B) |
| CurrentPositionLiftPercentage | 0x0008 | The slider value the hub shows |
| InstalledOpenLimitLift | 0x0010 | Always 0 (we normalize to %) |
| InstalledClosedLimitLift | 0x0011 | Always 100 |
| Mode | 0x0017 | Writable bitmap, controls reversed/calibration/maintenance |

Implemented commands:

| Cmd | ID | Action |
|---|---|---|
| UpOpen | 0x00 | `set_target_percentage(0)` |
| DownClose | 0x01 | `set_target_percentage(100)` |
| Stop | 0x02 | Cancel current trajectory, hold position |
| GoToLiftPercentage | 0x05 | `set_target_percentage(payload)` |

The motion task writes `CurrentPositionLiftPercentage` whenever it changes
by ≥1%. Reporting configuration is left to the hub (it'll set up the
attribute report at pairing time per its own policy).

## State machine

```
                  ┌─────────────────┐
        boot ───► │  UNCALIBRATED   │
                  └────────┬────────┘
                           │ cal_done in NVS
                           ▼
                  ┌─────────────────┐
                  │      IDLE       │◄─────────────┐
                  └────────┬────────┘              │
                  cmd to move │                    │ trajectory done
                              ▼                    │ or Stop cmd
                  ┌─────────────────┐              │
                  │     MOVING      │──────────────┘
                  └────────┬────────┘
                  stall/sync│error / overcurrent
                            ▼
                  ┌─────────────────┐
                  │     FAULT       │
                  └────────┬────────┘
                  hub requests recal
                           ▼
                  ┌─────────────────┐
        manual ─► │   CALIBRATING   │
                  └────────┬────────┘
                           │ success
                           ▼ (back to IDLE)
```

## Safety priorities

In order of precedence, every motion-task tick checks:
1. **Overcurrent** (`IPROPI` over threshold) — instant stop (short-brake,
   `EN`=0), fault. The DRV8876 also chops at its own ~2.95 A hardware limit
   independently.
2. **Sync error** — instant stop, fault.
3. **Stall detected** — stop, fault.
4. **Soft limit reached** — clean stop at target.
5. **Trajectory complete** — clean stop at target.

### Rest-state policy

When motion completes, at any position we leave the driver **awake with
`EN`=0** → low-side slow-decay brake: the motor terminals are shorted so the
gearbox + dynamic brake together prevent the blind drifting down. This costs
only the DRV8876's small quiescent current, meaningless for a line-powered
blind.

We don't fully sleep the drivers (`nSLEEP` low → coast) even at the endpoints.
Without limit switches there's no hard mechanical confirmation we're truly at
a stop, `nSLEEP` is shared so we couldn't sleep one motor independently
anyway, and coasting would let gravity unwind the cord. Holding the brake
everywhere is simplest and safe.

## Manual operation (debug)

The XIAO BOOT button (GPIO9) is repurposed as a manual override at runtime:
- Single tap: stop / start trajectory toggle.
- Double tap: full open.
- Triple tap: full close.
- Hold 3 sec at boot: force calibration.

Useful for testing without a hub paired, and as a fallback if the hub is
unreachable.

## Calibrated values (so far)

| Constant | Value | Source |
|---|---|---|
| Encoder PPR | 7 (× 4 quadrature = 28 counts per motor rev) | Empirical: 1 output rev = 897 counts |
| Gearbox ratio | 1:32 | Motor spec |
| `COUNTS_PER_OUTPUT_REV` | 896 | 7 × 4 × 32 |
| Rated output speed | 260 RPM @ rated load | Motor spec |
| Measured speed, 100% duty, no load | 296 RPM | CLI sweep (breadboard) |
| Top speed, 100% duty, no load (PCB) | 309 RPM (L) / 311 RPM (R) | `ramp` CLI (per-motor bench) |
| Duty-to-RPM slope, no load | ~2.8 RPM per % duty | Linear fit 10–100% |
| PWM frequency | 25 kHz | Above audible; quiet sweep result |
| Drum circumference | π × 22 = 69.1 mm/output rev | Mechanical |
| Blind drop | 1800 mm | Mechanical |
| `TRAVEL_COUNTS` (top→bottom) | ≈ 23,300 | (1800 / 69.1) × 896 |
| Target operating speed | 60 RPM output (≈26 s full travel) | Design preference |
| Duty needed for 60 RPM, no load | ~23% | Measured |

All measurements above are **no-load** and will shift once the cord +
bottom-rail load is mechanically engaged. We'll redo the duty/RPM sweep at
that point.

## Motor drivers & current sense (DRV8876, as built)

The breadboard prototype used a single **TB6612FNG** driving both motors. The
custom PCB swapped that for **two TI DRV8876PWPR** chips, one per motor,
gaining electrical current feedback. The rest of the software design is
unchanged — only the driver layer and stall-detection strategy differ.

### Why the change

- **IPROPI current feedback (the headline feature)**: each DRV8876 outputs
  an analog current proportional to motor current via the IPROPI pin. We
  route that to an ESP32-C6 ADC pin and now have a direct electrical view
  into mechanical load — a stalling motor's current spikes long before its
  velocity collapses. Combined detection (`current > threshold` AND
  `measured_rpm < commanded`) is far more decisive than velocity-only.
- **Built-in current limit**: IPROPI can also drive an *input* threshold
  to hard-clamp peak current in hardware. Calibration's intentional-stall
  step becomes safer — we never push past the chip's regulated ceiling
  even if a wire frays or a gear seizes.
- **One chip per motor**: shorter motor traces, simpler board layout, no
  shared substrate heating between channels under load.
- **nFAULT pin**: dedicated hardware fault output we can OR between the
  two chips or wire each separately — instant signal for over-temp,
  over-current, under-voltage, without needing to poll.
- **Voltage headroom**: DRV8876 handles up to 37 V vs TB6612's 13.5 V.
  Doesn't matter for our current motor but doesn't hurt.

### Driver mode: PH/EN

DRV8876 supports two input-control modes selected by the `PMODE` pin:

- **PH/EN mode**: one direction pin (`PH`) + one PWM'd enable (`EN`). The key
  detail an earlier draft of this doc missed: in PH/EN, `EN`=0 *with the
  driver awake* is **low-side slow-decay brake**, not coast — and PWM'ing `EN`
  toggles between drive and that brake. So PH/EN does give us the active brake
  the spring-less blind needs.
- **IN1/IN2 mode**: same shape as the TB6612 (either input carries PWM, both
  high = brake, both low = coast).

The board straps `PMODE` → **GND = PH/EN mode**. (The earlier draft planned
IN1/IN2 for braking; the as-built board uses PH/EN, which gives the same hold
behavior with one fewer concept: `PH` = direction, `EN` = speed, `EN`=0 =
brake.) Coast is reached only by taking the shared `nSLEEP` low, which coasts
both motors — never used mid-travel.

This means **one LEDC channel per motor** (on `EN`); `PH` is a plain GPIO. The
C6's LEDC channels comfortably cover 2 × `EN` + the user LED (ch 0 = LED, ch 1
= L `EN`, ch 2 = R `EN`).

### Pin allocation

See the **Pin allocation** table near the top — it is the as-built PCB
netlist. In short: dropping the breadboard's dedicated `PWMA`/`PWMB` pins (PWM
now rides on `EN`) freed room for the two `IPROPI` ADC lines (D0/D1) and the
shared `nSLEEP` enable (D2). `nFAULT` is pulled up on the board but **not**
wired to the MCU (no spare GPIO). Still fits on the 11 front-side header pins.

### `current_sense` component

```
components/current_sense/
├── include/blinds/current_sense.hpp
└── src/current_sense.cpp
```

Public API (implemented):
- `start()` — configure the ADC1 oneshot channels + curve-fitting calibration
  and kick off a 100 Hz sampling task
- `current_ma(Side)` — last sampled current draw in milliamps
- `voltage_mv(Side)` — last raw `IPROPI` voltage in millivolts (diagnostic)
- posts `Event::OvercurrentL/R` on the component's `EVENTS` bus when a motor's
  current stays above the fault threshold (~2800 mA) for >50 ms, so the motion
  task can react without polling

Conversion (DRV8876 datasheet §7.3.3.1, verified against the spec):
`V_IPROPI = I_OUT × A_IPROPI × R_IPROPI = I_OUT × 1000 µA/A × 560 Ω =
I_OUT × 0.56 V/A`, so `I_OUT(mA) = V_IPROPI(mV) × 1000 / 560`. In our
drive↔brake PWM the low-side FET conducts continuously, so `IPROPI` is valid
for asynchronous sampling (no PWM sync needed); it reads ~0 only in coast/Hi-Z.

Sampling at 100 Hz (same cadence as the speed loop) gives tight synchronization
between commanded duty, measured RPM, and measured current — the three signals
we need for confident stall detection.

### Updated stall detection

The breadboard's velocity-only approach: `measured_rpm < threshold` for a
sustained duration. Works but slow — stall current rises immediately while
RPM takes time to coast down.

The DRV8876's `IPROPI` lets us flip to current-first:

```
if motor.current_ma > STALL_CURRENT_MA
    AND motor.measured_rpm < motor.commanded_rpm × STALL_RPM_FRAC
    AND condition_held_for >= STALL_DURATION_MS:
        declare stall
```

Both signals required — current spike alone could just be a transient
load (cord catching briefly); RPM drop alone could be a coast-down at
zero command. Together they're unambiguous. `STALL_DURATION_MS` can drop
to ~50 ms (vs 200 ms needed for velocity-only) because both signals
respond faster.

This also tightens up the **top calibration**: we no longer need to drive
to a hard mechanical stop. As soon as the cord goes taut and current
crosses the threshold, we stop. Less stressful on motor, gearbox, and
cord.

### Migration status

Board bring-up is **done** and verified on hardware — single-motor bench tests
on each side (drive, encoder, current sense) plus a 0→100% duty `ramp` capture
showing clean position/velocity/acceleration and current draw (L and R within
~1% of each other):

1. ✅ `current_sense` component (ADC1 oneshot + 100 Hz sampling task).
2. ✅ `motor.cpp` reworked for **PH/EN** mode: `EN` on LEDC, `PH` direction,
   shared `nSLEEP` enable driven high at boot.
3. ⬜ nFAULT handler — **N/A**: `nFAULT` isn't routed to the MCU on this board
   (no spare GPIO); the driver auto-retries faults and we infer stall from
   `IPROPI`.
4. ⬜ Combined current+RPM stall detection in `motion`/`calibration` — still
   velocity-only; current-first is the next step now that the signal is live.
5. ⬜ Re-run the duty/RPM/current sweeps **under cord load** to recalibrate the
   feedforward and stall thresholds (no-load top speed measured ~309/311 RPM).

## Open questions

- ~~**Motor voltage / buck topology**~~: resolved — the board runs from 12 V
  with an AP63205 fixed-5 V buck into the XIAO `VBUS`. (Confirm the chosen
  motors are happy at 12 V `VM`.)
- **Gearbox backdrive resistance**: with the factory spring removed, the
  gearbox + short-brake is the only thing keeping the blind from
  free-falling. Need to confirm the chosen gearmotor's ratio is high enough
  that string tension from the blind weight can't backdrive it. Hang the
  fully-extended blind on it with the motor unpowered and see if it holds.
- **WindowCoveringType reporting**: ZCL doesn't have a "cellular" enum value.
  Defaulting to `Rollershade` (0x00) since that's what most cellular-shade
  motorization products (SwitchBot, IKEA Fyrtur, etc.) report, and it gets
  us the right cover entity in Home Assistant. `Drapery` is a plausible
  alternative; revisit if a hub renders the wrong icon.
- **Easing default choice**: cosine vs spring. Won't know which feels right
  until we have a real blind moving — pick after a quick A/B with both
  implemented.
- **Do we want manual wall control later?** A physical button board on the
  wall pairing with this device via Zigbee binding (no hub) is a clean
  fallback. Not in v1, but worth keeping the option open.
