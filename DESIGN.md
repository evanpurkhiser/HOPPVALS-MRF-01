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
- Be safe: never run a motor into a hard mechanical stop; detect stalls and
  cut power.

## Non-goals (for v1)

- Battery operation. Blinds are line-powered, so we run as a Zigbee Router
  (not a sleepy End Device). This gives us better mesh behavior for free.
- OTA firmware updates. The partition table already reserves space for it;
  we'll enable it later.
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
- Two limit switches (top and bottom of carriage travel).

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
| TB6612FNG dual H-bridge | PWM motor driver (one IC drives both motors) |
| 4× microswitches | Hard endstops (top + bottom per side) |
| External DC supply (TBD V) | Motor power (`VM`) |
| LDO or buck → 5V | Logic power into XIAO USB-C / VIN |

### Why TB6612FNG

It's the right size for the job — dual H-bridge in one package, ~1.2 A
continuous per channel (3.2 A peak), 2.5–13.5 V motor supply, and the input
side is 3.3 V-tolerant so the XIAO can drive it directly without level
shifters. Has an `STBY` pin so we can fully de-energize the H-bridge when
idle.

Critically for our use case (no spring, motor must hold position): the
TB6612FNG supports **short-brake** mode by driving both inputs high. This
shorts the motor terminals together so any rotation generates a back-EMF
current that resists the motion — dynamic braking. We use this whenever the
blind is at rest at a non-endpoint position. Combined with a high-ratio
gearbox, the blind can't drift down on its own.

Driver state truth table:

| `INx1` | `INx2` | `PWMx` | Mode |
|---|---|---|---|
| H | L | PWM | Forward (raise blind) |
| L | H | PWM | Reverse (lower blind) |
| H | H | x | **Short brake** (hold against gravity) |
| L | L | x | Coast (motor freewheels — never use mid-travel) |
| x | x | x | All off when `STBY` low |

## Power architecture

```
USB-C (5V dev)  ─┐
                 ├─► XIAO 5V rail ─► onboard 3V3 LDO ─► logic
                 │
External DC ─────┴─► buck/LDO ─────────────► XIAO 5V rail
            │
            └─► VM on TB6612FNG ──────────► motors
```

Two domains:
- **Logic (3.3 V)**: MCU, encoders, limit switches, TB6612FNG `VCC`.
- **Motor (VM)**: only the H-bridge outputs. Common ground with logic.

The motor supply voltage gets picked once we know the motor's rated voltage.
Buck regulator to give us 5 V for the XIAO is overkill if VM is already 5 V —
but most DC blind motors run at 6–12 V so we'll likely need a separate buck
down to 5 V for the MCU.

### Decoupling

Heavy bulk cap on `VM` (≥220 µF electrolytic + 0.1 µF ceramic) near the
TB6612FNG. Motor inrush + PWM switching transients can dip VM hard enough to
brown out the MCU if logic and motor share a regulator without isolation.

## Pin allocation (XIAO ESP32-C6, 15 usable GPIOs)

| XIAO pin | GPIO | Use | Peripheral |
|---|---|---|---|
| D0 | GPIO0 | LIMIT_L_TOP | GPIO IN, interrupt |
| D1 | GPIO1 | LIMIT_L_BOTTOM | GPIO IN, interrupt |
| D2 | GPIO2 | LIMIT_R_TOP | GPIO IN, interrupt |
| D3 | GPIO21 | LIMIT_R_BOTTOM | GPIO IN, interrupt |
| D4 | GPIO22 | ENCODER_L_A | PCNT unit 0 ch 0 |
| D5 | GPIO23 | ENCODER_L_B | PCNT unit 0 ch 1 |
| D6 | GPIO16 | ENCODER_R_A | PCNT unit 1 ch 0 |
| D7 | GPIO17 | ENCODER_R_B | PCNT unit 1 ch 1 |
| D8 | GPIO19 | MOTOR_L_PWM (PWMA) | LEDC ch 0 |
| D9 | GPIO20 | MOTOR_R_PWM (PWMB) | LEDC ch 1 |
| D10 | GPIO18 | MOTOR_L_IN1 (AIN1) | GPIO OUT |
| MTDO | GPIO7 | MOTOR_L_IN2 (AIN2) | GPIO OUT |
| MTDI | GPIO5 | MOTOR_R_IN1 (BIN1) | GPIO OUT |
| MTCK | GPIO6 | MOTOR_R_IN2 (BIN2) | GPIO OUT |
| MTMS | GPIO4 | TB6612 STBY | GPIO OUT |

**Reserved by board:** GPIO3 (RF switch), GPIO9 (BOOT), GPIO14 (antenna sel),
GPIO15 (user LED).

Tight but workable. If we end up wanting current sense, manual buttons, or
a status LED beyond the onboard one, we'd have to multiplex something or
add an I/O expander on the I2C lines (D4/D5) — but that means giving up an
encoder, which we don't want.

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
│ │ZCL   │ │  │ │ trajectory  │  │  │ │limit sw │ │
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

ISR work is minimal: limit switches just set a flag and notify the motion
task. PCNT decoding happens in hardware.

### Module layout

```
main/
├── blinds.c            app_main, wiring
├── zigbee.c/h          Endpoint + cluster registration, ZCL handlers
├── motion.c/h          Motion controller (trajectory + control loop)
├── motor.c/h           Single-motor abstraction (PWM + direction + encoder)
├── limits.c/h          Limit-switch debouncing + interrupt routing
├── calibration.c/h     Find-endstops routine
├── persistence.c/h     NVS save/load of calibration + last position
└── config.h            Pin map, tuning constants
```

## Motion control

### Coordinate convention

Internally we work in **encoder counts**, with 0 = fully open (top) and
`max_counts` = fully closed (bottom). We expose this to Zigbee as a 0–100
percentage where 0 = open, 100 = closed (matching the ZCL spec — yes, this
is counterintuitive).

### Trajectory generation

Open-loop "PWM at full duty until you get there" gives a violent start and
overshoots the target. Instead, we generate a trapezoidal velocity profile:

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

Tuning parameters (all in `config.h`, easy to adjust):
- `MOTION_V_MAX` — full-speed duty cycle (0..255 for 8-bit PWM).
- `MOTION_V_MIN_START` — minimum duty to overcome static friction (motors
  won't move below this — they just whine).
- `MOTION_ACCEL` — counts/sec². How aggressively we ramp up.
- `MOTION_DECEL_BAND` — encoder counts before target where we start ramping
  down.

For tiny moves (target within `2 × DECEL_BAND` counts) we use a triangular
profile — accel directly into decel, no cruise phase.

If S-curve smoothness becomes a goal later, the trapezoid generator gets
replaced with a 7-segment S-curve. Trapezoid is good enough for blinds.

### Per-motor closed loop

Each motor has its own simple P (or PI) controller running at 100 Hz:

```
error = setpoint_velocity - measured_velocity
duty  = clamp(Kp * error + integral_term, 0, V_MAX)
```

Where `setpoint_velocity` comes from the trajectory generator and
`measured_velocity` from PCNT count delta over the last tick. The direction
GPIOs are set from `sign(setpoint)`.

### Two-motor synchronization

This is the interesting part. Two strategies:

**(A) Position-sync (chosen).** Both motors share a single trajectory
generator producing the *target position* at each tick. Each motor's local
controller drives toward that target. If one motor lags, the other doesn't
race ahead — they're both servoing to the same point.

**(B) Master/slave velocity.** Master motor runs at commanded velocity; slave
runs at master's *measured* velocity. Simpler but tilts the blind if the
master stalls briefly.

We go with (A) plus a sync-error watchdog: if `|encoder_L - encoder_R|` ever
exceeds `SYNC_ERROR_MAX` counts, we stop both motors and report a fault.

### Stall detection

If the trajectory says we should be moving but `|Δcounts|/Δt` is below
`STALL_VELOCITY_MIN` for `STALL_TIMEOUT_MS`, we cut PWM, set `STBY` low, and
report a fault via Zigbee. Recovery requires a re-calibration or a hub
command.

## Limit switches

Two layers:

- **Soft limits**: encoder positions learned at calibration. The motion
  controller refuses moves outside the soft range. This is the normal
  protection.
- **Hard limits**: the physical microswitches. If the encoder loses sync
  with reality (slippage, miscount) and we drive past the soft limit, the
  switch interrupt fires and we hard-stop instantly.

Hitting a hard limit during normal operation is a **fault** — it means
calibration is wrong. We log it and report `ConfigStatus` with a fault flag
set, and require re-calibration.

Hitting a hard limit during *calibration* is **expected** — that's how
calibration finds the endstops.

Switches are wired active-low to GND with internal pullups (`GPIO_PULLUP_ONLY`).
Software debounce: 5 ms. (Mechanical switches bounce for ~1–2 ms typically.)

## Calibration

Triggered by writing `Mode.CalibrationMode = 1` from the hub, or holding the
BOOT button at startup.

Sequence:
1. Drive both motors **up** at slow speed (`MOTION_V_CALIBRATION`) until
   both top limit switches fire (or timeout → fault).
2. Stop. Zero both encoders. This is the **open** endpoint.
3. Drive both motors **down** at slow speed until both bottom limit switches
   fire (or timeout).
4. Record final encoder counts as `max_counts_L`, `max_counts_R`. These
   should be within 1% of each other; if not, the spools aren't matched and
   we fault.
5. Drive back up to ~50% as a "calibrated" sanity check, then stop.
6. Persist `max_counts` to NVS. Clear `Mode.CalibrationMode`.

The first power-on after flashing forces calibration before the device will
accept move commands.

## Persistence

NVS namespace `blinds`. Keys:
- `cal_max_l`, `cal_max_r` — calibrated max encoder counts per side.
- `pos_last` — last known position percentage (0–100).
- `cal_done` — bool; if false, force calibration.
- `mode` — current Window Covering `Mode` bitmap (motor reversed, LED feedback).

We write `pos_last` only on **stop** (not every tick — that'd burn out the
flash in a week). Flash is rated for ~100k writes per sector; this gives
us decades.

On boot, we load `pos_last` and assume the blind is still there (no way to
know without limit switches firing). If the assumption is wrong, the user
sees the blind at the wrong position in their hub and triggers
recalibration.

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
                  stall/sync│error / hard limit hit
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
1. **Hard limit hit** — instant stop (short-brake, then STBY if at endpoint).
2. **Sync error** — instant stop, fault.
3. **Stall detected** — stop, fault.
4. **Soft limit reached** — clean stop at target.
5. **Trajectory complete** — clean stop at target.

### Rest-state policy

When motion completes:
- If target is fully open (0%) or fully closed (100%) **and** a hard limit
  switch confirms it: `STBY` low (driver off, zero power). The mechanical
  endstop holds the blind.
- Otherwise (any intermediate position): both inputs high → short-brake mode.
  Driver dissipates a tiny amount of standby current; motor terminals are
  shorted so the gearbox + brake together prevent drift.

## Manual operation (debug)

The XIAO BOOT button (GPIO9) is repurposed as a manual override at runtime:
- Single tap: stop / start trajectory toggle.
- Double tap: full open.
- Triple tap: full close.
- Hold 3 sec at boot: force calibration.

Useful for testing without a hub paired, and as a fallback if the hub is
unreachable.

## Open questions

- **Motor voltage**: depends on which DC motors we land on. Need to know
  before picking the buck regulator topology.
- **Gearbox backdrive resistance**: with the factory spring removed, the
  gearbox + short-brake is the only thing keeping the blind from
  free-falling. Need to confirm the chosen gearmotor's ratio is high enough
  that string tension from the blind weight can't backdrive it (or that the
  worm/lead geometry makes it non-backdriveable by design). Hang the
  fully-extended blind on it with the motor unpowered and see if it holds.
- **Encoder PPR (pulses per revolution)** and drum circumference: these
  set our position resolution. A 12 PPR encoder × 100:1 gearbox = 4800 cpr;
  if one drum revolution = 30 mm of cord travel and the full blind drop is
  1500 mm, that's 240k counts — well within `uint32_t`.
- **Limit-switch placement**: physical detail for the mechanical design.
  Activation needs to happen *before* the carriage hits a hard mechanical
  stop, ideally with ~5 mm of overtravel margin so a brief over-run is OK.
- **WindowCoveringType reporting**: ZCL doesn't have a "cellular" enum value.
  Defaulting to `Rollershade` (0x00) since that's what most cellular-shade
  motorization products (SwitchBot, IKEA Fyrtur, etc.) report, and it gets
  us the right cover entity in Home Assistant. `Drapery` is a plausible
  alternative; revisit if a hub renders the wrong icon.
- **Do we want manual wall control later?** A physical button board on the
  wall pairing with this device via Zigbee binding (no hub) is a clean
  fallback. Not in v1, but worth keeping the option open.
