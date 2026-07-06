// Parsing + field metadata for the device `config` dump.
//
// Each line looks like:  motion.kp           = 0.100

export interface ConfigField {
  key: string;
  label: string;
  editable: boolean;
  // Hint for the <input>; net.* are strings shown read-only.
  kind: "number" | "string";
  // One-line explanation shown under the label.
  description: string;
  // When true, the field holds an absolute position in mm (below the homed
  // top), so the form offers a button to capture the current measured position
  // into it — drive the blind to the limit, then click to record it.
  setFromPosition?: boolean;
}

// A group of fields. Top-level sections may either hold fields directly or be
// split into subsections (used by "Motion Tuning"). Each group can carry a
// short blurb explaining what the group is for.
export interface ConfigGroup {
  title: string;
  description?: string;
  fields: ConfigField[];
}

export interface ConfigSection {
  title: string;
  description?: string;
  // Exactly one of these is set: a flat list of fields, or named subsections.
  fields?: ConfigField[];
  groups?: ConfigGroup[];
}

// Order here is the order shown in the form. net.pass is read-only (the device
// only ever reports "(set)"/"(unset)", never the real value).
export const SECTIONS: ConfigSection[] = [
  {
    title: "Device",
    description: "Identity shown on network and Zigbee surfaces after reboot.",
    fields: [
      {
        key: "device.location",
        label: "Location",
        editable: true,
        kind: "string",
        description:
          "Human label for this unit, e.g. Bedroom Right. Used for the WiFi hostname and Zigbee LocationDescription after reboot.",
      },
    ],
  },
  {
    title: "Travel & Limits",
    description: "Spool calibration and the travel range a move is allowed to cover.",
    fields: [
      {
        key: "motion.mm_per_rev",
        label: "mm per revolution",
        editable: true,
        kind: "number",
        description:
          "Cord travel (mm) per output revolution — the spool calibration mapping counts to mm. Required for go-to-position. Measure: home, drive N revs, measure the drop, divide.",
      },
      {
        key: "motion.hard_stop_mm",
        label: "Hard stop (mm)",
        editable: true,
        kind: "number",
        setFromPosition: true,
        description: "Full downward travel from the top, in mm. 'Go to 100%' maps to this, and downward motion never passes it.",
      },
      {
        key: "motion.soft_stop_mm",
        label: "Soft stop (mm)",
        editable: true,
        kind: "number",
        setFromPosition: true,
        description:
          "Optional down limit for a blind with an obstruction below — downward motion stops here instead of the hard stop. 0 = unset.",
      },
      {
        key: "motion.cover_rpm",
        label: "Cover speed (RPM)",
        editable: true,
        kind: "number",
        description: "Open/close speed for the Zigbee cover commands, in output-shaft RPM.",
      },
    ],
  },
  {
    title: "WiFi",
    description: "Debug-mode network. Set over USB/serial; shown read-only here.",
    fields: [
      {
        key: "net.ssid",
        label: "WiFi SSID",
        editable: false,
        kind: "string",
        description: "Network joined in debug mode. Set over USB/serial; read-only here.",
      },
      {
        key: "net.pass",
        label: "WiFi password",
        editable: false,
        kind: "string",
        description: "Stored on the device; it only ever reports whether one is set, never the value.",
      },
      {
        key: "net.conn_to",
        label: "WiFi connect timeout (s)",
        editable: false,
        kind: "string",
        description: "How long debug mode waits to associate before rebooting back to normal Zigbee mode.",
      },
    ],
  },
  {
    title: "Motion Tuning",
    description: "The speed-control loop and its safety watchdogs. Tweak with care.",
    groups: [
      {
        title: "Feedforward",
        description: "Open-loop duty the controller commands before the feedback loop trims it.",
        fields: [
          {
            key: "motion.duty_per_rpm",
            label: "Feedforward duty / RPM",
            editable: true,
            kind: "number",
            description:
              "Feedforward slope: % PWM duty per RPM of target speed. Carries the bulk of the command so the integrator only trims.",
          },
          {
            key: "motion.ff_off_raise",
            label: "FF offset raise (%)",
            editable: true,
            kind: "number",
            description:
              "Static feedforward duty added when raising, to overcome breakaway friction + gravity (~10% measured). Lets the integrator start near the right duty.",
          },
          {
            key: "motion.ff_off_lower",
            label: "FF offset lower (%)",
            editable: true,
            kind: "number",
            description: "Static feedforward duty added when lowering. ~0 since gravity already assists.",
          },
          {
            key: "motion.ff_trim_l",
            label: "FF trim — left",
            editable: true,
            kind: "number",
            description:
              "Feedforward multiplier for the left motor. The two motors aren't identical; trim the faster side down (e.g. 0.95) so both hit the target speed and the loop stops hunting. 1.0 = no trim.",
          },
          {
            key: "motion.ff_trim_r",
            label: "FF trim — right",
            editable: true,
            kind: "number",
            description:
              "Feedforward multiplier for the right motor. The two motors aren't identical; trim the faster side down (e.g. 0.95) so both hit the target speed and the loop stops hunting. 1.0 = no trim.",
          },
        ],
      },
      {
        title: "Speed loop (PID)",
        description: "Feedback gains that close the loop on measured RPM.",
        fields: [
          {
            key: "motion.kp",
            label: "Kp (duty / RPM error)",
            editable: true,
            kind: "number",
            description: "Proportional gain: % duty added per RPM of speed error. The immediate response.",
          },
          {
            key: "motion.ki",
            label: "Ki (duty / RPM·s)",
            editable: true,
            kind: "number",
            description:
              "Integral gain: % duty per RPM-second of accumulated error. Removes steady-state offset; too high overshoots (the 'fast then slow' surge).",
          },
          {
            key: "motion.i_max",
            label: "Integrator clamp",
            editable: true,
            kind: "number",
            description: "Anti-windup clamp on the integrator (RPM·s). Bounds how much duty the integral term can add.",
          },
        ],
      },
      {
        title: "Motor sync",
        description: "Keeps the two motors in step and faults if they drift apart.",
        fields: [
          {
            key: "motion.k_sync",
            label: "Sync gain (RPM / count)",
            editable: true,
            kind: "number",
            description:
              "Cross-coupling: RPM bias added per encoder count one side lags the other, to keep the two in step. Set 0 to tune each motor independently.",
          },
          {
            key: "motion.sync_fault",
            label: "Sync fault limit (counts)",
            editable: true,
            kind: "number",
            description:
              "Fault and brake if the two encoders diverge by more than this many counts while driving (~896 counts = one output revolution).",
          },
        ],
      },
      {
        title: "Stall detection",
        description: "Watchdog that brakes the motors if they stop while commanded to move.",
        fields: [
          {
            key: "motion.stall_delta",
            label: "Stall Δcount max",
            editable: true,
            kind: "number",
            description: "Per-tick |Δcount| at or below this is treated as 'not moving' (absorbs encoder jitter).",
          },
          {
            key: "motion.stall_ms",
            label: "Stall fault (ms)",
            editable: true,
            kind: "number",
            description: "Both motors must read stopped this long, while commanded to move, before a stall fault fires.",
          },
          {
            key: "motion.grace_ms",
            label: "Startup grace (ms)",
            editable: true,
            kind: "number",
            description: "Stall watchdog is suppressed this long after motion starts, covering stiction and the integrator ramp-up.",
          },
        ],
      },
      {
        title: "Homing",
        description: "The open-loop run that drives both motors to the top stop and zeroes the encoders.",
        fields: [
          {
            key: "motion.home_duty",
            label: "Home duty (%)",
            editable: true,
            kind: "number",
            description: "Open-loop duty each motor is driven upward at while homing — high enough to move immediately.",
          },
          {
            key: "motion.home_settle",
            label: "Home settle (ms)",
            editable: true,
            kind: "number",
            description:
              "A motor is 'homed' once its encoder has been stopped this long (wedged at the top); its encoder is then zeroed.",
          },
          {
            key: "motion.home_to",
            label: "Home timeout (s)",
            editable: true,
            kind: "number",
            description: "Safety cap on a homing run: aborts and brakes if a motor never settles.",
          },
        ],
      },
      {
        title: "Go-to position",
        description: "The approach profile used when seeking a target position.",
        fields: [
          {
            key: "motion.goto_slow_mm",
            label: "Go-to slow zone (mm)",
            editable: true,
            kind: "number",
            description: "How far before the target a go-to starts ramping speed down for a soft landing.",
          },
          {
            key: "motion.goto_min_rpm",
            label: "Go-to floor speed (RPM)",
            editable: true,
            kind: "number",
            description: "Minimum speed during the go-to approach, so it keeps creeping in rather than stalling short of target.",
          },
          {
            key: "motion.goto_tol_mm",
            label: "Go-to tolerance (mm)",
            editable: true,
            kind: "number",
            description: "Arrival tolerance — the go-to brakes once within this distance of the target.",
          },
        ],
      },
    ],
  },
];

export type ConfigValues = Record<string, string>;

// Parse a `config` response into a key→value map. Tolerates extra whitespace
// and ignores lines that aren't `key = value`.
export function parseConfig(text: string): ConfigValues {
  const out: ConfigValues = {};
  for (const raw of text.split("\n")) {
    const line = raw.trim();
    const eq = line.indexOf("=");
    if (eq < 0) continue;
    const key = line.slice(0, eq).trim();
    const value = line.slice(eq + 1).trim();
    if (key) out[key] = key === "device.location" && value === "(unset)" ? "" : value;
  }
  return out;
}
