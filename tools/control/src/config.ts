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
}

// Order here is the order shown in the form. net.pass is read-only (the device
// only ever reports "(set)"/"(unset)", never the real value).
export const FIELDS: ConfigField[] = [
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
  {
    key: "motion.k_sync",
    label: "Sync gain (RPM / count)",
    editable: true,
    kind: "number",
    description:
      "Cross-coupling: RPM bias added per encoder count one side lags the other, to keep the two in step. Set 0 to tune each motor independently.",
  },
  {
    key: "motion.cover_rpm",
    label: "Cover speed (RPM)",
    editable: true,
    kind: "number",
    description: "Open/close speed for the Zigbee cover commands, in output-shaft RPM.",
  },
  {
    key: "motion.sync_fault",
    label: "Sync fault limit (counts)",
    editable: true,
    kind: "number",
    description:
      "Fault and brake if the two encoders diverge by more than this many counts while driving (~896 counts = one output revolution).",
  },
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
    description: "Full downward travel from the top, in mm. 'Go to 100%' maps to this, and downward motion never passes it.",
  },
  {
    key: "motion.soft_stop_mm",
    label: "Soft stop (mm)",
    editable: true,
    kind: "number",
    description:
      "Optional down limit for a blind with an obstruction below — downward motion stops here instead of the hard stop. 0 = unset.",
  },
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
    if (key) out[key] = value;
  }
  return out;
}
