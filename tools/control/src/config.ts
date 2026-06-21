// Parsing + field metadata for the device `config` dump.
//
// Each line looks like:  motion.kp           = 0.100

export interface ConfigField {
  key: string;
  label: string;
  editable: boolean;
  // Hint for the <input>; net.* are strings shown read-only.
  kind: "number" | "string";
}

// Order here is the order shown in the form. net.pass is read-only (the device
// only ever reports "(set)"/"(unset)", never the real value).
export const FIELDS: ConfigField[] = [
  { key: "motion.duty_per_rpm", label: "Feedforward duty / RPM", editable: true, kind: "number" },
  { key: "motion.kp", label: "Kp (duty / RPM error)", editable: true, kind: "number" },
  { key: "motion.ki", label: "Ki (duty / RPM·s)", editable: true, kind: "number" },
  { key: "motion.i_max", label: "Integrator clamp", editable: true, kind: "number" },
  { key: "motion.k_sync", label: "Sync gain (RPM / count)", editable: true, kind: "number" },
  { key: "motion.cover_rpm", label: "Cover speed (RPM)", editable: true, kind: "number" },
  { key: "motion.sync_fault", label: "Sync fault limit (counts)", editable: true, kind: "number" },
  { key: "motion.stall_delta", label: "Stall Δcount max", editable: true, kind: "number" },
  { key: "motion.stall_ms", label: "Stall fault (ms)", editable: true, kind: "number" },
  { key: "motion.grace_ms", label: "Startup grace (ms)", editable: true, kind: "number" },
  { key: "net.ssid", label: "WiFi SSID", editable: false, kind: "string" },
  { key: "net.pass", label: "WiFi password", editable: false, kind: "string" },
  { key: "net.conn_to", label: "WiFi connect timeout (s)", editable: false, kind: "string" },
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
