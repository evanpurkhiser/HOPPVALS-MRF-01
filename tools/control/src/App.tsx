import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  DeviceClient,
  parsePosition,
  type LogEntry,
  type Position,
  type Status,
} from "./device.ts";
import {
  SECTIONS,
  parseConfig,
  type ConfigField,
  type ConfigValues,
} from "./config.ts";

const MOVE_KEYS_UP = new Set(["ArrowUp", "w", "W"]);
const MOVE_KEYS_DOWN = new Set(["ArrowDown", "s", "S"]);
const FAST_MOVE_MULTIPLIER = 2;

export default function App() {
  const clientRef = useRef<DeviceClient | null>(null);
  if (clientRef.current === null) clientRef.current = new DeviceClient();
  const client = clientRef.current;

  const [ip, setIp] = useState("10.0.0.135");
  const [status, setStatus] = useState<Status>("disconnected");
  const [log, setLog] = useState<LogEntry[]>([]);
  const [live, setLive] = useState<ConfigValues>({});
  const [edits, setEdits] = useState<ConfigValues>({});
  const [rpm, setRpm] = useState(40);
  const [moving, setMoving] = useState<"raise" | "lower" | null>(null);
  const [homing, setHoming] = useState(false);
  const [pct, setPct] = useState(50);
  const [seeking, setSeeking] = useState(false);
  const [raw, setRaw] = useState("");
  const [position, setPosition] = useState<Position | null>(null);
  const [dark, setDark] = useState(() =>
    document.documentElement.classList.contains("dark"),
  );

  const toggleTheme = useCallback(() => {
    setDark((prev) => {
      const next = !prev;
      document.documentElement.classList.toggle("dark", next);
      try {
        localStorage.setItem("theme", next ? "dark" : "light");
      } catch {
        /* ignore */
      }
      return next;
    });
  }, []);

  // Wire client callbacks once.
  useEffect(() => {
    client.onStatus = setStatus;
    client.onLog = (e) => setLog((prev) => [...prev.slice(-299), e]);
    client.onPosition = setPosition;
  }, [client]);

  const refreshConfig = useCallback(async () => {
    try {
      const text = await client.send("config");
      setLive(parseConfig(text));
      setEdits({});
    } catch {
      /* logged by client */
    }
  }, [client]);

  const connect = useCallback(() => {
    client.connect(ip.trim());
  }, [client, ip]);

  const disconnect = useCallback(() => {
    client.stop();
    client.disconnect();
  }, [client]);

  // On connect, pull config and the current position.
  useEffect(() => {
    if (status !== "connected") return;
    void refreshConfig();
    client.send("pos").catch(() => {});
  }, [status, refreshConfig, client]);

  // ── Motion ────────────────────────────────────────────────────────────
  const stop = useCallback(() => {
    client.stop();
    setMoving(null);
    setSeeking(false);
  }, [client]);

  const move = useCallback(
    (dir: "raise" | "lower", multiplier = 1) => {
      if (!client.isOpen()) return;
      const r = Math.max(1, Math.min(300, Math.round(rpm * multiplier)));
      client.send(`motion ${r} ${dir}`).catch(() => {});
      setMoving(dir);
    },
    [client, rpm],
  );

  // Home drives both motors up to the top stop; it blocks on the device until
  // both settle (or its timeout), so allow well beyond the firmware's cap.
  const home = useCallback(() => {
    if (!client.isOpen() || homing) return;
    setHoming(true);
    client
      .send("home", 40000)
      .catch(() => {})
      .finally(() => {
        setHoming(false);
        client.send("pos").catch(() => {}); // refresh position at the new zero
      });
  }, [client, homing]);

  // Go to a position as % of full travel, at the RPM from the Motion panel.
  // Fire-and-forget on the device (the move runs in the control task), so the
  // command returns promptly. `seeking` stays set until the position poll below
  // sees the controller go idle; a rejected command clears it right away.
  const gotoPct = useCallback(() => {
    if (!client.isOpen() || seeking || homing) return;
    const p = Math.max(0, Math.min(100, Math.round(pct)));
    const r = Math.max(1, Math.min(300, Math.round(rpm)));
    setSeeking(true);
    client
      .send(`gotopct ${p} ${r}`)
      .then((resp) => {
        if (/rejected/i.test(resp)) setSeeking(false);
      })
      .catch(() => setSeeking(false));
  }, [client, pct, rpm, seeking, homing]);

  // While seeking a position, poll `pos` until the controller reports it's no
  // longer moving. Each reply also refreshes the live readout via onPosition.
  useEffect(() => {
    if (status !== "connected" || !seeking) return;

    let cancelled = false;
    let idleReads = 0;
    const startedAt = Date.now();

    const id = setInterval(async () => {
      try {
        const text = await client.send("pos");
        if (cancelled) return;
        const p = parsePosition(text);
        if (!p) return;
        if (p.moving) {
          idleReads = 0;
          return;
        }
        // Ignore the brief window before the move registers, then require two
        // idle reads so a mid-move sample dip doesn't end the seek early.
        idleReads += 1;
        if (Date.now() - startedAt > 500 && idleReads >= 2) setSeeking(false);
      } catch {
        /* logged by client */
      }
    }, 350);

    const backstop = setTimeout(() => setSeeking(false), 90000);
    return () => {
      cancelled = true;
      clearInterval(id);
      clearTimeout(backstop);
    };
  }, [client, seeking, status]);

  // Keyboard hold-to-move. The pressed set guards against keydown auto-repeat.
  useEffect(() => {
    const pressed = new Set<string>();

    const currentDirection = () => {
      for (const key of pressed) {
        if (MOVE_KEYS_UP.has(key)) return "raise";
        if (MOVE_KEYS_DOWN.has(key)) return "lower";
      }

      return null;
    };

    const isTypingTarget = (t: EventTarget | null) =>
      t instanceof HTMLElement && (t.tagName === "INPUT" || t.tagName === "TEXTAREA");

    const onKeyDown = (e: KeyboardEvent) => {
      if (isTypingTarget(e.target)) return;
      if (e.key === "Shift") {
        const dir = currentDirection();
        if (dir) move(dir, FAST_MOVE_MULTIPLIER);
        return;
      }

      const up = MOVE_KEYS_UP.has(e.key);
      const down = MOVE_KEYS_DOWN.has(e.key);
      if (!up && !down) return;
      e.preventDefault();
      if (pressed.has(e.key)) return; // ignore auto-repeat
      pressed.add(e.key);
      move(up ? "raise" : "lower", e.shiftKey ? FAST_MOVE_MULTIPLIER : 1);
    };

    const onKeyUp = (e: KeyboardEvent) => {
      if (e.key === "Shift") {
        const dir = currentDirection();
        if (dir) move(dir);
        return;
      }

      if (!MOVE_KEYS_UP.has(e.key) && !MOVE_KEYS_DOWN.has(e.key)) return;
      pressed.delete(e.key);
      stop();
    };

    // Safety: brake on any focus/visibility loss so a held key can't be "stuck".
    const onBlur = () => {
      pressed.clear();
      stop();
    };
    const onVisibility = () => {
      if (document.visibilityState === "hidden") {
        pressed.clear();
        stop();
      }
    };

    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    window.addEventListener("blur", onBlur);
    document.addEventListener("visibilitychange", onVisibility);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      window.removeEventListener("blur", onBlur);
      document.removeEventListener("visibilitychange", onVisibility);
    };
  }, [move, stop]);

  // ── Config editing ────────────────────────────────────────────────────
  const setEdit = (key: string, value: string) =>
    setEdits((prev) => ({ ...prev, [key]: value }));

  const dirtyKeys = useMemo(
    () =>
      Object.keys(edits).filter(
        (k) => edits[k] !== undefined && edits[k] !== (live[k] ?? ""),
      ),
    [edits, live],
  );

  const save = useCallback(async () => {
    for (const key of dirtyKeys) {
      try {
        await client.send(`config set ${key} ${edits[key]}`);
      } catch {
        return;
      }
    }
    try {
      await client.send("config save");
    } catch {
      return;
    }
    await refreshConfig();
  }, [client, dirtyKeys, edits, refreshConfig]);

  const reload = useCallback(async () => {
    try {
      await client.send("config reset");
    } catch {
      /* ignore */
    }
    await refreshConfig();
  }, [client, refreshConfig]);

  const sendRaw = useCallback(() => {
    const cmd = raw.trim();
    if (!cmd) return;
    client.send(cmd).catch(() => {});
    setRaw("");
  }, [client, raw]);

  const connected = status === "connected";

  // % of full travel, using the in-form hard-stop value (edited or live).
  const hardStop = Number(
    edits["motion.hard_stop_mm"] ?? live["motion.hard_stop_mm"] ?? 0,
  );
  const posPct =
    position && position.valid && hardStop > 0
      ? Math.round(Math.min(100, Math.max(0, (position.mm / hardStop) * 100)))
      : null;

  return (
    <div className="app">
      <header className="masthead">
        <div className="eyebrow">hoppvals / hv-mrf-01 / control</div>
        <h1>Blind Controller</h1>
        <p className="sub">
          Tune, drive, and home the dual-motor blind over its WiFi debug-mode
          websocket. Gains apply live; the console mirrors every command.
        </p>
      </header>

      <div className={`toolbar conn-${status}`}>
        <div className="conn">
          <input
            value={ip}
            onChange={(e) => setIp(e.target.value)}
            placeholder="device IP"
            disabled={connected || status === "connecting"}
          />
          {connected || status === "connecting" ? (
            <button className="btn" onClick={disconnect}>
              Disconnect
            </button>
          ) : (
            <button className="btn btn-primary" onClick={connect}>
              Connect
            </button>
          )}
        </div>
        <span className="status-pill">
          <span className="status-dot" />
          {status}
        </span>
        <div className="spacer" />
        <button
          className="theme-toggle"
          onClick={toggleTheme}
          title="Toggle dark mode"
        >
          {dark ? "☀ Light" : "☾ Dark"}
        </button>
      </div>

      <main>
        <div className="work">
          <section className="panel motion">
            <div className="panel-head">
              <span className="panel-label">Motion</span>
              <div className="spacer" />
              <label className="field-inline">
                RPM
                <input
                  type="number"
                  min={1}
                  max={300}
                  value={rpm}
                  onChange={(e) => setRpm(Number(e.target.value))}
                />
              </label>
            </div>
            <div className="panel-body">
              <div className="motion-grid">
                <div className="controls">
                  <div className="move-grid">
                    <button
                      className={`move ${moving === "raise" ? "active" : ""}`}
                      disabled={!connected}
                      onPointerDown={(e) => {
                        e.preventDefault();
                        move("raise", e.shiftKey ? FAST_MOVE_MULTIPLIER : 1);
                      }}
                      onPointerUp={stop}
                      onPointerLeave={() => moving === "raise" && stop()}
                    >
                      ▲ Raise
                    </button>
                    <button
                      className={`move ${moving === "lower" ? "active" : ""}`}
                      disabled={!connected}
                      onPointerDown={(e) => {
                        e.preventDefault();
                        move("lower", e.shiftKey ? FAST_MOVE_MULTIPLIER : 1);
                      }}
                      onPointerUp={stop}
                      onPointerLeave={() => moving === "lower" && stop()}
                    >
                      ▼ Lower
                    </button>
                    <button className="move stop" onClick={stop}>
                      ■ STOP
                    </button>
                  </div>

                  <div className="row">
                    <button
                      className="btn grow"
                      disabled={!connected || homing}
                      onClick={home}
                    >
                      {homing ? "⌂ Homing…" : "⌂ Home"}
                    </button>
                    <label className="field-inline">
                      Go to %
                      <input
                        type="number"
                        min={0}
                        max={100}
                        value={pct}
                        onChange={(e) => setPct(Number(e.target.value))}
                      />
                    </label>
                    <button
                      className="btn btn-primary"
                      disabled={!connected || seeking || homing}
                      onClick={gotoPct}
                    >
                      {seeking ? "Going…" : "Go"}
                    </button>
                  </div>
                </div>

                <div className={`position ${position?.moving ? "moving" : ""}`}>
                  <div className="position-readout">
                    <div className="position-top">
                      <span className="plabel">Position</span>
                      <span className="pstate">
                        {position && position.valid
                          ? position.moving
                            ? "moving"
                            : "stopped"
                          : ""}
                      </span>
                    </div>
                    {position && position.valid ? (
                      <div className="pval">
                        <span className="pmm">{position.mm.toFixed(1)}</span>
                        <span className="punit">mm</span>
                        {posPct !== null && (
                          <span className="ppct">{posPct}%</span>
                        )}
                      </div>
                    ) : (
                      <div className="pval-muted">
                        {position ? "not homed — run Home" : "—"}
                      </div>
                    )}
                  </div>
                  {position && position.valid && posPct !== null && (
                    <div className="travel-vert">
                      <div className="vends">
                        <span>open</span>
                        <span>closed</span>
                      </div>
                      <div className="vtrack">
                        <div
                          className="vfill"
                          style={{ height: `${posPct}%` }}
                        />
                      </div>
                    </div>
                  )}
                </div>
              </div>

              <p className="hint">
                Hold <kbd>↑</kbd>/<kbd>W</kbd> to raise, <kbd>↓</kbd>/<kbd>S</kbd>{" "}
                to lower. Release to stop; buttons work with mouse/touch too.{" "}
                Hold <kbd>Shift</kbd> for 2x speed. {" "}
                <strong>Home</strong> drives both up to the top stop and zeroes
                the encoders. <strong>Go to %</strong> needs a prior home and a
                calibrated mm/rev; 100% = hard stop (clamped to soft stop).
              </p>
            </div>
          </section>

          <section className="panel config">
            <div className="panel-head">
              <span className="panel-label">Configuration</span>
              <div className="spacer" />
              <button
                className="btn btn-sm"
                disabled={!connected}
                onClick={refreshConfig}
              >
                Refresh
              </button>
            </div>
            <div className="panel-body">
              <div className="sections">
                {SECTIONS.map((section) => (
                  <section key={section.title} className="cfg-section">
                    <h3>{section.title}</h3>
                    {section.description && (
                      <p className="section-desc">{section.description}</p>
                    )}
                    {section.fields && (
                      <div className="fields">
                        {section.fields.map((f) => (
                          <Field
                            key={f.key}
                            field={f}
                            live={live}
                            edits={edits}
                            connected={connected}
                            position={position}
                            onEdit={setEdit}
                          />
                        ))}
                      </div>
                    )}
                    {section.groups?.map((group) => (
                      <div key={group.title} className="cfg-group">
                        <h4>{group.title}</h4>
                        {group.description && (
                          <p className="group-desc">{group.description}</p>
                        )}
                        <div className="fields">
                          {group.fields.map((f) => (
                            <Field
                              key={f.key}
                              field={f}
                              live={live}
                              edits={edits}
                              connected={connected}
                              position={position}
                              onEdit={setEdit}
                            />
                          ))}
                        </div>
                      </div>
                    ))}
                  </section>
                ))}
              </div>
              <div className="config-actions">
                <button
                  className="btn btn-primary"
                  disabled={!connected || dirtyKeys.length === 0}
                  onClick={save}
                >
                  Save{dirtyKeys.length ? ` (${dirtyKeys.length})` : ""}
                </button>
                <button className="btn" disabled={!connected} onClick={reload}>
                  Reset / Reload
                </button>
              </div>
            </div>
          </section>
        </div>
      </main>

      <section className="console">
        <div className="panel-head">
          <span className="panel-label">Console</span>
          <div className="spacer" />
          <span className="panel-label">
            {log.length} line{log.length === 1 ? "" : "s"}
          </span>
        </div>
        <div className="panel-body">
          <div className="log">
            {log.map((e) => (
              <div key={e.id} className={`log-line log-${e.dir}`}>
                <span className="log-ts">
                  {new Date(e.ts).toLocaleTimeString()}
                </span>
                <span className="log-tag">{e.dir}</span>
                <span className="log-text">{e.text}</span>
              </div>
            ))}
          </div>
          <div className="raw">
            <input
              value={raw}
              placeholder="raw command (e.g. enc, cur, state)"
              disabled={!connected}
              onChange={(e) => setRaw(e.target.value)}
              onKeyDown={(e) => e.key === "Enter" && sendRaw()}
            />
            <button className="btn" disabled={!connected} onClick={sendRaw}>
              Send
            </button>
          </div>
        </div>
      </section>

      <footer>hold-to-move · live tuning · ws debug console</footer>
    </div>
  );
}

function Field({
  field,
  live,
  edits,
  connected,
  position,
  onEdit,
}: {
  field: ConfigField;
  live: ConfigValues;
  edits: ConfigValues;
  connected: boolean;
  position: Position | null;
  onEdit: (key: string, value: string) => void;
}) {
  const liveVal = live[field.key] ?? "";
  const editVal = edits[field.key] ?? liveVal;
  const dirty = field.editable && editVal !== liveVal;

  // "Set to current position" is only meaningful with a valid (homed) reading.
  const canCapture = Boolean(
    field.setFromPosition && connected && position?.valid,
  );

  return (
    <label className={`field ${dirty ? "dirty" : ""}`}>
      <span className="fkey">{field.label}</span>
      <span className="fdesc">{field.description}</span>
      <div className="fcontrol">
        <input
          type={field.kind === "number" ? "number" : "text"}
          step="any"
          value={editVal}
          readOnly={!field.editable}
          disabled={!connected || !field.editable}
          onChange={(e) => onEdit(field.key, e.target.value)}
        />
        {field.setFromPosition && (
          <button
            type="button"
            className="capture btn btn-sm"
            disabled={!canCapture}
            title={
              canCapture
                ? "Record the current measured position into this field"
                : "Home the blind first to get a valid position"
            }
            onClick={(e) => {
              e.preventDefault();
              if (position) onEdit(field.key, position.mm.toFixed(1));
            }}
          >
            ⤓ Use current
          </button>
        )}
      </div>
    </label>
  );
}
