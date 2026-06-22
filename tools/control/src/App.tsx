import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { DeviceClient, type LogEntry, type Status } from "./device.ts";
import { FIELDS, parseConfig, type ConfigValues } from "./config.ts";

const MOVE_KEYS_UP = new Set(["ArrowUp", "w", "W"]);
const MOVE_KEYS_DOWN = new Set(["ArrowDown", "s", "S"]);

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

  // Wire client callbacks once.
  useEffect(() => {
    client.onStatus = setStatus;
    client.onLog = (e) => setLog((prev) => [...prev.slice(-299), e]);
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

  // On connect, pull config.
  useEffect(() => {
    if (status === "connected") void refreshConfig();
  }, [status, refreshConfig]);

  // ── Motion ────────────────────────────────────────────────────────────
  const stop = useCallback(() => {
    client.stop();
    setMoving(null);
  }, [client]);

  const move = useCallback(
    (dir: "raise" | "lower") => {
      if (!client.isOpen()) return;
      const r = Math.max(1, Math.min(300, Math.round(rpm)));
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
      .finally(() => setHoming(false));
  }, [client, homing]);

  // Go to a position as % of full travel. Blocks on the device until it
  // arrives (or the move's time cap), so use a long timeout like home.
  const gotoPct = useCallback(() => {
    if (!client.isOpen() || seeking || homing) return;
    const p = Math.max(0, Math.min(100, Math.round(pct)));
    setSeeking(true);
    client
      .send(`gotopct ${p}`, 40000)
      .catch(() => {})
      .finally(() => setSeeking(false));
  }, [client, pct, seeking, homing]);

  // Keyboard hold-to-move. The pressed set guards against keydown auto-repeat.
  useEffect(() => {
    const pressed = new Set<string>();

    const isTypingTarget = (t: EventTarget | null) =>
      t instanceof HTMLElement && (t.tagName === "INPUT" || t.tagName === "TEXTAREA");

    const onKeyDown = (e: KeyboardEvent) => {
      if (isTypingTarget(e.target)) return;
      const up = MOVE_KEYS_UP.has(e.key);
      const down = MOVE_KEYS_DOWN.has(e.key);
      if (!up && !down) return;
      e.preventDefault();
      if (pressed.has(e.key)) return; // ignore auto-repeat
      pressed.add(e.key);
      move(up ? "raise" : "lower");
    };

    const onKeyUp = (e: KeyboardEvent) => {
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

  return (
    <div className="app">
      <header>
        <h1>HV-MRF-01 Control</h1>
        <div className={`conn conn-${status}`}>
          <input
            value={ip}
            onChange={(e) => setIp(e.target.value)}
            placeholder="device IP"
            disabled={connected || status === "connecting"}
          />
          {connected || status === "connecting" ? (
            <button onClick={disconnect}>Disconnect</button>
          ) : (
            <button onClick={connect}>Connect</button>
          )}
          <span className="status-dot" /> <span className="status-label">{status}</span>
        </div>
      </header>

      <main>
        <section className="panel motion">
          <h2>Motion</h2>
          <label className="rpm">
            RPM
            <input
              type="number"
              min={1}
              max={300}
              value={rpm}
              onChange={(e) => setRpm(Number(e.target.value))}
            />
          </label>
          <div className="move-buttons">
            <button
              className={`move ${moving === "raise" ? "active" : ""}`}
              disabled={!connected}
              onPointerDown={(e) => {
                e.preventDefault();
                move("raise");
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
                move("lower");
              }}
              onPointerUp={stop}
              onPointerLeave={() => moving === "lower" && stop()}
            >
              ▼ Lower
            </button>
            <button className="stop" onClick={stop}>
              ■ STOP
            </button>
          </div>
          <div className="move-buttons">
            <button
              className="home"
              disabled={!connected || homing}
              onClick={home}
            >
              {homing ? "⌂ Homing…" : "⌂ Home"}
            </button>
          </div>
          <div className="goto-row">
            <label className="rpm">
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
              className="home"
              disabled={!connected || seeking || homing}
              onClick={gotoPct}
            >
              {seeking ? "Going…" : "Go"}
            </button>
          </div>
          <p className="hint">
            Hold <kbd>↑</kbd>/<kbd>W</kbd> to raise, <kbd>↓</kbd>/<kbd>S</kbd> to lower. Release
            to stop. Buttons work with mouse/touch too. <strong>Home</strong> drives both up to
            the top stop and zeroes the encoders. <strong>Go to %</strong> needs a prior home and
            a calibrated mm/rev; 100% = hard stop (clamped to soft stop).
          </p>
        </section>

        <section className="panel config">
          <h2>
            Configuration
            <button className="small" disabled={!connected} onClick={refreshConfig}>
              Refresh
            </button>
          </h2>
          <div className="fields">
            {FIELDS.map((f) => {
              const liveVal = live[f.key] ?? "";
              const editVal = edits[f.key] ?? liveVal;
              const dirty = f.editable && editVal !== liveVal;
              return (
                <label key={f.key} className={`field ${dirty ? "dirty" : ""}`}>
                  <span className="fkey">{f.label}</span>
                  <span className="fdesc">{f.description}</span>
                  <input
                    type={f.kind === "number" ? "number" : "text"}
                    step="any"
                    value={editVal}
                    readOnly={!f.editable}
                    disabled={!connected || !f.editable}
                    onChange={(e) => setEdit(f.key, e.target.value)}
                  />
                </label>
              );
            })}
          </div>
          <div className="config-actions">
            <button disabled={!connected || dirtyKeys.length === 0} onClick={save}>
              Save{dirtyKeys.length ? ` (${dirtyKeys.length})` : ""}
            </button>
            <button disabled={!connected} onClick={reload}>
              Reset / Reload
            </button>
          </div>
        </section>

        <section className="panel console">
          <h2>Console</h2>
          <div className="log">
            {log.map((e) => (
              <div key={e.id} className={`log-line log-${e.dir}`}>
                <span className="log-ts">{new Date(e.ts).toLocaleTimeString()}</span>
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
            <button disabled={!connected} onClick={sendRaw}>
              Send
            </button>
          </div>
        </section>
      </main>
    </div>
  );
}
