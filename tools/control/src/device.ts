// WebSocket client for the HV-MRF-01 debug console.
//
// Protocol: one text frame out = one command line; one text frame back = that
// command's captured output. The device serializes commands, so we keep at
// most one normal command in flight and match responses FIFO.
//
// `motion stop` is special: it bypasses the pending queue (and flushes it) so a
// release/blur always brakes promptly and a queued move can't fire after a stop.

export type Status = "disconnected" | "connecting" | "connected" | "error";

// Current blind position, decoded from a device `pos L=… R=… moving=… valid=…`
// line (emitted by `pos` and the `motion stop` reply). mm is the L/R average,
// in mm below the homed top. valid is false when there's no usable reference
// (not homed / mm_per_rev unset), so the mm figures are meaningless.
export interface Position {
  mm: number;
  mmL: number;
  mmR: number;
  moving: boolean;
  valid: boolean;
}

const POS_RE = /pos L=(-?[\d.]+)mm R=(-?[\d.]+)mm moving=([01]) valid=([01])/;

// Pull a Position out of any frame that carries the canonical position line,
// or null if the frame has none.
export function parsePosition(text: string): Position | null {
  const m = text.match(POS_RE);
  if (!m) return null;
  const mmL = parseFloat(m[1]);
  const mmR = parseFloat(m[2]);
  return {
    mm: (mmL + mmR) / 2,
    mmL,
    mmR,
    moving: m[3] === "1",
    valid: m[4] === "1",
  };
}

export interface LogEntry {
  id: number;
  ts: number;
  dir: "tx" | "rx" | "info" | "err";
  text: string;
}

interface Waiter {
  resolve: (text: string) => void;
  reject: (err: Error) => void;
  timer: ReturnType<typeof setTimeout>;
  cmd: string;
}

interface Queued {
  cmd: string;
  resolve: (text: string) => void;
  reject: (err: Error) => void;
  timeoutMs: number;
}

const STOP_CMD = "motion stop";
const CMD_TIMEOUT_MS = 5000;

export class DeviceClient {
  private ws: WebSocket | null = null;
  private waiters: Waiter[] = [];
  private pending: Queued[] = [];
  private logId = 0;

  status: Status = "disconnected";
  onStatus: (s: Status) => void = () => {};
  onLog: (e: LogEntry) => void = () => {};
  // Fires for any frame carrying a position line, regardless of which command
  // (if any) it answers — so an out-of-band `motion stop` reply still updates
  // the UI's position readout.
  onPosition: (p: Position) => void = () => {};

  private setStatus(s: Status) {
    this.status = s;
    this.onStatus(s);
  }

  private log(dir: LogEntry["dir"], text: string) {
    this.onLog({ id: ++this.logId, ts: Date.now(), dir, text });
  }

  connect(ip: string) {
    this.disconnect();
    const url = `ws://${ip}/ws`;
    this.setStatus("connecting");
    this.log("info", `connecting to ${url}`);

    let ws: WebSocket;
    try {
      ws = new WebSocket(url);
    } catch (e) {
      this.setStatus("error");
      this.log("err", `bad URL: ${String(e)}`);
      return;
    }
    this.ws = ws;

    ws.onopen = () => {
      this.setStatus("connected");
      this.log("info", "connected");
    };
    ws.onmessage = (ev) => {
      const text = typeof ev.data === "string" ? ev.data : "[binary frame]";
      const w = this.waiters.shift();
      this.log("rx", text);
      const pos = parsePosition(text);
      if (pos) this.onPosition(pos);
      if (w) {
        clearTimeout(w.timer);
        w.resolve(text);
      }
      this.pump();
    };
    ws.onerror = () => {
      this.log("err", "websocket error");
      this.setStatus("error");
    };
    ws.onclose = () => {
      this.log("info", "disconnected");
      this.failAll(new Error("connection closed"));
      if (this.status !== "error") this.setStatus("disconnected");
      this.ws = null;
    };
  }

  disconnect() {
    if (this.ws) {
      const ws = this.ws;
      ws.onopen = ws.onmessage = ws.onerror = ws.onclose = null;
      try {
        ws.close();
      } catch {
        /* ignore */
      }
      this.ws = null;
    }
    this.failAll(new Error("disconnected"));
    this.setStatus("disconnected");
  }

  isOpen() {
    return this.ws != null && this.ws.readyState === WebSocket.OPEN;
  }

  // Queue a normal command. Resolves with the response text. timeoutMs lets
  // long-blocking commands (e.g. `home`, which drives until both motors settle)
  // override the default response timeout.
  send(cmd: string, timeoutMs: number = CMD_TIMEOUT_MS): Promise<string> {
    return new Promise<string>((resolve, reject) => {
      if (!this.isOpen()) {
        reject(new Error("not connected"));
        return;
      }
      this.pending.push({ cmd, resolve, reject, timeoutMs });
      this.pump();
    });
  }

  // Brake immediately: flush any queued moves, then send stop out of band.
  // Safe to call even when disconnected (no-op).
  stop() {
    const flushed = this.pending;
    this.pending = [];
    for (const q of flushed) q.reject(new Error("flushed by stop"));
    if (!this.isOpen()) return;
    this.writeFrame(STOP_CMD).catch((e) => this.log("err", `stop: ${e.message}`));
  }

  private pump() {
    if (!this.isOpen()) return;
    if (this.waiters.length > 0) return; // one normal command in flight at a time
    const next = this.pending.shift();
    if (!next) return;
    this.writeFrame(next.cmd).then(
      () => {},
      (e) => next.reject(e),
    );
    this.waiters.push({
      resolve: next.resolve,
      reject: next.reject,
      cmd: next.cmd,
      timer: setTimeout(() => this.onTimeout(next.cmd), next.timeoutMs),
    });
  }

  private writeFrame(cmd: string): Promise<void> {
    return new Promise((resolve, reject) => {
      if (!this.isOpen()) {
        reject(new Error("not connected"));
        return;
      }
      try {
        this.ws!.send(cmd);
        this.log("tx", cmd);
        resolve();
      } catch (e) {
        reject(e instanceof Error ? e : new Error(String(e)));
      }
    });
  }

  // A timed-out response desyncs FIFO matching, so we tear the socket down to
  // resync cleanly. failAll runs via onclose; the UI's stop-on-close fires too.
  private onTimeout(cmd: string) {
    this.log("err", `timeout waiting for response to: ${cmd}`);
    this.setStatus("error");
    this.disconnect();
  }

  private failAll(err: Error) {
    for (const w of this.waiters) {
      clearTimeout(w.timer);
      w.reject(err);
    }
    this.waiters = [];
    for (const q of this.pending) q.reject(err);
    this.pending = [];
  }
}
