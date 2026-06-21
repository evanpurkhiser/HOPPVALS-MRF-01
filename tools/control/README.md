# HV-MRF-01 Control

A small React web app to control and tune the blinds controller over its WiFi
debug-mode websocket — live config tuning, hold-to-move motion control, and a
raw command console.

## Running

```sh
npm install
npm run dev
```

Open the printed `http://localhost:5173`. Served over plain `http` on purpose so
the browser allows the device's `ws://` connection (an `https` origin would
block it as mixed content).

## Using it

1. Put the device in **WiFi debug mode**: flip the "Debug Mode" switch on the
   device in Home Assistant. It reboots off Zigbee onto WiFi and comes up at its
   IP (default `10.0.0.135`; check the device's debug-mode boot log).
2. Enter the IP, click **Connect**. The config form auto-populates.
3. **Tune:** edit any `motion.*` field (changed fields highlight), click **Save**
   — the app sends `config set …` for each change, then `config save`, then
   re-reads to confirm. Gains apply live (no reboot). `net.*` is read-only.
4. **Move:** hold to move, release to stop —
   - `↑` / `W` → raise, `↓` / `S` → lower.
   - The **▲ Raise** / **▼ Lower** buttons work with mouse/touch (press-and-hold).
   - **■ STOP** brakes immediately.
5. **Console:** every command/response is logged; the raw input sends any line
   (`enc`, `cur`, `state`, etc.).

## Safety note

`motion stop` is sent on key release, button release, window blur, tab hide, and
on disconnect — and it bypasses the command queue so it's never delayed behind a
slower command.

⚠️ The device keeps executing its last motion command on its own; if the
**websocket drops while the blind is moving**, the browser can no longer send a
stop. The firmware's stall watchdog will brake at a hard end-stop, but the app
cannot stop a mid-travel move once disconnected. Keep the tab focused and
connected while jogging the blind.
