# Bug: D1S WS path occasionally doubles `$` on the wire → grblHAL Error 3

**Status**: open, intermittent, hard to reproduce reliably from a Python
script — appears far more often when a real sender (EGSender, gSender)
is the client. We've ruled out every layer we can without ESP3D
instrumentation.

**Date opened**: 2026-04-25

## Symptom

When the user sets a work-coord value via the sender's DRO (which
dispatches `G10 L20 P0 <axis><value>` followed by `$#` to refresh the
cached offsets), the controller occasionally responds with:

```
error:3   ('$' system command was not recognized or supported.)
```

The "invalid command" looks fine in the sender's TX log (`$#`).
The bug is on the wire — a hex dump of the bytes the STM32 mux
*actually receives* shows `0x24 0x24 0x23 0x0A` = literal `$$#\n`,
not `$#\n`. That extra `$` byte is what grblHAL's parser correctly
rejects (it sees `$$` = "list-all-settings" with `#` as garbage args).

User reproduces consistently in EGSender via repeated DRO clicks.
gSender shows the same misbehavior on D1S+telnet ("values don't
hold"). Bcnc and the same EGSender on a FlexiHAL controller are
clean.

## Capture

Captured by adding a `[STATS]` user-command in `boards/himill_d1s.c`
that dumps a 64-byte ring of the most recent bytes the stream mux
saw (after telnet IAC stripping but otherwise raw). Reset with
`[STATS:reset]` between runs.

Tail from a failing session, just after the user clicked X / Y / Z
on the sender's DRO:

```
0a
G10 L20 P0 X30\n
$#\n
G10 L20 P0 Y-30\n
$#\n
G10 L20 P0 Z2\n
$$#\n            <-- bug fires here
[STATS]\n
```

Counters during the same run:

```
mux:active=UART  usb_conn=0  iac_state=0  tunnel_ms=0
usb:rx_now=0 rx_free=512 tx_now=0
uart:rx_now=0 rx_free=512 tx_now=100
counters:reads=9676099 bytes=226 iac=0 lines=33
picks:usb=0 uart=9676099 fallback=0 usb_no_data=0
```

Interpretation: bytes accounted for, no IAC, no telnet artifact, no
USB DTR flicker, no fallback to UART, no overrun visible at this
layer. The corruption is **upstream of `mux_read`** — the byte
arrives over UART2 already malformed.

## What we've ruled out

| Layer | Status | Evidence |
|---|---|---|
| EGSender command queue | clean | `[STATS:tail]` shows the doubled `$` arriving at STM32 from ESP3D's UART writer; sender's TX log shows clean `$#` |
| Different sender | not the cause | gSender hits the same on D1S |
| Other controllers | not affected | EGSender on FlexiHAL is clean |
| Sender protocol layer | not the cause | bcnc clean on D1S USB; same controller fails over WS/telnet |
| STM32 USB CDC path | clean | bcnc + USB serial: 0 errors over 200-cycle stress |
| STM32 UART2 stream mux | clean | counters above; tail shows raw inbound bytes |
| Stream mux `cancel_read_buffer` gap | unrelated | already fixed (1b18fb5/3979b24); reproduces with that fix in place |

## Architecture clarification (2026-04-25)

ESP3D has **two** WebSocket servers (websocket_server.cpp:35-38):

```cpp
WebSocket_Server websocket_terminal_server("webui-v3", webui_websocket);
WebSocket_Server websocket_data_server   ("arduino",   websocket);   // port 8282
```

They route incoming frames very differently:

- `websocket_terminal_server` (`"webui-v3"`): in `handle_Websocket_Terminal_Event`, both `WStype_TEXT` and `WStype_BIN` are **ignored** (except for auth `PING:` strings). Never enters `push2RXbuffer`. This is the WebUI status pipe, not a gcode pipe.
- `websocket_data_server` (`"arduino"`, port 8282): every text/binary frame goes through `push2RXbuffer` byte-by-byte and `flushRXbuffer` on `\n`. **This is the path with the bug**.

EGSender's machine-config flag `web-ui?` selects subprotocol negotiation:

```clojure
;; ws.cljs:670
ws (if web-ui?
     (js/WebSocket. url "webui-v3")
     (js/WebSocket. url))
```

That clears up two earlier mysteries:

1. "FlexiHAL is clean with WebUI on" — FlexiHAL is **not** using ESP3D at all. It's grblHAL/STM32F4xx with the in-tree `Plugin_WebUI`, a separate implementation that just happens to also advertise `webui-v3`. Different codebase, different bug surface.
2. "D1S won't connect with WebUI on" — D1S's data port (8282) advertises `"arduino"`, not `"webui-v3"`. The handshake fails. Client falls back to no-subprotocol, which lands on the same buggy data path.

There is no alternative subprotocol on ESP3D's data port that would route around `push2RXbuffer`. The fix has to be in `push2RXbuffer` / `flushRXbuffer` themselves (mutex around `_RXbuffer` / `_RXbufferSize`).

## What's still on the table

- **ESP3D's WebSocket → UART writer**: sender uses `WebSocket.send(Uint8Array)`
  which produces a **WStype_BIN** frame; `bcnc` does not use WS at all.
  ESP3D's `webSocketEvent` dispatches BIN and TEXT to the same
  `push2RXbuffer`, but the WebSockets-library decoder ahead of that
  (mask, length parsing, fragmentation) differs by frame type and
  could leak a byte into the buffer.
- **ESP3D buffer race**: `push2RXbuffer` is byte-by-byte; if the WS
  event callback runs in a different FreeRTOS task than where
  `flushRXbuffer` resets `_RXbufferSize=0`, two near-simultaneous
  frames could append into a buffer that hasn't been reset yet.
  Telnet's analogous code (`telnet_server.cpp`) is driven from
  polled `handle()`, so it doesn't have the race; that fits with
  telnet showing a much lower failure rate (~5% vs ~50% on WS).
- **Initial conditions**: gSender and EGSender both fire a "connect
  flood" of commands on attach (`$I+ $# $G $EG $ESH $ES $$ $EA $EE
  $FM $F $#`). Some of these (`$$`, `$ESH`, `$EE`) produce multi-KB
  responses that exceed typical WS frame size and may force
  fragmentation in ESP3D's WS writer. The Python script reproduces
  this connect flood (`init_burst` test) but does not reliably
  trigger the bug, so connect-flood priming alone isn't the
  trigger. Something about the *browser's* WS implementation
  (TLS-like compression, frame size choice, ping cadence) is what
  finishes lighting the fuse.

## Heisenbug warning

Adding the `[STATS]` instrumentation appears to **mask the bug** in
the Python repro script:

- Before STATS instrumentation: `stress` (200 cycles, no real
  sender) showed ~50% errors on WS, ~5% on telnet.
- After STATS instrumentation: same `stress` shows 0/200.

Sender + browser still reproduce after the STATS build — so the
underlying ESP3D bug is unchanged, but our STM32-side mux now reads
bytes a few cycles slower per byte (extra counter increments + the
`tail` ring write), and apparently that's enough to shift the
ESP3D writer-side timing past whatever race window the script was
hitting. The browser is fast enough to keep hitting the window.

This means: **we cannot reliably regression-test ESP3D from the
script alone**. The repro story is "use a browser sender against
the controller and click DRO coords ~10 times". Until we patch
ESP3D, that's our test.

## Where to look next

1. ESP3D `esp3d/src/modules/websocket/websocket_server.cpp`:
   - `WebSocket_Server::push2RXbuffer` — byte-by-byte append, no
     locking around `_RXbuffer` / `_RXbufferSize`.
   - `webSocketEvent` (line ~95) — dispatched by WebSockets library
     callback, almost certainly from a different task than
     `WebSocket_Server::handle()` which also flushes on timeout.
   - `flushRXbuffer` resets `_RXbufferSize = 0` only AFTER
     `flushRXData → esp3d_commands.process(message)` returns; if
     `process()` queues asynchronously and yields, a concurrent
     `push2RXbuffer` from the WS event task can write to
     `_RXbuffer[_RXbufferSize]` with the OLD `_RXbufferSize` value.
2. Possible simple fix: wrap `push2RXbuffer` and `flushRXbuffer` in
   a FreeRTOS mutex.
3. Confirm with logic-analyser capture on UART2 TX from ESP32: does
   the doubled `$` appear on the wire from ESP32, or is it added
   somewhere on the STM32 side we haven't instrumented?

## Repro recipe

1. Flash current firmware (with STATS — does not block reproduction
   when driven by a browser sender).
2. Connect the sender (EGSender or gSender) over WebSocket to D1S.
3. Wait for the connect flood to drain — the sender shows "Connected".
4. Click any work-coord in the DRO, type a value, press Enter.
   Repeat for X, Y, Z, A in any order.
5. Within ~3-5 clicks, you'll see `error:3 → $#` in the console log.
6. Send `[STATS]` to dump counters + tail. Tail shows `0x24 0x24
   0x23 0x0A` somewhere in the recent byte stream.

## Workaround at the sender layer (not implemented)

If we accept this is ESP3D's problem and we can't fix ESP3D right
now, EGSender could:

- Defer the `$#` after `G10 L20` by ~150 ms, OR
- Drop the `$#` entirely and rely on the next `<...|WCO:...>` status
  report to refresh cached WCS offsets.

The status-report approach is cleaner — WCO is broadcast on change
anyway. Sender's `:wcs-offsets` cache won't update per-WCS until a
fresh `$#` runs at idle, but the DRO's live work-coord display only
uses WCO and would update correctly.

We chose not to ship this workaround so the underlying bug stays
visible.
