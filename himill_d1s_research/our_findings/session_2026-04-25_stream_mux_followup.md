# Session follow-up: stream mux fix, `$esh` recovery, and telnet vs websocket behavior

Date: 2026-04-25

## Summary

This session split the original "telnet is broken" problem into two separate issues:

1. A **real STM32/grblHAL-side stream-mux bug** was causing long outputs such as `$esh` to stop mid-stream when `USB_SERIAL_CDC=1`.
2. The remaining **double-banner + first-command-on-telnet oddities now look telnet-path-specific on the ESP3D side**, not like a general STM32/UART/USB instability.

The important result is that **`$esh` now completes successfully** with `USB_SERIAL_CDC=1`, so the major blocker is no longer the long-output/reset-cascade behavior.

## What was actually wrong in the D1S firmware

In `boards/himill_d1s.c` we had a partial stream wrapper:

- `hal.stream.read`
- `hal.stream.write`
- `hal.stream.write_char`
- some RX helpers

were overridden to dynamically pick USB CDC or UART2.

However, grblHAL's reporting code does not only use `write()` and `write_char()`. In particular, human-readable settings reporting (`$esh`) uses `hal.stream.write_n()` for chunks of descriptive text in `grbl/report.c`.

That meant the stream was effectively **split-brain**:

- short/plain writes went through the D1S mux
- some long/structured writes still went directly to the original USB CDC handlers

With `USB_SERIAL_CDC=1`, this was enough to break `$esh` over the ESP3D UART bridge.

## Firmware changes made

The D1S board mux in `boards/himill_d1s.c` was expanded so that the active stream is consistently mediated.

Added wrapper coverage for:

- `is_connected`
- `write_n`
- `get_tx_buffer_count`
- `reset_read_buffer`
- `reset_write_buffer`
- `disable_rx`
- `suspend_read`
- `set_enqueue_rt_handler`

Also added:

- USB-read fallback to UART read when USB appears active but has no byte ready

This fallback was intended to prevent stale or phantom USB DTR state from black-holing the first network command.

## Result of the mux fix

Confirmed improvement:

- `$esh` now completes successfully over the network path
- build still works with `USB_SERIAL_CDC=1`
- no regression seen in the current build flow

This is the strongest sign that the main architectural bug was in our mux integration, not in some mysterious hardware reset condition.

## Additional telnet-side experiments

After the mux fix:

- `websocat ws://<ip>:8282` behaved cleanly
  - no welcome banners
  - first command executed cleanly
- `telnet <ip> 23` still showed:
  - double welcome/banner behavior
  - first command returning `error:1`
- `nc <ip> 23` showed:
  - no immediate banner
  - no command response at first
  - after about 6 seconds: `Welcome to ESP3D VHMD1-1.0.2_based_on_3.0.0.b1`
  - after that delay, commands executed normally without the initial `error:1`

These results matter because they strongly suggest the remaining problem is **specific to the ESP3D telnet daemon/session behavior**, not the raw UART bridge:

- websocket path is clean
- telnet path is noisy
- `nc` on port 23 is delayed until ESP3D presents its own welcome/ready state

## What this likely means now

The current best interpretation is:

- the STM32/grblHAL side is now fundamentally healthy for network use
- ESP3D telnet is not acting like a transparent raw byte pipe immediately on connect
- some telnet-specific welcome/protocol/session-init behavior is still being injected before the bridge becomes clean

That would explain why:

- websocket is fine
- telnet is not
- the original MaxMake firmware may have "worked" simply because it tolerated or masked this telnet behavior better, or because its sender path used websocket-like semantics more often, or because its firmware-side parser/status behavior absorbed the noise more gracefully

## Why this is probably not a hidden STM32 problem anymore

If there were still a generic STM32-side instability, we would expect it to show up on websocket too, because websocket still uses the same ESP32-to-UART2-to-STM32 path for payload bytes.

Instead:

- websocket is clean
- telnet is not

That is a strong discriminator. It does **not** prove there is zero remaining firmware-side nuance, but it does mean the remaining issue is unlikely to be the same class of bug as the old `$esh` failure.

## Current status

### Fixed

- Long-output network failure for `$esh`
- The major `USB_SERIAL_CDC=1` blocker appears resolved in the D1S firmware

### Still odd, but narrowed down

- Telnet double banner on connect
- First telnet command may return `error:1`
- Telnet session appears to become "normal" only after ESP3D's own delayed welcome/ready phase

### Clean path today

- Websocket (`ws://<ip>:8282`) is currently the preferred network transport

## Recommended next step

Treat websocket as the preferred supported path for now.

If telnet matters later, the next investigation should be on the ESP3D side, not in grblHAL first:

- inspect or rebuild ESP3D telnet behavior
- consider enabling `DISABLE_TELNET_WELCOME_MESSAGE` in the captured ESP3D config
- compare telnet session sequencing against MaxMake's shipped behavior if needed

## Files touched in this session

- `boards/himill_d1s.c`

## Build verification

Verified with:

```sh
pio run -e himill_d1s
```

Build completed successfully after the mux changes.
