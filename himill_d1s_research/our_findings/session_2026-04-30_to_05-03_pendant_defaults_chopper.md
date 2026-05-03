# Session 2026-04-30 → 2026-05-03 — pendant, defaults, chopper

Three feature drops landed in this window. Each is described in the
README under "What works" — this file captures the diagnosis chains
and dead-ends, the things you'd want to know if you were resuming
work on any of them.

## 1. SmartPendant on UART4 (GX12-4P rear connector)

### Goal

Wire Nick Shilov's SmartPendant
([github.com/nickshl/SmartPendant](https://github.com/nickshl/SmartPendant))
to the D1S's rear GX12-4P connector. The pendant was already working
on the user's STM32F4 flexihal build; goal was feature parity on the
STM32F1 D1S.

### Wiring

GX12-4P pinout (verified by user with multimeter on the connector
shell, 2026-04-29):

| GX12 pin | Signal       | MCU pin |
|----------|--------------|---------|
| 1        | 5V           | —       |
| 2        | UART4_TX     | PC10    |
| 3        | UART4_RX     | PC11    |
| 4        | GND          | —       |

Build flags:

```
-D SERIAL1_PORT=4
-D MPG_ENABLE=2
-D MPG_STREAM=1
```

`MPG_ENABLE=2` = software toggle (`0x8B` from pendant), no
`MPG_MODE_PIN`. `MPG_STREAM=1` = pendant lives on the second
registered serial stream (UART2 / ESP3D is instance 0).

### What broke and was fixed

**(a) RAM exhaustion: $-dump truncation + hard crash.**
Two 8 KB UART TX buffers (UART2 + UART4) didn't fit in the F103RC's
48 KB SRAM. Symptoms: `$$` cut off after `$5=7`, USB CDC then
hard-crashed. Fix: per-UART buffer typedef in `Src/serial.c`,
`TX_BUFFER1_SIZE=1024` for UART4. RAM headroom restored.

**(b) Floating-pin noise → random feed-holds.**
With nothing plugged into UART4, `MPG_ENABLE=2` routes UART4 RX bytes
into `protocol_enqueue_realtime_command` via `stream_mpg_check_enable`.
Floating-pin noise on PC11 produced random ASCII bytes that mapped to
real-time commands (feed hold, soft reset, etc.). Fix: `GPIO_PULLUP`
on PC11 in `Src/serial.c:828-833`. Pin idles HIGH (UART idle) when
nothing is connected.

**(c) Self-sustaining MPG toggle loop (the hard one).**
This took 4 sessions to root-cause. Symptoms: pendant button engages
MPG mode, status reports start alternating `|MPG:1|`/`|MPG:0|` at
~0.5 Hz, jog wheel does nothing. Same firmware in
"`HIMILL_SKIP_MUX=1`" isolation mode (no D1S stream mux) worked
perfectly — confirmed pendant + protocol were healthy and pinpointed
the mux as the suspect.

False leads we ruled out:

- ESP3D forwarding `0x8B` bytes from telnet/WS clients
  (tested with no clients connected, still toggled)
- 0x8B at the mux source (`mux_usb_enqueue_rt`, `mux_uart_enqueue_rt`)
  — already filtered, didn't help
- `stream_select` reconnects via `esp_at` plugin (`ESP_AT_ENABLE` is
  not defined on D1S — dead code)
- Cable orientation (MPG mode engages → TX/RX direction is correct)

Real cause: when MPG is engaged, the pendant sends `?` to poll
status. That hits `protocol_enqueue_realtime_command` which schedules
a "regular" status report. The report goes through
`hal.stream.write_all` — which on D1S we override with `mux_write_all`.
`mux_write_all` was hardcoded to broadcast to USB + UART2 only — it
did NOT include the MPG stream (UART4). In stock grblHAL,
`stream_write_all` walks the connection list which includes the MPG
stream, so the pendant gets reports automatically. Our override broke
that.

The pendant, never seeing `|MPG:1` in the reply, decided MPG didn't
engage and re-sent `0x8B`. Our 500 ms `0x8B` debounce ate most of
those, but every 500 ms one slipped through, hit
`protocol.c:931-933`'s "0x8B in MPG mode → disable MPG" handler, and
flipped the state. → toggle loop.

Fix:

```c
// boards/himill_d1s.c, mux_write_all
if (sys.mpg_mode && mpg_uart_stream && mpg_uart_stream->write)
    mpg_uart_stream->write(str);
```

Three lines. Pendant immediately worked. **This is the single most
important property to preserve in any future mux/stream-wrapper
rework on this board.** It's documented in
`memory/project_d1s_mux_write_all_mpg.md` (auto-memory, persists
across sessions) and in `boards/himill_d1s.c:399-415` with a long
comment.

### Things to know about the SmartPendant protocol

- Sends `0x8B` (CMD_MPG_MODE_TOGGLE) to engage/release MPG.
- Sends `?` for status polls; expects `|MPG:1|` field in reply.
- Sends `$J=...\n` for jog moves while in MPG mode.
- **Heartbeats `0x8B` if it never sees `|MPG:1`** — this is the
  trap the toggle bug fell into. Once status reports reach the
  pendant, `0x8B` retries stop.
- Confirmed by reading `Application/GrblComm.cpp` in the SmartPendant
  repo.

## 2. Factory NVS seeding for G30 + G59.3

### Goal

`$#` parameters (G30, G59.3, G54-G59, TLO, etc.) live in a separate
NVS block from `$$` settings. They're NOT captured by the standard
"backup my settings" workflow grblHAL users follow. A re-flash or
factory reset leaves a new D1S without the toolsetter location the
M6 / probe pipeline expects. User wanted the D1S geometry baked in.

### What we settled on

`seed_himill_coord_defaults()` in `boards/himill_d1s.c`. Reads G30 and
G59.3 from NVS; if either is at all-zeros (the "uninitialized"
pattern), writes the D1S factory values:

- `G30 = (30, -7, 0)` — tool-change parking position
- `G59.3 = (0, -7, -15)` — toolsetter location

Called from both `himill_settings_load` (every boot) and
`himill_settings_restore` (`$RST=*` / `$RST=&` / first-boot
version-mismatch path). Idempotent — safe to call repeatedly.

### Coverage matrix

| User flow                            | Seeded? | When |
|--------------------------------------|---------|------|
| Fresh flash, blank NVS               | yes     | first boot via restore |
| Reboot, NVS valid + already seeded   | no-op   | every boot, but skipped (non-zero) |
| `$RST=$` + reboot                    | no-op   | settings reset doesn't touch coords |
| `$RST=#` + reboot                    | yes     | next boot, coords are zero → seed |
| `$RST=*` + reboot                    | yes     | both restore + load fire |
| `$RST=&` + reboot                    | yes     | restore fires immediately |

### Why no live re-seed on `$RST=#`

`grbl/settings.c:2798` only fires `details->restore` for non-core
plugins when `restore.driver_parameters` is set. `$RST=#` only sets
`restore.parameters`. So our hook can't fire mid-session on
`$RST=#`. User must reboot. Acceptable — rebooting after a factory
reset is the norm.

### Edge case: user intentionally sets G30 to (0,0,0)

Re-seeded on next boot. Trivially worked-around (use 0.0001) and
unlikely on this machine since both factory positions are non-zero
by design. Not addressing.

### `$342=65` decision

Tied to the `G59.3.Z=-15` choice. With Z at -15 mm in machine coords,
the toolsetter probe needs more search range than grblHAL's stock
`$342=30`. Validated 2026-05-01 with `$342=65`. Baked as
`-D DEFAULT_TOOLCHANGE_PROBING_DISTANCE=65.0f`.

## 3. TMC chopper user setting (`$451`)

### What it does

`Setting_UserDefined_1`, `Format_RadioButtons` "StealthChop,
SpreadCycle". Default 1 (SpreadCycle). Drives PD2 (TMC STEPPER_MODE)
in `apply_stepper_mode()`. Survives reboot, survives `$RST=$`.
`$RST=*` and `$RST=&` reset it back to 1.

### Why not auto-switch (TPWMTHRS hybrid mode)

TMC2208/2209 chips have a `TPWMTHRS` register that switches
StealthChop → SpreadCycle automatically above a programmed velocity.
That's how SKR / FlexiHAL boards deliver "silent at low speed, full
torque at high speed."

The D1S TMCs are wired in **standalone mode** — config pins on the
PCB set current/microsteps, no UART/SPI link to the MCU. So
`TPWMTHRS` is unreachable in firmware. Confirmed by
`boards/himill_d1s_map.h:36-37`:

```c
#if TRINAMIC_ENABLE
#error "HiMill D1/D1S TMC drivers are in STANDALONE mode. ..."
#endif
```

### Empirical findings (2026-05-03)

- `$451=0` (StealthChop): drops steps above ~1000 mm/min.
- `$451=1` (SpreadCycle): sustains the configured 2000 mm/min on
  $110/$111. This is also the back-EMF ceiling for SpreadCycle on
  this PCB voltage — pushing $110/$111 past 2000 would likely stall
  in either mode. MaxMake set 2000 mm/min as max in laser mode for
  the same reason.

### MaxMake's choice was the opposite of mine initially

MaxMake stock firmware: laser ON → SpreadCycle, laser OFF →
StealthChop. Their milling default was StealthChop. We ended up with
SpreadCycle as the default because:

1. Our `$110/$111=2000` overrides MaxMake's lower milling-mode max
   (probably 600-1000 mm/min in their config). At 2000 mm/min,
   StealthChop drops steps.
2. SpreadCycle has the torque headroom for the configured envelope.

If someone reduces `$110/$111` back below ~1000 mm/min (e.g., for a
machine that has known mechanical issues at speed), `$451=0`
StealthChop becomes a safe quieter option. Document for power users.

### Wear trade-offs (informational)

SpreadCycle vs StealthChop at the same average current:

| Component       | SpreadCycle effect                          |
|-----------------|---------------------------------------------|
| TMC chip        | More switching → MOSFETs run warmer         |
| Stepper windings| Higher RMS current (chopper ripple) → more I²R heat |
| Stepper iron    | Higher eddy losses at chopper frequency     |
| PSU caps        | Higher ripple draw (negligible)             |
| Bearings/screws | No effect — average force is identical      |

Not catastrophic in either mode — TMCs and steppers are spec'd for
SpreadCycle. But for a long unattended job at moderate load,
StealthChop is genuinely cooler. The D1S has continuous fans on the
driver board, so chip cooling isn't the bottleneck — stepper case
temperature is.

## File reference

Primary changes in this window (relative to repo root):

- `platformio.ini` — added `SERIAL1_PORT=4`, `MPG_ENABLE=2`,
  `MPG_STREAM=1`, `DEFAULT_TOOLCHANGE_PROBING_DISTANCE=65.0f`.
- `boards/himill_d1s.c` — pendant filter (`himill_mpg_rx_filter`,
  `himill_intercept_rt`), MPG fan-out fix in `mux_write_all`,
  `seed_himill_coord_defaults()`, `apply_stepper_mode()`,
  `Setting_UserDefined_1` registration.
- `boards/himill_d1s_map.h` — GX12-4P / UART4 documentation block.
- `Src/serial.c` — per-UART `TX_BUFFER1_SIZE=1024` typedef,
  `GPIO_PULLUP` on UART4 RX (PC11).

Auto-memory (persists across Claude sessions):

- `memory/project_d1s_back_gx12_uart4.md` — connector pinout
- `memory/project_user_smartpendant.md` — pendant model + heartbeat
  caveat
- `memory/project_d1s_mux_write_all_mpg.md` — mux/MPG fan-out invariant
