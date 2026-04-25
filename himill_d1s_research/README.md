# HiMill D1S grblHAL Research

This folder collects everything learned during the bring-up of grblHAL on
the HiMill D1S CNC controller (2026-04-24 → 2026-04-25). Use it as the
starting point for any future debugging session.

## Hardware in scope

- **MCU:** STM32F103RCT6 (LQFP-64, 256 KB flash, 48 KB SRAM, 8 MHz HSE)
- **Coprocessor:** ESP32-D0WD running ESP3D 3.0.0.b1 (HMD1-1.0.2 fork)
- **Connection between them:** USART2 (PA2 TX / PA3 RX) at 115200 baud, raw bytes
- **Drivers:** TMC2208/2209 in standalone mode (PD2 toggles silent ↔ fast)
- **Bootloader:** HiMill custom at 0x08000000 (16 KB used, 32 KB reserved)
- **App:** at 0x08008000 (verified from MaxMake reset vector 0x08008148)

## Folder layout

- `maxmake_artifacts/` — vendor-supplied originals
  - `HiMill_D1-v1.0.34-20251222.bin` — running app firmware
  - `Custom_Commands.CSV` / `.png` — vendor's `[ESP420]` / `[ESP110]` extension list
  - `HiMill_Main_Control_Board_IO_Pin_Description.CSV` — authoritative pin map
  - `ScreenShot_*.png` — assorted vendor docs
- `bootloader/` — HiMill bootloader binary + objdump
  - `HiMill_D1_Boot-v1.4-hw_v1.0.hex`
  - `bootloader.bin` (raw binary, easier for Ghidra)
  - `bootloader.asm` (`arm-none-eabi-objdump --adjust-vma=0x08000000` output)
- `esp3d/` — ESP3D source-of-truth config from the vendor
  - `configuration.h`
  - `platformio.ini`
- `our_findings/` — write-ups generated during this work
- `disassembly/` — staging area for Ghidra projects, additional notes

## What works (current build state, 2026-04-25)

These all work cleanly on our grblHAL build:

- Motion: X/Y up to 2000 mm/min, Z 1500 mm/min (TMCs forced to fast mode at boot via PD2)
- Homing: X/Y/Z with `$23=3` (X+Y home in negative direction per switch placement)
- Limits: X/Y/Z hard limits via PC0/PC1/PC2 EXTI
- Probe + toolsetter: PA1 = toolsetter, PB2 = 3D probe (G65P5Q1/Q0 selects)
- Spindle: PWM on PB8 (TIM4_CH3), $33=10000 Hz, $36=99 (avoid 100% no-toggle issue)
- Spindle ramping: $9 bit 3 + $394 / $539 = 1.0 s
- E-stop on PB4 (when `CONTROL_ENABLE=64`) — but see Open Issues
- Fast mode steppers (PD2 driven HIGH at boot)
- SD card: `$F`, `$FM` work
- A axis as rotary ($376=8)
- ESP3D bridge for websocket clients (`ws://<ip>:8282`) works cleanly
- ESP3D bridge for telnet clients mostly works, but see Open Issues
- Polled buttons on PB1/PC4 via `grbl.on_execute_realtime` shim
- `[BOOT]` / `[DFU]` user commands routed through `himill_user_command`

## Open issues / unknowns

### 1. Telnet path is still noisy; websocket path is clean

**Current state (after mux fix):**

- `$esh` now completes successfully with `USB_SERIAL_CDC=1`
- `websocat ws://<ip>:8282` is clean:
  - no duplicate banners
  - no initial `error:1`
  - first command executes normally
- Telnet on port 23 is still odd:
  - duplicate banner behavior on connect
  - first command may return `error:1`
  - `nc <ip> 23` shows delayed ESP3D welcome/bridge-ready behavior

**What was actually fixed:**

The D1S stream mux in `boards/himill_d1s.c` originally wrapped only part of
`hal.stream`. That left grblHAL's long-output path (`hal.stream.write_n()` in
`grbl/report.c`) still going straight to USB CDC, while short writes went
through the mux. Expanding the mux to cover the rest of the stream contract
fixed the `$esh` failure.

**What the current evidence suggests:**

The remaining problem is likely **specific to the ESP3D telnet daemon/session
path**, not a generic STM32/USB/UART instability:

- websocket uses the same ESP32→UART2→STM32 data path and is clean
- telnet is not transparent immediately on connect
- ESP3D's captured config leaves telnet welcome enabled

**Implication:**

Treat websocket (`ws://<ip>:8282`) as the preferred supported network path for
now. If telnet support matters later, debug the ESP3D telnet side first.

See also:
- `our_findings/session_2026-04-25_stream_mux_followup.md`

### 2. MaxMake's `DS:<bits>` status field

The vendor firmware's `?` status reply included a custom `DS:<n>` bitfield
exposing pins that grblHAL's stock `Pn:` doesn't surface. From the binary's
strings + observed behavior:

| Bit | Meaning |
|----|---------|
| 3 | Safety door enable |
| 4 | Safety door trigger (active-high) |
| 5 | Physical KEY button (always set in samples — possibly power rail) |
| 7 | Height gauge (3D probe) trigger (active-low) |

**Speculation:** MaxMake may have routed PB1 (door), PB4 (e-stop / reset),
PC4 (button), and PB2 (probe2) as POLLED status bits in `DS:` rather than
binding them to grblHAL's CONTROL signals. That avoids:
- IRQ collisions with limit pins on STM32F1's per-line EXTI hardware
- The reset-cascade triggered by grblHAL's `mc_reset()` chain on critical
  signal events
- The "any glitch on PB4 = full system reset" failure mode

If this is correct, the path forward is to mirror their architecture:
**don't bind these pins to grblHAL CONTROL inputs at all**; expose them
through a custom realtime status field for senders to monitor.

### 3. Bootloader entry from running app

This is now understood and working.

Recovered from the bootloader + vendor app binaries:

- flag address: `0x2000BFF0`
- flag value: `0xFA0505F5`

Required sequence:

1. write `0xFA0505F5` to `*(volatile uint32_t *)0x2000BFF0`
2. wait about 10 ms
3. request system reset (`SYSRESETREQ`)

`[BOOT]` in `boards/himill_d1s.c` now uses this mechanism and has been
verified on hardware.

See also:
- `our_findings/session_2026-04-25_bootloader_flag_recovered.md`

### 4. Telnet first-command `error:1`

This is still present on telnet, but **not** on websocket.

Latest observations:

- `websocat ws://<ip>:8282` → first command executes cleanly
- `telnet <ip> 23` → immediate ESP3D + grblHAL banner behavior, then first
  command may return `error:1`
- `nc <ip> 23` → no immediate response; after roughly 6 seconds ESP3D emits
  `Welcome to ESP3D ...`, after which commands execute normally

Current interpretation:

- likely telnet-session / ESP3D-side sequencing or injected bytes
- not the same class of bug as the old `$esh` failure
- not reproduced on websocket

### 5. USB enumeration vs network primacy

ESP3D-bridge users typically don't connect USB. But during firmware
flashing or initial WiFi setup, USB is needed. Keeping `USB_SERIAL_CDC=1`
in the build is still required.

The DTR-driven mux switching is now working well enough for normal use:

- USB CDC can stay enabled in the build
- websocket networking works cleanly with USB support compiled in
- the remaining rough edges are telnet-specific, not a general "USB CDC on
  breaks networking" failure anymore

## Build flags reference

Current `[env:himill_d1s]` flags worth knowing:

| Flag | Value | Purpose |
|------|-------|---------|
| `USB_SERIAL_CDC` | 1 | USB CDC active for ESP3D bootstrap + dev |
| `SERIAL_PORT` | 2 | Register UART2 (PA2/PA3) as a stream |
| `TX_BUFFER_SIZE` | 8192 | Big enough for full $esh dump (~10 KB) without blocking |
| `RX_BUFFER_SIZE` | 512 | Default — RX is fine |
| `CONTROL_ENABLE` | 0 | DIAGNOSTIC — e-stop currently disabled |
| `SAFETY_DOOR_ENABLE` | (off) | Door is dual-purpose, not bound; polled instead |
| `TOOLSETTER_ENABLE` | 1 | PA1 = toolsetter, PB2 = primary probe |
| `MACROS_ENABLE` | 1 | Macros plugin (button-action mappings) |
| `ROTATION_ENABLE` | 1 | A axis as rotary |
| `MINIMIZE_PROBE_OVERSHOOT` | 1 | Probe accuracy |
| `SDCARD_ENABLE` | 1 | $F, $FM |
| `VECT_TAB_OFFSET` | 0x8000 | Match HiMill bootloader's app entry |

## Settings baked as `-D DEFAULT_*` flags

All in `[env:himill_d1s].build_flags`:
- $0=7 (step pulse), $1=25 (idle delay), $4=15 (enable invert all axes)
- $5=7 (limit invert NC switches), $6=1 (probe invert)
- $9 ramp bit, $14=64 (e-stop invert), $16=4 (PWM invert)
- $21=1, $22=43, $23=3 (limits/homing/dir per axis)
- $24=25 (homing slow feed), $27=1.0 (pulloff)
- $30=13000, $33=10000, $36=99 (spindle envelope)
- $40=1 (jog soft limit), $341=3 (manual M6 toolchange)
- $394=1.0, $539=1.0 (spindle ramp times)
- $100-103=1600/1600/1600/88.889 (steps/mm)
- $110/$111=2000, $112=1500, $113=2000 (max rates)
- $120-123 accel, $130-133 max travel
- DEFAULT_AXIS_ROTATIONAL_MASK=8 (A rotary)

## Files modified in our work

- `boards/himill_d1s_map.h` — pin map per HiMill CSV + comments on EXTI constraints
- `boards/himill_d1s.c` — board init, [BOOT]/[DFU]/[ESP] user commands,
  realtime hook for button polling, USB-CDC ↔ UART2 mux, fast-mode-always
  PD2 drive, pause-on-SD-job
- `Inc/driver.h` — added PB8 TIM4_CH3 to spindle PWM resolver, added
  BOARD_HIMILL_D1S include
- `Src/driver.c` — added `#ifdef COOLANT_FLOOD_PIN` guard
- `platformio.ini` — added [env:himill_d1s] env with all the above flags

## EGSender side

The companion sender (EGSender) has D1S-specific code in
`docs/controllers/himill_d1s.md` covering protocol quirks (G92 zero
workflow, SD unmount during HTTP I/O, etc.). That's the user-facing
documentation; this folder is the firmware-side research.

## Tools used

- `arm-none-eabi-objdump --adjust-vma=0x08000000` for bootloader disasm
- `arm-none-eabi-objcopy -I ihex -O binary` for hex → bin conversion
- `strings -a` for finding embedded text in vendor firmware
- `usbpcap` (Windows) / `usbmon` (Linux) for capturing the original
  MaxmakeLAB ↔ bootloader USB protocol
- ST-LINK + `st-flash --freq=100` for SWD flashing (note: faster speed
  hits flash-loader bugs; 100 kHz is reliable)
- `pio run -e himill_d1s` for builds

## Next session: where to pick up

1. **Decide whether telnet matters enough to chase.**
   - If no: standardize on websocket (`ws://<ip>:8282`) and move on.
   - If yes: inspect or rebuild ESP3D telnet behavior first.
2. **MaxMake `DS:<bits>` compatibility decision.**
   - Keep stock grblHAL behavior, or
   - Add a custom status extension to mirror vendor semantics for door /
     key / probe / estop-adjacent signals.
3. **Rotary axis testing** — N_AXIS=4 + DEFAULT_AXIS_ROTATIONAL_MASK=8
   built but not yet exercised end-to-end with a real rotary.
4. **Laser routing decision** — currently single-spindle-with-laser-mode
   (`$32=1`); could be split into dual-spindle (PB8 + PB0/PC5).
5. **WS2812 LEDs** — PB5 ring + PB15 indicator, both signal-inverted, not
   yet driven.

## Resume checklist

When resuming this work:

1. Read this README first
2. Check `our_findings/session_*.md` (when populated) for the most recent
   session's notes
3. Run `pio run -e himill_d1s` to confirm a clean build
4. Use `st-flash --freq=100 --reset write .pio/build/himill_d1s/firmware.bin 0x08008000`
   for ST-LINK flashing (the slow speed avoids the loader bug)
5. Check `EGSender/docs/controllers/himill_d1s.md` for sender-side context
6. Memory file `project_d1s_firmware_flash.md` (auto-memory) has the
   session-to-session pointers
