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
- Homing: X/Y/Z with `$23=1` + `$3=2` (Y motor reversed in stock D1S wiring; bake both)
- Limits: X/Y/Z hard limits via PC0/PC1/PC2 EXTI
- Probe + toolsetter: PA1 = toolsetter, PB2 = 3D probe (`G65P5Q1` / `Q0` selects)
- Spindle: PWM on PB8 (TIM4_CH3), $33=10000 Hz, $36=99 (avoid 100% no-toggle issue)
- Spindle ramping: $9 bit 3 + $394 / $539 = 1.0 s
- E-stop on PB4 with `CONTROL_ENABLE=64`
- Fast mode steppers (PD2 driven HIGH at boot)
- SD card: `$F`, `$FM` work
- A axis as rotary ($376=8)
- ESP3D bridge for websocket clients (`ws://<ip>:8282`) works cleanly
- ESP3D bridge for telnet clients mostly works, but see Open Issues
- Polled buttons on PC4 (cycle-start) via `grbl.on_execute_realtime` shim
- `[BOOT]` / `[DFU]` / `[LASER_MODE]` / `[ESP*]` user commands routed
  through `himill_user_command` — laser mode toggles spindle output
  PB8↔PB0 plus PC5 enable
- Jog cancel works correctly on telnet/WS — was a stream-mux gap
  (`cancel_read_buffer` missed); fixed 2026-04-25
- Toolsetter cleanup hook on `grbl.on_tool_selected`: optional via
  `$450=1`, hovers spindle 5 mm above toolsetter at G59.3 X/Y, runs
  S13000 for 3 s before each M6 (skipped on the very first M6 of the
  session — no TLR yet ⇒ nothing has been cut yet ⇒ no chips). Default
  Off. Use case: aluminum chips covering the toolsetter; spindle
  airflow blows them off.

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

The SRAM word is protected by the HiMill-only linker script
`STM32F103RCTX_HIMILL_D1S_FLASH.ld`, which moves `_estack` down to
`0x2000BFF0`. The shared `STM32F103RCTX_FLASH.ld` remains generic for other
boards.

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
| `CONTROL_ENABLE` | 64 | E-stop on PB4 (bit 6 = `CONTROL_ESTOP`) |
| `SAFETY_DOOR_ENABLE` | (off) | Door is dual-purpose, not bound; polled instead |
| `TOOLSETTER_ENABLE` | 1 | PA1 = toolsetter, PB2 = primary probe |
| `MACROS_ENABLE` | 1 | Macros plugin (button-action mappings) |
| `ROTATION_ENABLE` | 1 | A axis as rotary |
| `MINIMIZE_PROBE_OVERSHOOT` | 1 | Probe accuracy |
| `SDCARD_ENABLE` | 1 | $F, $FM |
| `VECT_TAB_OFFSET` | 0x8000 | Match HiMill bootloader's app entry |

## Settings baked as `-D DEFAULT_*` flags

All in `[env:himill_d1s].build_flags`:
- $0=7 (step pulse), $1=255 (idle delay = hold forever; required for TMC standalone)
- $3=2 (Y direction inverted), $4=15 (enable invert all axes)
- $5=7 (limit invert NC switches), $6=1 (probe invert)
- $9 ramp bit, $14=64 (e-stop invert), $16=4 (PWM invert)
- $21=1 (hard limits), $22=43 (homing options), $23=1 (X home -, Y home +)
- $24=25 (homing slow feed), $27=1.0 (pulloff)
- $30=13000, $33=10000, $36=99 (spindle envelope)
- $40=1 (jog soft limit), $341=3 (auto-touchoff M6 toolchange)
- $346 bit1+bit2 (toolchange at G30, fast probe pulloff)
- $394=1.0, $539=1.0 (spindle ramp times)
- $100-103=1600/1600/1600/88.889 (steps/mm)
- $110/$111=2000, $112=1500, $113=2000 (max rates)
- $120-123 accel, $130-133 max travel
- DEFAULT_AXIS_ROTATIONAL_MASK=8 (A rotary)

## Files modified in our work

- `boards/himill_d1s_map.h` — pin map per HiMill CSV + comments on EXTI constraints
- `boards/himill_d1s.c` — board init, [BOOT]/[DFU]/[ESP*]/[LASER_MODE]
  user commands, realtime hook for button polling, full USB-CDC ↔ UART2
  stream mux (covers all 15 io_stream_t methods incl. `cancel_read_buffer`
  for jog-cancel), fast-mode-always PD2 drive, pause-on-SD-job, optional
  toolsetter cleanup hook ($450) on `grbl.on_tool_selected`
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

## What's left

Decided / done since previous notes:

- Telnet vs WS — websocket is the canonical network path. Telnet is left as-is.
- Laser routing — `[LASER_MODE:0|1]` ships in firmware (PB0 TIM3_CH3 + PC5).
- Rotary axis — wired and exercised; user has the rotary working.
- Stream mux completeness — fixed (jog cancel, $esh long output).
- Toolsetter chip-cleanup — `$450=1` opt-in hook landed.
- Y axis direction — `$3=2` + `$23=1` bake mirrors stock D1S.

Still on the list:

1. **First-boot preset** — share/d1s/d1_factory.nc seeds G30 + G59.3
   per-machine. Once we see variance across 3+ testers, decide whether
   to keep the SD-card macro or move to firmware NVS seeding (#88).
2. **WS2812 LEDs** — PB5 ring + PB15 indicator. Flash budget is fine
   (~48 KB headroom); blocker is bit-bang timing under our IRQ load.
   Re-try via SPI1_MOSI on PB5 hardware — sidesteps the timing issue.
3. **`himill` utility script** consolidation — currently
   `himill_flash.py` only flashes. Add: `wifi`, `scan`, `status`, `ip`,
   optional static IP. Tracked as #79–#83/#85.
4. **`[ESP*]` tunnel close** — append newline so subsequent terminal
   prompts render cleanly (#86).
5. **Bootloader handshake race** — occasional flaky [BOOT] response
   needs port-reopen retry on the host side (#85).
6. **Per-tester `tc.macro`** if testers want full M6 replacement
   (alternative to the on_tool_selected hook). NGC_EXPRESSIONS is the
   only blocker — costs ~2.5 KB flash, fits only by giving up
   ROTATION_ENABLE or going COMPATIBILITY_LEVEL=2. Stay at the hook for
   now.

## Resume checklist

When resuming this work:

1. Read this README first
2. Check `our_findings/session_*.md` (when populated) for the most recent
   session's notes
3. Run `pio run -e himill_d1s` to confirm a clean build
4. Use `tools/himill_flash.py .pio/build/himill_d1s/firmware.bin` for normal
   USB bootloader flashing via `[BOOT]`
5. Use `tools/himill_flash.py wifi-setup` to scan/select WiFi over USB, and
   `tools/himill_flash.py wifi-status` to print current ESP3D SSID/IP
6. If using ST-LINK instead, use
   `st-flash --freq=100 --reset write .pio/build/himill_d1s/firmware.bin 0x08008000`
   (the slow speed avoids the loader bug)
7. Check `EGSender/docs/controllers/himill_d1s.md` for sender-side context
8. Memory file `project_d1s_firmware_flash.md` (auto-memory) has the
   session-to-session pointers
