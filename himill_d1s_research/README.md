# HiMill D1S grblHAL Research

This folder collects everything learned during the bring-up of grblHAL on
the HiMill D1S CNC controller (2026-04-24 → 2026-05-03). Use it as the
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
- `grblhal_setting_descriptions.json` — exported `setting_descr_t`
  defaults for sender-side fallback when firmware is built with
  `NO_SETTINGS_DESCRIPTIONS`; keeps preprocessor-gated variants and
  `appliesTo` hints for normalized axis / repeated settings.

## What works (current build state, 2026-05-03)

These all work cleanly on our grblHAL build:

- Motion: X/Y up to 2000 mm/min, Z 1500 mm/min (default $451=1 = SpreadCycle)
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
- **SmartPendant on UART4 / GX12-4P rear connector** (added 2026-04-30):
  `SERIAL1_PORT=4` + `MPG_ENABLE=2` + `MPG_STREAM=1`. Pendant button
  sends `0x8B` to engage MPG mode; wheel produces `$J=` jogs; pendant
  LCD shows axes/coords. Internal pull-up on PC11 (UART4 RX) is
  required so the pin idles HIGH when nothing is plugged in (`Src/
  serial.c:828-833`). 500 ms `0x8B` debounce in `himill_intercept_rt`
  + `himill_mpg_rx_filter` to swallow pendant heartbeat retries.
  **Critical fix**: `mux_write_all` MUST broadcast to the MPG stream
  while `sys.mpg_mode` is true — otherwise the pendant never sees
  `|MPG:1` in status replies, retries `0x8B`, and toggle-loops at
  ~0.5 Hz. See `bug_ws_dollar_doubling.md` and
  `session_2026-04-30_pendant.md` for the diagnosis chain.
- **Per-UART TX buffer sizing** (added 2026-04-30): the stock driver uses
  one global `TX_BUFFER_SIZE` for every UART. Two 8 KB buffers (UART2
  + UART4) blew through the F103RC's 48 KB SRAM and caused $-dump
  truncation + hard crashes. Introduced `TX_BUFFER1_SIZE=1024` for
  UART4 (`Src/serial.c`).
- **Factory-default NVS seeding** (added 2026-05-02): `seed_himill_
  coord_defaults()` writes `G30=(30,-7,0)` and `G59.3=(0,-7,-15)` if
  either is at zeros. Called from both `himill_settings_load` (every
  boot — handles fresh flash + post-`$RST=#` reboot) and
  `himill_settings_restore` (`$RST=*` / `$RST=&` / first-boot
  version-mismatch path). Idempotent.
- **`$342=65` baked default** (added 2026-05-02): tool-change probing
  distance increased from grblHAL's stock 30 mm to 65 mm to span the
  search range needed with `G59.3.Z=-15`.
- **`$451` user-settable TMC chopper mode** (added 2026-05-03):
  `Setting_UserDefined_1`, `Format_RadioButtons` "StealthChop,
  SpreadCycle". Default 1 (SpreadCycle). Drives PD2 (TMC STEPPER_MODE)
  in `apply_stepper_mode()`. **Empirical: StealthChop hits a back-EMF
  wall at ~1000 mm/min on D1S** — drops steps above that. SpreadCycle
  sustains the configured 2000 mm/min ceiling. No hybrid mode (TMC's
  `TPWMTHRS` register) available — D1S TMCs are standalone, no UART
  to chip. Manual mode-flip is the only knob.
- **`$452` RGB status LEDs** (added 2026-05-10):
  `Setting_UserDefined_2`, `Format_Bool`. Default 0 (disabled) so the
  D1S default path performs no WS2812 writes. Set `$452=1` to enable
  automatic state colors and `M150`.
- **Boot-time D1S profile repair** (added 2026-05-10):
  board init repairs critical machine-specific settings and prints
  `[MSG:D1S repaired]` if anything changed. Forced values: `$10=6143`,
  `$650=3`, `$20=1`, `$21=1`, `$40=1`, `$65=10`, `$341=3`, `$342=65`,
  `$346=7`. `$22` is not forced wholesale; only the "set machine origin
  to 0" bit is forced on so other homing options remain user-configurable.

## RGB LED / M150 reference

The D1S build exposes two WS2812 RGB outputs:

| Output | Pin | LEDs | Use |
|--------|-----|------|-----|
| `S0` / `hal.rgb0` | PB5 | 24 | Front LED strip; automatic machine-state color |
| `S1` / `hal.rgb1` | PB15 | 1 | Inside cycle button LED; amber during tool-change wait |

RGB output is runtime-gated by `$452`:

```gcode
$452=0 ; disable all D1S WS2812 writes (default)
$452=1 ; enable automatic state colors and M150
```

Automatic RGB updates are deferred during jogging so a WS2812 refresh
cannot block UART receive while `$J=` lines are arriving. The idle color
is applied after jogging has been quiet briefly.

Automatic front-strip state colors from `boards/himill_d1s.c`:

| State | RGB | Hex | Meaning |
|-------|-----|-----|---------|
| E-stop | `96, 0, 0` | `#600000` | Emergency stop active |
| Alarm | `64, 0, 0` | `#400000` | Alarm / locked state |
| Tool change | `64, 28, 0` | `#401C00` | Waiting for manual tool change / resume |
| Safety door | `60, 18, 0` | `#3C1200` | Safety door condition |
| Hold | `56, 40, 0` | `#382800` | Feed hold / paused |
| Homing | `32, 32, 32` | `#202020` | Homing cycle running |
| Jog | `28, 0, 48` | `#1C0030` | Jogging |
| Cycle / Run | `0, 0, 64` | `#000040` | Program or motion running |
| Check mode | `0, 40, 40` | `#002828` | Check / dry-run mode |
| Sleep | `0, 0, 0` | `#000000` | Asleep / LEDs off |
| Idle | `0, 32, 0` | `#002000` | Ready / idle |

The button LED is `#401C00` (`64, 28, 0`) only while
`STATE_TOOL_CHANGE` is active; otherwise it is off.

`M150` can manually set LED colors. Syntax used here:

- `S0` = front strip, `S1` = button LED
- `R` = red, `U` = green, `B` = blue, each `0..255`
- `P` = brightness / intensity, `0..255`
- `I` = front-strip pixel index, `0..23`

Examples:

```gcode
; Front 24-LED strip: dim blue
M150 S0 R0 U0 B255 P64

; Front strip: low green / idle-like
M150 S0 R0 U32 B0

; Front strip: red
M150 S0 R96 U0 B0

; Front strip: turn off
M150 S0 R0 U0 B0

; Inside cycle button LED: amber/tool-change color
M150 S1 R64 U28 B0

; Inside cycle button LED: off
M150 S1 R0 U0 B0

; Set LED index 5 on the front strip to purple
M150 S0 I5 R28 U0 B48
```

Manual `M150` colors are overwritten the next time grblHAL reports a
machine-state change and the D1S state-color hook refreshes the LEDs.
`M150` is ignored while `$452=0`.

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

### 6. StealthChop velocity ceiling on D1S

User-empirical (2026-05-03): `$451=0` (StealthChop) starts dropping
steps above ~1000 mm/min. `$451=1` (SpreadCycle) sustains the
configured 2000 mm/min on $110/$111. Cause: TMC StealthChop's
PWM tuning loop hits a back-EMF wall at the D1S supply voltage (24 V?)
+ the specific motors. On boards with UART/SPI to the TMC, the
`TPWMTHRS` register switches StealthChop → SpreadCycle automatically
above a programmed velocity — but D1S TMCs are standalone, that
register is unreachable. So StealthChop is a **manual mode for slow
operations only** (probing, drag-knife, quiet jogging). Default stays
SpreadCycle.

MaxMake's stock firmware uses StealthChop for milling and SpreadCycle
for laser — likely paired with a lower max-feed config in milling
mode. Our `$110/$111=2000` config + StealthChop combination is what
exposes the ceiling.

### 7. Bootloader handshake variant

A second tester's D1S bootloader replied `a1 02 01` to our `51 05 0a`
handshake (`tools/himill_flash.py:86`) instead of the
reverse-engineered `a1 04 01`. Framing matched (same `a1` opcode +
`01` trailer), only the middle byte differed (`02` vs `04`).
Hypothesis: middle byte is a hardware/bootloader variant identifier.
Awaiting HiMill clarification on what the byte represents and whether
the chunk-upload protocol is identical across variants. If yes, plan
is to add `--allow-variant` flag to the flasher that warns instead of
erroring. 2026-05-01.

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
| `NO_SETTINGS_DESCRIPTIONS` | 1 | Strip long core setting descriptions from `$ESH` / per-setting help |
| `NGC_EXPRESSIONS_ENABLE` | 1 | Enable NGC parameter expressions for probing macros |
| `SDCARD_ENABLE` | 1 | $F, $FM |
| `RGB_LED_ENABLE` | 2 | WS2812 state LEDs plus M150 plugin |
| `VECT_TAB_OFFSET` | 0x8000 | Match HiMill bootloader's app entry |
| `SERIAL1_PORT` | 4 | UART4 on PC10/PC11 (GX12-4P rear connector → SmartPendant) |
| `MPG_ENABLE` | 2 | Software MPG toggle (no MPG_MODE_PIN) |
| `MPG_STREAM` | 1 | Pendant stream = UART4 (instance 1; UART2 is instance 0) |

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
- $342=65 (toolchange probing distance; stock grblHAL was 30 — needed
  larger range with G59.3.Z=-15)
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
  for jog-cancel), pause-on-SD-job, optional toolsetter cleanup hook
  ($450) on `grbl.on_tool_selected`. **Added 2026-04-30 → 2026-05-03:**
  SmartPendant integration (`himill_mpg_rx_filter`,
  `himill_intercept_rt`, `0x8B` debounce, MPG fan-out in
  `mux_write_all`), factory NVS seeding for G30/G59.3
  (`seed_himill_coord_defaults`), user-settable TMC chopper mode (`$451`
  via `apply_stepper_mode` + `Setting_UserDefined_1`).
- `Inc/driver.h` — added PB8 TIM4_CH3 to spindle PWM resolver, added
  BOARD_HIMILL_D1S include
- `Src/driver.c` — added `#ifdef COOLANT_FLOOD_PIN` guard
- `Src/serial.c` — **(added 2026-04-30)** per-UART TX buffer typedef
  with `TX_BUFFER1_SIZE=1024` for UART4 (single global TX_BUFFER_SIZE
  doesn't fit two 8 KB buffers in F103RC's 48 KB SRAM). `GPIO_PULLUP`
  on UART1/UART4 RX (PC11) — without it, MPG_ENABLE=2 routes
  floating-pin noise into `protocol_enqueue_realtime_command` and
  random feed-holds / resets fire.
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
- **First-boot preset** — done in firmware via `seed_himill_coord_
  defaults()` (NVS seed, no SD macro). Covers fresh flash, `$RST=*`,
  `$RST=&`, and `$RST=#` (re-seeds on next boot). 2026-05-02.
- **SmartPendant via UART4 GX12-4P** — bring-up complete, see "What
  works" entry. 2026-04-30.
- **`$342=65` baked default** — 2026-05-02.
- **TMC chopper user setting (`$451`)** — 2026-05-03.

Still on the list:

1. **WS2812 toolchange indicator hardware validation** — RGB state
   output and M150 are compiled in; still test the internal LED path
   during an actual toolchange wait-for-cycle-resume sequence.
2. **`himill` utility script** consolidation — currently
   `himill_flash.py` only flashes. Add: `wifi`, `scan`, `status`, `ip`,
   optional static IP. Tracked as #79–#83/#85. Wifi setup partially
   landed (commit `ae53d0b`, "wifi scan and setup").
3. **`[ESP*]` tunnel close** — append newline so subsequent terminal
   prompts render cleanly (#86).
4. **Bootloader handshake race** — occasional flaky [BOOT] response
   needs port-reopen retry on the host side (#85).
5. **Bootloader handshake variant** — second tester's D1S bootloader
   replied `a1 02 01` to our `51 05 0a` handshake instead of the
   reverse-engineered `a1 04 01`. Framing matches (same `a1` opcode +
   `01` trailer) — middle byte is plausibly a hardware/bootloader
   variant identifier. Awaiting HiMill clarification on what the byte
   represents and whether the chunk-upload protocol is identical
   across variants. 2026-05-01. If protocol is the same, plan is to
   add `--allow-variant` flag to `himill_flash.py` that warns instead
   of erroring on the byte mismatch. See `tools/himill_flash.py:86`.
6. **Door safety** — PB1 door switch is currently dispatched as
   `safety_door_ajar` through `hal.control.interrupt_callback` from
   the polled-button hook (`himill_poll_buttons`) but full safety-
   door modal integration not wired. User has stated low priority
   ("don't care enough" 2026-04-30).
7. **Per-tester `tc.macro`** if testers want full M6 replacement
   (alternative to the on_tool_selected hook).

## Build constraints / flash budget

Current known-good usage on `[env:himill_d1s]` with RGB LEDs, macros, and
NGC expressions enabled, with long core setting descriptions stripped:

```
RAM:   62.5% (30 716 / 49 152 bytes)
Flash: 85.2% (223 220 / 262 144 bytes)
```

Actual D1S app flash budget is 225,280 bytes because the linker reserves
the HiMill app offset plus EEPROM emulation. The current expression-enabled
`firmware.bin` is 223,716 bytes, leaving 1,564 bytes of real flash headroom.

### NGC_EXPRESSIONS_ENABLE — fits with setting descriptions stripped

The grblHAL feature flag `NGC_EXPRESSIONS_ENABLE` (parameter expressions
like `#1 = 5` and `[#1 + 2]`) now fits without disabling RGB, macros,
MPG, SD card, rotation, or the 4 KB EEPROM/NVS reserve.

Final tested build, 2026-05-03:

- `NO_SETTINGS_DESCRIPTIONS=1` wraps the large core `setting_descr[]`
  table in `grbl/settings.c` and registers no core description table.
- `$ES` / `$ESH` still enumerate IDs, names, groups, units, types,
  formats, bounds, reboot flags, and values. The long description field
  is blank for core settings.
- Per-setting help (`$$=<n>` / `$SED=<n>`) loses the long explanatory
  paragraph for core settings. Sender-side fallback text is the right
  place for this.
- No LTO is used.
- Size: `text=219364`, `data=4348`, `bss=35592`,
  `firmware.bin=223716`.
- Real app budget: `0x37000` / 225,280 bytes, so slack is 1,564 bytes.
- IRQ sanity check passed: `USB_LP_CAN1_RX0_IRQHandler`,
  `SysTick_Handler`, `TIM2_IRQHandler`, `USART2_IRQHandler`, and
  `UART4_IRQHandler` all resolve to real handlers, not `Default_Handler`.

Rejected / cautionary build paths:

- Plain `NGC_EXPRESSIONS_ENABLE` without stripping setting descriptions
  failed at link in scratch builds.
- Adding `-flto` made earlier NGC builds fit, but the resulting image was
  unsafe on hardware.
- LTO exposes a duplicate `_exit` between `Src/syscalls.c` and
  `libnosys.a`; making the local `_exit` shim weak resolves that without
  using `--allow-multiple-definition`.
- Hardware result after flashing the LTO image: USB full-speed pull-up
  appears, but enumeration fails during descriptor/control transfers
  (`device descriptor read/64, error -32`, then `error -71`).
- ELF inspection showed the root cause: vector-table entries such as
  `USB_LP_CAN1_RX0_IRQHandler`, `SysTick_Handler`, `TIM2_IRQHandler`,
  `USART2_IRQHandler`, and `UART4_IRQHandler` resolved to the startup
  file's weak `Default_Handler` instead of the real C handlers.
- A follow-up scratch build used D1S-only non-LTO IRQ wrappers so those
  vector names resolved to real handlers. Once the real handlers were
  retained, the all-features image no longer fit: linker overflow was
  2,580 bytes.

### General flash discipline

When adding features:

- Audit before enabling new core flags — most cost 1-3 KB.
- Static `const` strings for setting names/descriptions cost flash
  too; reuse existing strings where reasonable.
- Avoid `printf` / `snprintf` family pulled in from a single new path —
  the float-formatting variants pull in 8+ KB of libc.

## Resume checklist

When resuming this work:

1. Read this README first
2. Check `our_findings/session_*.md` for chronological notes; the most
   recent (as of this writing) is
   `session_2026-04-30_to_05-03_pendant_defaults_chopper.md` covering
   the SmartPendant integration, NVS factory seeding, `$342` baked
   default, and the `$451` TMC chopper-mode setting.
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
9. Auto-memory invariants worth preserving (see `memory/MEMORY.md`):
   - `project_d1s_mux_write_all_mpg.md` — `mux_write_all` MUST broadcast
     to the MPG stream while `sys.mpg_mode` is true. Regression
     symptom: alternating `|MPG:1|` / `|MPG:0|` at ~0.5 Hz, pendant
     unresponsive.
   - `project_user_smartpendant.md` — pendant retries `0x8B` if it
     never sees `|MPG:1` in status replies. Hostile to any wrapper
     that swallows status reports.
   - `project_d1s_back_gx12_uart4.md` — GX12-4P pinout for the rear
     pendant connector.
