---
name: D1S firmware flash — replace HiMill fork with stock grblHAL
description: Long-term project. Working build exists. The major long-output and bootloader-entry blockers are solved; the main remaining network rough edge is ESP3D telnet behavior, while websocket works cleanly.
type: project
originSessionId: 49df1868-7915-4c09-b728-9443a485f48e
---

## Status (2026-04-25)

**Working build** at `/home/galin/Downloads/do_not_backup/cnc/STM32F1xx/.pio/build/himill_d1s/firmware.bin`. Multiple flashes done. Most features work end-to-end (motion, homing, probe, toolsetter, spindle ramp, SD card, A axis rotary, polled buttons, ESP3D bridge, bootloader entry).

**Major blockers solved:**

1. Long-output failure with `USB_SERIAL_CDC=1` was fixed by correcting the D1S stream mux in `boards/himill_d1s.c`.
2. `[BOOT]` now works by writing `0xFA0505F5` to `0x2000BFF0` before reset, matching the vendor app/bootloader handshake.

**Main remaining rough edge:** telnet on ESP3D port 23 is still noisy (duplicate banner / initial `error:1`), while websocket on `ws://<ip>:8282` is clean.

## Read-first when resuming

**`himill_d1s_research/README.md` in the firmware repo** is the canonical resume point — has full pin map, build flag list, all open issues with what's been ruled out, and next-session checklist. **Read that before doing anything else.** This memory file is a pointer; the README is the source of truth.

Path: `/home/galin/Downloads/do_not_backup/cnc/STM32F1xx/himill_d1s_research/README.md`

## Research artifacts (all in `himill_d1s_research/`)

- `maxmake_artifacts/` — vendor binary, custom commands CSV, pin map CSV, screenshots
- `bootloader/` — HiMill bootloader hex + bin + objdump (ready for Ghidra)
- `esp3d/` — vendor's ESP3D `configuration.h` + `platformio.ini`
- `our_findings/` — session writeups
- `disassembly/` — Ghidra staging area

## Current open items

1. **ESP3D telnet path** — likely telnet-daemon/session behavior on the ESP side. Websocket is clean and should be preferred unless telnet compatibility is worth extra work.
2. **MaxMake `DS:<bits>` behavior** — still potentially relevant if matching vendor UX/status semantics matters.
3. **Rotary / laser / LED follow-up work** — feature-completion rather than architecture blockers.

## Key invariants (don't lose these)

- App at 0x08008000 (`VECT_TAB_OFFSET=0x8000`). HiMill bootloader stays at 0x08000000.
- ST-LINK flash MUST use `--freq=100`; default speed hits a flash-loader bug on this RC variant.
- TMC drivers in standalone mode; PD2 driven HIGH at boot for fast mode (silent mode skipped — fan is always on).
- PA1 = TOOLSETTER, PB2 = PROBE (not what HiMill CSV says — CSV is misleading; map header has the truth).
- PB4 EXTI4 must stay clean for e-stop; PC4 dropped from aux pool for that reason.
- PB1/PC4 polled via `grbl.on_execute_realtime` shim, not IRQ-bound (EXTI line conflicts).

## Tasks superseded by README

The ad-hoc task list in TaskList is for in-session tracking. The README's "Next session" section is the durable plan.
