# Laser routing for D1S

Status: design captured; needs implementation + hardware test.

## Confirmed via testing (2026-04-25)

Setting `$32=1` (laser mode) does NOT redirect output to the laser
head. Result: spindle still spins. This is correct grblHAL behavior —
`$32=1` only changes timing semantics (no dwell, instant on/off, sync
with motion), not the physical output pin.

## Hardware on the D1S

Per HiMill IO CSV + verified silkscreen:

- **PB8** (TIM4_CH3) — main spindle PWM (CNC motor driver)
- **PB9** — main spindle enable (CNC motor)
- **PB0** (TIM3_CH3) — **laser PWM** (separate laser-head driver)
- **PC5** — laser enable

The two are physically separate driver chips. PB8/PB9 talks to the
CNC motor controller; PB0/PC5 talks to the laser-diode driver. Right
now our build only wires up PB8/PB9.

## Three implementation paths

### A. Two PWM spindles via SPINDLE1_ENABLE (recommended)

Standard grblHAL multi-spindle setup. User has CNC and laser as two
distinct spindles; selects active one via `$$` setting or M-code.

Build flags to add:
```
-D SPINDLE1_ENABLE=(SPINDLE_PWM|SPINDLE_ENA)
```

Map header additions:
```c
#define SPINDLE1_PWM_PORT      GPIOB
#define SPINDLE1_PWM_PIN       0      // PB0, TIM3_CH3
#define SPINDLE1_ENABLE_PORT   GPIOC
#define SPINDLE1_ENABLE_PIN    5      // PC5
```

Driver work needed: `Inc/driver.h`'s spindle PWM resolver currently
maps PB8 → TIM4_CH3. Need an analogous block for PB0 → TIM3_CH3 so
the second spindle gets a real PWM timer.

User workflow:
1. `$32=1` AND select spindle 1 → laser-mode behavior + laser PWM
2. `$32=0` AND select spindle 0 → CNC motor with normal spindle behavior

How spindle is selected at runtime: grblHAL setting `$x` (the spindle
selector) or via M-codes like M3 vs M4 + spindle setting. Senders that
support multi-spindle (gsender, ioSender) expose a UI for it.

### B. Single PWM spindle, switching pin on $32 change

Hook `grbl.on_settings_changed`. When `$32` flips:
- 1 → reconfigure spindle peripheral to drive PB0 instead of PB8
- 0 → drive PB8

Cleaner UX (one setting flip = laser ready) but custom code in
`himill_d1s.c` and the spindle peripheral has to be reinit'd at
runtime. Risk: re-init while motion in progress is dangerous.

### C. M-code switch (M-code triggers PD2 + spindle re-route)

`M52`-style custom M-code that the user invokes: "switch to laser mode".
The M-code handler:
- Drives PD2 high (already happens at boot for fast-mode-always)
- Switches spindle output to PB0
- Sets $32=1 internally
- Mirror `M53` for back-to-CNC

Like B but explicit, less error-prone.

## What MaxMake did

Their stock firmware exposes both. From the binary's strings:
- `[LASER_MODE:1]` — bracket command toggles laser mode (presumably
  also drives PD2 + switches PWM destination)

So they went with C (or equivalent) — explicit user command.

## Reality check: STM32F1xx driver has no SPINDLE1 support

Confirmed by inspection of `Src/driver.c` — the driver registers exactly
one spindle. There's no `DRIVER_SPINDLE1_ENABLE` code block that wires
up a second PWM peripheral. Setting `-D SPINDLE1_ENABLE=14` in the
build flags would just trigger the warning "Selected spindle 1 is not
supported!" and fail.

To make path A work we'd have to:

1. Duplicate `spindleConfig` for spindle 1 (driving TIM3 instead of
   TIM4) — handles PWM frequency, duty cycle init, GPIO AF config
2. Duplicate `spindleSetStateVariable`, `spindleSetSpeed`,
   `spindleGetPWM`, `spindleGetState` for spindle 1
3. Duplicate the `static const spindle_ptrs_t spindle = {...}` block
   for spindle 1 with `.ref_id = SPINDLE_PWM1_NODIR`
4. Add second `spindle_register(&spindle1, "Laser")` call
5. Add GPIO + clock setup for PB0 (TIM3_CH3) and PC5 (output)
6. Update `Inc/driver.h` to resolve `SPINDLE1_PWM_PIN=0` → TIM3_CH3
   timer/channel macros (`SPINDLE1_PWM_TIMER_N=3`, etc.)

Realistic LOC: ~200-300 lines, plus a non-trivial risk of breaking
the existing CNC spindle path during refactor. Not a couple-of-lines
change.

## Recommendation

**Defer to its own focused session.** Three reasons:

1. Real driver work, not a config tweak
2. Risk of breaking working CNC spindle if done in passing
3. Distribution can ship as "CNC-only firmware, laser support coming"

When picked up, path A is still the right architecture (clean
multi-spindle, well-supported by senders).

## Quick-and-dirty fallback (if user needs laser before A is done)

Custom M-codes in `boards/himill_d1s.c` that talk directly to the
laser hardware, bypassing grblHAL's spindle abstraction:

- `M62 P3` → drive PC5 high (laser enable on)
- `M63 P3` → drive PC5 low (off)
- `M67 E2 Q<intensity>` → set PB0 (TIM3_CH3) PWM duty cycle 0-100%

User writes laser gcode with explicit M-codes:
```
M62 P3      ; laser enable
M67 E2 Q50  ; 50% power
G1 X10 F1000 ; cut
M63 P3      ; off
```

**Drawbacks vs. A:**
- No `$32=1` laser-mode timing semantics (no automatic
  power-on-S>0, no sync with motion). User manages explicitly.
- Senders' laser-mode UI doesn't apply
- More verbose gcode
- But ~30-50 lines instead of 300

## Why path A is still the right answer

Eventually we'll want proper grblHAL laser support — better sender
integration, automatic power scaling with feed rate, dwell-free
operation during cutting. That's worth the driver work when scheduled
as a focused effort.

User flow:
1. CNC: select spindle 0 (default), use M3/M4/M5
2. Laser: in $$ or via sender UI, switch to spindle 1 + set $32=1.
   M3 commands now drive the laser.

## Implementation checklist (when picked up)

1. Map header: add `SPINDLE1_PWM_PORT/PIN` + `SPINDLE1_ENABLE_PORT/PIN` (free PB0/PC5 from `AUXOUTPUT2/3`)
2. `Inc/driver.h`: add PB0/TIM3_CH3 branch in spindle PWM resolver
3. `platformio.ini`: add `-D SPINDLE1_ENABLE=(SPINDLE_PWM|SPINDLE_ENA)`
   (or numeric equivalent — check exact value)
4. Build & test:
   - `$$` shows separate $30/$31 etc. for spindle 1
   - Selecting spindle 1 + M3 fires laser PWM on PB0
   - PC5 goes high on M3, low on M5
5. Laser-fast-mode interaction: PD2 already driven HIGH at boot
   (unconditionally). User doesn't need to do anything for fast mode.

## Possible complication: PD2 STEPPER_MODE timing

We currently drive PD2 HIGH unconditionally at boot. If MaxMake's
laser path expected PD2 to TOGGLE on laser-mode-on/off (rather than
permanent fast-mode), the laser driver might behave unexpectedly.
Worth verifying: does the laser fire correctly with PD2 already high?
If not, `[LASER_MODE]` M-code could explicitly toggle PD2.
