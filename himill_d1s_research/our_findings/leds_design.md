# LED design for D1S (PB5 progress strip + PB15 inside-button)

Status: first-pass implementation in `boards/himill_d1s.c`.

## Hardware

Per HiMill IO CSV + map header:

- **PB5** — 24-LED WS2812 progress strip (signal-inverted)
- **PB15** — 1-LED WS2812 indicator inside the cycle-start button (signal-inverted)

Both are declared in `boards/himill_d1s_map.h` as `AUXOUTPUT5` / `AUXOUTPUT6`
(generic aux outputs, currently unused).

## Why this is non-trivial on STM32F1

WS2812 needs 800 kHz / ±150 ns timing per bit:
- '0' bit = 0.4 µs high + 0.85 µs low
- '1' bit = 0.8 µs high + 0.45 µs low
- reset = 50+ µs low

That's tight enough that plain GPIO writes from a CPU loop can't drive
it reliably — interrupts mid-pulse will violate timing and the strip
shows wrong colors.

## Three implementation paths

1. **Bit-bang with `__disable_irq()` around each refresh** — simplest
   code (~80 lines), CPU is locked for ~30 µs per LED (≈ 720 µs for the
   24-LED strip + 30 µs for the button LED). We'd refresh only on state
   changes, not in any tight loop, so a sub-millisecond IRQ block once
   per state transition is acceptable. Stepper IRQs do not fire during
   the refresh, so this should be watched on hardware during Run/Hold
   transitions.

2. **SPI-driven** — STM32 SPI MOSI driving carefully prescaled so each
   WS2812 bit = 3 SPI bits (e.g., `100` for 0, `110` for 1). Frees the
   CPU during transmission. **PB15 is SPI2 MOSI** — perfect for the
   button LED. **PB5 is NOT a default SPI pin** — would need bit-bang
   for the strip regardless. Mixed approach: SPI2 for PB15, bit-bang
   for PB5.

3. **TIM + DMA** — most efficient but most setup complexity. STM32F1
   has TIM2 / TIM3 with PWM capability + DMA streams that can update
   GPIO bytes. Overkill for refresh-on-state-change patterns.

Implementation: **path 1 (bit-bang)**. The refresh frequency is low
enough that the CPU cost is negligible. The D1S board code registers
`hal.rgb0` as the 24-pixel front strip and `hal.rgb1` as the single
inside-button pixel; `RGB_LED_ENABLE=2` enables the grblHAL M150 plugin
on top of those HAL callbacks. `$536` / `$537` default to 24 / 1 to
match the physical LED counts after a settings reset.

## Plug-in points in grblHAL

- `grbl.on_state_change` — fires when machine state transitions
  (Idle, Run, Hold, Alarm, Door, etc.). This is where the strip
  updates its color.
- `grbl.on_program_completed` — could pulse the strip green once.
- `grbl.on_realtime_report` — for progress-bar animations during long
  jobs (read planner buffer fill % to set LED count).

For the inside-button LED:
- `state_get() & STATE_TOOL_CHANGE` — light when waiting for M6 confirm
- A flag we set in `himill_stream_changed` (the pause-on-SD hook) when
  M1 is inserted at SD-stream-start — light while in HOLD from that
  flag, clear when CYCLE_START fires

## Default color scheme (proposed)

- IDLE → solid green at low brightness
- RUN  → blue, optionally progress-bar animation (1 LED per N% of plan)
- HOLD → yellow
- ALARM → red
- DOOR → magenta solid
- SAFETY_DOOR → orange
- CHECK_MODE → cyan
- ESTOP → red solid (same hardware indicator as ALARM, brighter)

The first pass uses solid colors only; blinking/progress animations are
left for a later non-blocking task.

## Inside-button LED logic

```c
bool button_led_should_be_on (void)
{
    sys_state_t s = state_get();

    // Tool change confirm wait
    if (s & STATE_TOOL_CHANGE)
        return true;

    // SD job paused at start (our pause-on-SD hook inserts M1)
    if ((s & STATE_HOLD) && himill_sd_pause_pending)
        return true;

    return false;
}
```

`himill_sd_pause_pending` set by our `himill_stream_changed` hook when
StreamType_SDCard kicks in; cleared when state leaves STATE_HOLD.

## Why this is lowest priority

- Machine works without LEDs
- WS2812 driver is real engineering effort
- Visual polish, not functional necessity
- Better to ship core functionality first, add LEDs as v0.x update

## Pick-up checklist for future work

1. Done: pick bit-bang approach.
2. Done: implement WS2812 send with `__disable_irq` + GPIO BSRR writes + NOP timing.
3. Done: hook `grbl.on_state_change` for front-strip state colors.
4. Done: light the inside-button LED while `STATE_TOOL_CHANGE` is active.
5. Pending: test with the physical 24-LED strip + button LED.
6. Optional: add settings for color customization after default behavior is proven.
