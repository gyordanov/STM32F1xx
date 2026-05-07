/*
  himill_d1s_map.h - grblHAL pin map for the HiMill D1 / D1S CNC main board.

  Part of grblHAL (STM32F1xx driver).

  Source of truth: HiMill's official IO pin description CSV (care package
  2026-04-24). Every signal's port/pin/active-level below comes directly
  from that doc — cross-reference:
      docs/controllers/himill_d1s/IO_Pin_Description.CSV (vendored)

  Board identification (verified 2026-04-24 by silkscreen photo):
    MCU:     STM32F103RCT6    — LQFP-64, 256 KB flash, 48 KB RAM
    HSE:     8.000 MHz        — Hosonic HX-C 8.0 SMD crystal
    Startup: StartupRC/startup_stm32f103rctx.s (existing)
    Linker:  STM32F103RCTX_HIMILL_D1S_FLASH.ld

  Bootloader offset (verified against MaxMake v1.0.34 official app):
    Bootloader code: 0x08000000-0x08003E10 (16 KB used, 32 KB reserved)
    App entry:       0x08008000 — reset vector 0x08008148 in their .bin
                     confirms the jump target. The 16 KB gap between
                     bootloader code end and app start is reserved by
                     HiMill (scratch / settings / forced alignment).
    Our build uses VECT_TAB_OFFSET=0x8000 in platformio.ini to match.

  TMC drivers run in STANDALONE mode on this board. STEPPER_MODE (PD2)
  is a single digital pin that toggles the TMC between silent (CNC) and
  fast (laser) — NOT a UART configuration line. Trinamic plugin should
  stay disabled; expose STEPPER_MODE as an aux output instead so a
  plugin / $M-code can toggle it.
*/

#if N_ABC_MOTORS > 1
#error "HiMill D1/D1S has 4 axes (X/Y/Z/A). Set N_ABC_MOTORS to 1 in the build env."
#endif

#if TRINAMIC_ENABLE
#error "HiMill D1/D1S TMC drivers are in STANDALONE mode. Leave TRINAMIC_ENABLE off."
#endif

// Note: the STM32F103RCT6 uses the ST HAL "high density" variant
// (STM32F103xE) — counterintuitive, but the RC chip shares the
// high-density peripheral set despite being 256 KB / "C" grade. There
// is no STM32F103xC macro in ST's HAL. Enforcement is handled at the
// platformio.ini level (board=genericSTM32F103RC selects the right
// startup + linker + HAL flags), so no guard here.

#ifndef BOARD_NAME
#define BOARD_NAME "HiMill D1/D1S"
#endif
#define BOARD_URL "https://github.com/galin-yordanov/EGSender/tree/main/docs/controllers/himill_d1s"

// Triggers driver.c → board_init() at startup; implemented in
// boards/himill_d1s.c. Wires the [BOOT] command handler so MaxmakeLAB
// and himill_flash.py can trigger bootloader re-entry for firmware
// updates.
#define HAS_BOARD_INIT

// -------- Step pulses --------
// HiMill CSV rows 1-4. All four step pulses share GPIOB. Use shared-
// port `STEP_PORT` + GPIO_MAP so driver.c can write all step signals
// in a single register touch. Per-axis `*_STEP_PORT` is NOT defined —
// the driver infers them from STEP_PORT when pins are all on one port.
#define STEP_PORT                GPIOB
#define X_STEP_PIN               6
#define Y_STEP_PIN               14
#define Z_STEP_PIN               13
#define STEP_OUTMODE             GPIO_MAP

// -------- Step direction --------
// HiMill CSV rows 5-8. All four dir pins share GPIOC. Same shared-
// port pattern as STEP_PORT above. High = positive direction —
// grblHAL's $3 DIR_SIGNALS_INVERT_MASK handles axis inversion at
// runtime (MaxMake has $3=2 → Y inverted).
#define DIRECTION_PORT           GPIOC
#define X_DIRECTION_PIN          12
#define Y_DIRECTION_PIN          8
#define Z_DIRECTION_PIN          7
#define DIRECTION_OUTMODE        GPIO_MAP

// -------- A axis (M3) --------
// HiMill CSV rows 4 + 8. A axis has step + dir but NO dedicated limit
// switch (CSV only lists X/Y/Z limits). A axis is rotary on the D1S.
// STEP/DIRECTION PORTS inherit from the shared defines above.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT             STEP_PORT
#define M3_STEP_PIN              12
#define M3_DIRECTION_PORT        DIRECTION_PORT
#define M3_DIRECTION_PIN         6
#endif

// -------- Stepper enable --------
// HiMill CSV row 9. Single shared enable for X/Y/Z (A axis not mentioned
// — assume enable also gates A). Active-HIGH per HiMill doc, but
// grblHAL's "enable" convention is driver-side disable. Flip via
// $4 (ENABLE_SIGNALS_INVERT_MASK) at runtime.
#define STEPPERS_ENABLE_PORT     GPIOA
#define STEPPERS_ENABLE_PIN      15

// -------- Homing / limit switches --------
// HiMill CSV rows 11-13. Shared-port pattern — all three limits on GPIOC.
#define LIMIT_PORT               GPIOC
#define X_LIMIT_PIN              0
#define Y_LIMIT_PIN              1
#define Z_LIMIT_PIN              2
#define LIMIT_INMODE             GPIO_MAP

// -------- Aux outputs / spindle / laser / TMC mode toggle --------
// HiMill CSV rows 10, 14-17, 20-21. The board has two independent
// PWM heads (spindle + laser), both active-LOW PWM. grblHAL's primary
// spindle routes through AUXOUTPUT0/AUXOUTPUT2 below; the laser gets
// AUXOUTPUT3/AUXOUTPUT4 so plugin code can drive it when in laser
// mode ($32=1) or expose it as a second spindle.
#define AUXOUTPUT0_PORT          GPIOB // Spindle PWM  (PB8, TIM4_CH3)
#define AUXOUTPUT0_PIN           8
#define AUXOUTPUT1_PORT          GPIOB // Spindle enable (PB9)
#define AUXOUTPUT1_PIN           9
#define AUXOUTPUT2_PORT          GPIOB // Laser PWM (PB0, TIM3_CH3)
#define AUXOUTPUT2_PIN           0
#define AUXOUTPUT3_PORT          GPIOC // Laser enable (PC5)
#define AUXOUTPUT3_PIN           5
#define AUXOUTPUT4_PORT          GPIOD // TMC STEPPER_MODE (PD2): L=silent/CNC, H=fast/laser
#define AUXOUTPUT4_PIN           2
#define AUXOUTPUT5_PORT          GPIOB // WS2812 progress strip (PB5, 24 LEDs; signal-inverted)
#define AUXOUTPUT5_PIN           5
#define AUXOUTPUT6_PORT          GPIOB // WS2812 aux LED (PB15, 1 LED; signal-inverted)
#define AUXOUTPUT6_PIN           15

// -------- Driver spindle wiring --------
// Route the primary spindle to AUXOUTPUT0/AUXOUTPUT1. PWM is active-low
// at the pin; grblHAL's SPINDLE_PWM_INVERT setting ($1x) handles that.
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT      AUXOUTPUT1_PORT
#define SPINDLE_ENABLE_PIN       AUXOUTPUT1_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT_BASE    GPIOB_BASE
#define SPINDLE_PWM_PORT         AUXOUTPUT0_PORT
#define SPINDLE_PWM_PIN          AUXOUTPUT0_PIN
#endif

// -------- Aux inputs / reset / toolsetter / probe / button --------
// Per HiMill CSV labels (now adopted as authoritative since MaxMake's
// remap was just a workaround for not exposing dual-probe support):
//   - PA1 = TOOL_SETTER (touch plate on the bed for TLO measurement)
//   - PB2 = 3D_PROBE   (real 3D probe / touch probe in spindle)
//   - PB1 = SAFETY_DOOR / inside-door button (dual purpose in HiMill HW)
//   - PB4 = RESET / e-stop (not in CSV — only in MaxMake $pins)
//   - PC4 = front-panel BUTTON — DROPPED from map (would steal EXTI4 from
//           e-stop on PB4; user opted to abandon the external button).
//
// STM32F103 EXTI constraint: each EXTI line N (0..15) listens to exactly
// ONE port. With X/Y/Z limits on PC0/PC1/PC2, EXTI 0/1/2 are claimed by
// limits — any aux input on PA1/PB1/PB2/PC1/PC2 collides. The driver
// (driver.c:1050) forces colliding aux inputs into IRQ_Mode_None mode,
// then (line 1054-1056) only assigns the function ID if it's a probe-
// type input. So:
//   * PROBE / TOOLSETTER / PROBE2 → polled, work fine
//   * SAFETY_DOOR / CYCLE_START / FEED_HOLD → cannot bind on this board
//     (would degrade to generic Aux). Use the macros plugin instead for
//     polled equivalents.
#define AUXINPUT0_PORT           GPIOB // Reset / e-stop (PB4): per MaxMake $pins
#define AUXINPUT0_PIN            4
#define AUXINPUT1_PORT           GPIOA // TOOL_SETTER (PA1): touch plate on bed
#define AUXINPUT1_PIN            1
#define AUXINPUT2_PORT           GPIOB // 3D_PROBE (PB2): touch probe in spindle
#define AUXINPUT2_PIN            2
#define AUXINPUT3_PORT           GPIOB // PB1: door switch — generic aux input.
#define AUXINPUT3_PIN            1
#define AUXINPUT4_PORT           GPIOC // PC4: front-panel + inside-door BUTTONS (parallel-
#define AUXINPUT4_PIN            4     //   wired). Polled only — driver.c's expanded
                                       //   DRIVER_IRQMASK forces irq_mode=None because
                                       //   pin 4 collides with PB4 e-stop on EXTI4.
                                       //   Bind to cycle-start via the macros plugin.

#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT               AUXINPUT0_PORT
#define RESET_PIN                AUXINPUT0_PIN
#endif

#if TOOLSETTER_ENABLE
// Toolsetter on PA1 — used for tool-length offsets. Polled (EXTI1 belongs
// to Y limit on PC1).
#define TOOLSETTER_PORT          AUXINPUT1_PORT
#define TOOLSETTER_PIN           AUXINPUT1_PIN
#endif

#if PROBE_ENABLE
// Primary 3D probe on PB2 — used by G38.x work-edge probing. Polled
// (EXTI2 belongs to Z limit on PC2).
#define PROBE_PORT               AUXINPUT2_PORT
#define PROBE_PIN                AUXINPUT2_PIN
#endif

// -------- SD card (SPI1 primary, no remap) --------
// HiMill CSV rows 24-27. SPI1 primary pins: SCK=PA5, MISO=PA6, MOSI=PA7.
// This differs from generic_map.h which uses SPI1-remap (PB3/PB4/PB5).
// DO NOT define SPI1_REMAP here.
#if SDCARD_ENABLE
#define SD_CS_PORT               GPIOA
#define SD_CS_PIN                4
// Reference — actual init happens in HAL_SPI_MspInit:
#define SD_IO_PORT               GPIOA
#define SD_SCK_PIN               5
#define SD_MISO_PIN              6
#define SD_MOSI_PIN              7
#endif

// -------- USB (full-speed) --------
// HiMill CSV rows 28-29. PA11/PA12 are the standard STM32F103 USB FS
// pins — nothing to configure here; USB_SERIAL_CDC=1 in my_machine.h
// activates the existing USB CDC stack.

// -------- UART to ESP32 WiFi module --------
// HiMill CSV rows 30-31. USART2 on PA2 (TX) / PA3 (RX), 115200. ESP3D
// runs on the ESP32 side; grblHAL talks to it as a secondary stream.
// Not a pin-map concern beyond leaving USART2's pins free of other
// use. Verify TIM2 channel 3/4 remap does NOT steal PA2/PA3 at
// runtime (we aren't using TIM2 for step pulses on this board —
// steppers live on TIM3 via GPIOB).

// -------- $ defaults live in platformio.ini --------
// All DEFAULT_* macro overrides for this board are passed via build flags
// in `[env:himill_d1s] build_flags`. Reason: settings.c (where defaults
// are consumed) only includes `hal.h`, never `driver.h` or this map
// header. `#define DEFAULT_X` here is silently ignored. Build flags
// reach every translation unit, including settings.c. Bake any new
// defaults there, not here.

// Tuning notes for the $5 invert mask (=7):
//   HiMill CSV says "Low level trigger" for all three limit switches,
//   which matches grblHAL's default wiring (pullup + NO switch → LOW
//   when closed). That would suggest $5=0. MaxMake ships $5=7 (all
//   three inverted) — most likely because the mechanical switches are
//   normally-closed (pin sits LOW untriggered, goes HIGH on trigger).
//   This is a common CNC choice for failsafe behavior (a cut wire looks
//   like a triggered limit). We inherit MaxMake's value because their
//   firmware is verified to work on the real board.

/*
  Unmapped CSV entries (functions with no current grblHAL equivalent):
    - LED_1 (PB5, WS2812 24-LED progress strip, signal-inverted)
    - LED_2 (PB15, WS2812 1-LED aux, signal-inverted)
  Both exposed as AUXOUTPUT5/6 above. A status-LED plugin (or HAL
  RGB-ring plugin) can drive them. For a first bring-up leave them
  dark — no functional regression.

  Commands HiMill documented but NOT wired by this map:
    - [SET_OFFSET]   — pure gcode macro, no MCU side.
    - [LASER_MODE:…] — expected to toggle STEPPER_MODE (PD2) and
                       route spindle→laser PWM. Wire up once we're on
                       native grblHAL modes ($32=1).
    - [SD_FORMAT], [SD_FREE], [SD_FILE_CANCEL], [SD_FILE_READY:…] —
                       HiMill's ESP3D-side SD commands. grblHAL's
                       standard sdcard plugin has its own command set
                       (`$F`, `$FM` etc.). Reconcile on the sender
                       side rather than in firmware.
    - `?` → DS:<bits> status reply — replaced by stock grblHAL's
                       `?` realtime status. Sender (EGSender) will see
                       the stock format.
*/
