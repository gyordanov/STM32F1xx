/*
  himill_d1s.c — board-specific code for the HiMill D1 / D1S.

  Part of grblHAL.

  Provides the `[BOOT]` bracket-command handler that MaxmakeLAB (and
  our own himill_flash.py) uses to trigger firmware updates. On
  receipt of `[BOOT]`, hand control back to HiMill's stock bootloader
  at 0x08000000 so the bootloader's USB-CDC protocol can accept a
  new firmware image. See EGSender/docs/controllers/himill_d1s.md
  for the full bootloader protocol.

  Invoked by grblHAL core via `board_init()` at startup when
  `HAS_BOARD_INIT` is defined in the board map header.
*/

#include "driver.h"

#if defined(BOARD_HIMILL_D1S)

#include <string.h>
#include "grbl/hal.h"
#include "grbl/gcode.h"
#include "grbl/stream.h"
#include "grbl/protocol.h"
#include "grbl/motion_control.h"
#include "grbl/state_machine.h"
#include "grbl/nvs_buffer.h"
#include "grbl/system.h"

// HiMill stock bootloader lives at 0x08000000. It waits for a USB-CDC
// handshake (0x51 0x05 0x0a) within a short window before jumping to
// the app at 0x08008000. Earlier attempts used NVIC_SystemReset() to
// trigger bootloader entry, but that caused the bootloader to jump
// back to our app immediately — we'd need an unknown magic flag to
// tell it to stay in boot mode.
//
// Direct-jump approach instead: set up the CPU to enter the
// bootloader's reset vector as if we'd arrived via a fresh power-up.
// Point VTOR at the bootloader's vector table, load the stack pointer
// from its first word, then branch to its reset handler from the
// second word. The bootloader's init code runs next; whatever it
// expects at power-on, it sees now.
#define HIMILL_BOOTLOADER_BASE 0x08000000UL
#define HIMILL_BOOTFLAG_ADDR   ((volatile uint32_t *)0x2000BFF0UL)
#define HIMILL_BOOTFLAG_MAGIC  0xFA0505F5UL

// STM32F103's factory-burned ROM bootloader (System Memory region).
// Speaks standard USB DFU → dfu-util works out of the box on any OS.
// Independent of HiMill's bootloader — OS-agnostic recovery path even
// if HiMill's bootloader gets corrupted.
#define STM32_ROM_BOOTLOADER   0x1FFFF000UL

// `[BOOT]` and `[DFU]` are bracket commands. grblHAL routes lines starting
// with `[` through `grbl.on_user_command` (see protocol.c:254), NOT
// `grbl.on_unknown_sys_command` (which only fires for `$`-prefixed lines).
// Registering on the wrong hook silently drops the command and the parser
// returns error:1 ("Expected command letter").
static on_user_command_ptr     on_user_command;
static on_report_options_ptr   on_report_options;
static on_execute_realtime_ptr on_execute_realtime;
static on_stream_changed_ptr   on_stream_changed;

#if RGB_LED_ENABLE == 2

#define HIMILL_FRONT_RGB_LEDS  24
#define HIMILL_BUTTON_RGB_LEDS 1
#define HIMILL_RGB_JOG_DEFER_MS 250

static on_state_change_ptr on_state_change;

typedef struct {
    GPIO_TypeDef *port;
    uint32_t bit;
} himill_ws2812_line_t;

static rgb_color_t himill_front_leds[HIMILL_FRONT_RGB_LEDS];
static rgb_color_t himill_button_led;
static bool himill_rgb_runtime_enabled = false;
static volatile uint32_t himill_rgb_deferred_ticks = 0;
static bool himill_rgb_update_deferred = false;

static const himill_ws2812_line_t himill_front_rgb = {
    .port = AUXOUTPUT5_PORT,
    .bit = 1UL << AUXOUTPUT5_PIN
};

static const himill_ws2812_line_t himill_button_rgb = {
    .port = AUXOUTPUT6_PORT,
    .bit = 1UL << AUXOUTPUT6_PIN
};

#define HIMILL_WS2812_DELAY_0H() __asm__ volatile ( \
    "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t" \
    "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t" \
    "nop\n\t""nop\n\t" ::: "memory")
#define HIMILL_WS2812_DELAY_1H() __asm__ volatile ( \
    "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t" \
    "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t" \
    "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t" \
    "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t" \
    "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t" \
    "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t" \
    "nop\n\t""nop\n\t" ::: "memory")
#define HIMILL_WS2812_DELAY_0L() __asm__ volatile ( \
    "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t" \
    "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t" \
    "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t" \
    "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t" \
    "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t" \
    "nop\n\t""nop\n\t" ::: "memory")
#define HIMILL_WS2812_DELAY_1L() __asm__ volatile ( \
    "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t" \
    "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t" ::: "memory")

static inline void himill_ws2812_high (const himill_ws2812_line_t *line)
{
    line->port->BSRR = line->bit << 16; // Inverted level shifter.
}

static inline void himill_ws2812_low (const himill_ws2812_line_t *line)
{
    line->port->BSRR = line->bit;
}

static inline void himill_ws2812_send_byte (const himill_ws2812_line_t *line, uint8_t value)
{
    for (uint_fast8_t mask = 0x80; mask; mask >>= 1) {
        himill_ws2812_high(line);
        if (value & mask) {
            HIMILL_WS2812_DELAY_1H();
            himill_ws2812_low(line);
            HIMILL_WS2812_DELAY_1L();
        } else {
            HIMILL_WS2812_DELAY_0H();
            himill_ws2812_low(line);
            HIMILL_WS2812_DELAY_0L();
        }
    }
}

static inline void himill_ws2812_send_color (const himill_ws2812_line_t *line, rgb_color_t color)
{
    himill_ws2812_send_byte(line, color.G);
    himill_ws2812_send_byte(line, color.R);
    himill_ws2812_send_byte(line, color.B);
}

static void himill_ws2812_reset_delay (void)
{
    for (volatile uint32_t i = 0; i < 6000; i++)
        __NOP();
}

static void himill_ws2812_send (const himill_ws2812_line_t *line, const rgb_color_t *leds, uint_fast16_t n_leds)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    for (uint_fast16_t i = 0; i < n_leds; i++)
        himill_ws2812_send_color(line, leds[i]);

    himill_ws2812_low(line);

    __set_PRIMASK(primask);
    himill_ws2812_reset_delay();
}

static inline rgb_color_t himill_rgb_color (uint8_t red, uint8_t green, uint8_t blue)
{
    return (rgb_color_t){ .R = red, .G = green, .B = blue };
}

static void himill_rgb0_write_raw (void)
{
    himill_ws2812_send(&himill_front_rgb, himill_front_leds, HIMILL_FRONT_RGB_LEDS);
}

static void himill_rgb1_write_raw (void)
{
    himill_ws2812_send(&himill_button_rgb, &himill_button_led, HIMILL_BUTTON_RGB_LEDS);
}

static void himill_rgb0_write (void)
{
    if (himill_rgb_runtime_enabled)
        himill_rgb0_write_raw();
}

static void himill_rgb1_write (void)
{
    if (himill_rgb_runtime_enabled)
        himill_rgb1_write_raw();
}

static void himill_rgb0_out (uint16_t device, rgb_color_t color)
{
    if (device < HIMILL_FRONT_RGB_LEDS)
        himill_front_leds[device] = color;
}

static void himill_rgb1_out (uint16_t device, rgb_color_t color)
{
    if (device < HIMILL_BUTTON_RGB_LEDS) {
        himill_button_led = color;
        himill_rgb1_write();
    }
}

static void himill_rgb0_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    if (device < HIMILL_FRONT_RGB_LEDS) {
        if (mask.R)
            himill_front_leds[device].R = color.R;
        if (mask.G)
            himill_front_leds[device].G = color.G;
        if (mask.B)
            himill_front_leds[device].B = color.B;
    }
}

static void himill_rgb1_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    if (device < HIMILL_BUTTON_RGB_LEDS) {
        if (mask.R)
            himill_button_led.R = color.R;
        if (mask.G)
            himill_button_led.G = color.G;
        if (mask.B)
            himill_button_led.B = color.B;
        himill_rgb1_write();
    }
}

static void himill_front_rgb_fill (rgb_color_t color)
{
    for (uint_fast8_t i = 0; i < HIMILL_FRONT_RGB_LEDS; i++)
        himill_front_leds[i] = color;
}

static rgb_color_t himill_rgb_color_for_state (sys_state_t state)
{
    if (state & STATE_ESTOP)
        return himill_rgb_color(96, 0, 0);
    if (state & STATE_ALARM)
        return himill_rgb_color(64, 0, 0);
    if (state & STATE_TOOL_CHANGE)
        return himill_rgb_color(64, 28, 0);
    if (state & STATE_SAFETY_DOOR)
        return himill_rgb_color(60, 18, 0);
    if (state & STATE_HOLD)
        return himill_rgb_color(56, 40, 0);
    if (state & STATE_HOMING)
        return himill_rgb_color(32, 32, 32);
    if (state & STATE_JOG)
        return himill_rgb_color(28, 0, 48);
    if (state & STATE_CYCLE)
        return himill_rgb_color(0, 0, 64);
    if (state & STATE_CHECK_MODE)
        return himill_rgb_color(0, 40, 40);
    if (state & STATE_SLEEP)
        return himill_rgb_color(0, 0, 0);

    return himill_rgb_color(0, 32, 0);
}

static void himill_rgb_update_state (sys_state_t state)
{
    himill_front_rgb_fill(himill_rgb_color_for_state(state));
    himill_rgb0_write();

    himill_button_led = state & STATE_TOOL_CHANGE
                         ? himill_rgb_color(64, 28, 0)
                         : himill_rgb_color(0, 0, 0);
    himill_rgb1_write();
}

static void himill_rgb_update_state_or_defer (sys_state_t state)
{
    // A 24-pixel WS2812 refresh blocks interrupts long enough to overrun
    // UARTs at 115200 baud. Jogging flips Idle/Jog rapidly, exactly while
    // senders are likely streaming $J= lines, so postpone the LED update
    // until jogging has been quiet for a short window.
    if (!himill_rgb_runtime_enabled)
        return;

    if ((state & STATE_JOG) || (state == STATE_IDLE && himill_rgb_update_deferred)) {
        himill_rgb_update_deferred = true;
        himill_rgb_deferred_ticks = hal.get_elapsed_ticks ? hal.get_elapsed_ticks() : 0;
        return;
    }

    himill_rgb_update_deferred = false;
    himill_rgb_update_state(state);
}

static void himill_rgb_flush_deferred_state (sys_state_t state)
{
    uint32_t now = hal.get_elapsed_ticks ? hal.get_elapsed_ticks() : 0;

    if (himill_rgb_runtime_enabled && himill_rgb_update_deferred &&
        !(state & STATE_JOG) && (now - himill_rgb_deferred_ticks) >= HIMILL_RGB_JOG_DEFER_MS) {
        himill_rgb_update_deferred = false;
        himill_rgb_update_state(state);
    }
}

static void himill_rgb_set_enabled (bool enabled)
{
    bool was_enabled = himill_rgb_runtime_enabled;

    himill_rgb_runtime_enabled = enabled;
    himill_rgb_update_deferred = false;

    if (enabled) {
        himill_rgb_update_state_or_defer(state_get());
    } else if (was_enabled) {
        himill_front_rgb_fill(himill_rgb_color(0, 0, 0));
        himill_button_led = himill_rgb_color(0, 0, 0);
        himill_rgb0_write_raw();
        himill_rgb1_write_raw();
    }
}

static void himill_rgb_on_state_change (sys_state_t state)
{
    if (on_state_change)
        on_state_change(state);

    himill_rgb_update_state_or_defer(state);
}

static void himill_rgb_init (void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    himill_ws2812_low(&himill_front_rgb);
    himill_ws2812_low(&himill_button_rgb);

    GPIO_InitTypeDef gpio = {
        .Pin = GPIO_PIN_5 | GPIO_PIN_15,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };
    HAL_GPIO_Init(GPIOB, &gpio);

    himill_ws2812_low(&himill_front_rgb);
    himill_ws2812_low(&himill_button_rgb);

    hal.rgb0.out = himill_rgb0_out;
    hal.rgb0.out_masked = himill_rgb0_out_masked;
    hal.rgb0.write = himill_rgb0_write;
    hal.rgb0.cap = himill_rgb_color(255, 255, 255);
    hal.rgb0.flags.is_blocking = On;
    hal.rgb0.flags.is_strip = On;
    hal.rgb0.num_devices = HIMILL_FRONT_RGB_LEDS;

    hal.rgb1.out = himill_rgb1_out;
    hal.rgb1.out_masked = himill_rgb1_out_masked;
    hal.rgb1.write = himill_rgb1_write;
    hal.rgb1.cap = himill_rgb_color(255, 255, 255);
    hal.rgb1.flags.is_blocking = On;
    hal.rgb1.flags.is_strip = On;
    hal.rgb1.num_devices = HIMILL_BUTTON_RGB_LEDS;

    on_state_change = grbl.on_state_change;
    grbl.on_state_change = himill_rgb_on_state_change;

    if (himill_rgb_runtime_enabled)
        himill_rgb_update_state(STATE_IDLE);
}

#endif // RGB_LED_ENABLE == 2

// Pendant traffic rate-limits. Without these the SmartPendant overwhelms
// F103RC's main loop and produces self-sustained MPG toggle oscillation
// (~200 enable/disable cycles per second observed; see /tmp/mpg3.txt
// analysis 2026-04-30).
//
// HIMILL_MPG_POLL_MIN_MS — minimum gap between forwarded `?` status polls.
// 100 ms = 10 Hz pendant display refresh, plenty smooth.
//
// HIMILL_MPG_TOGGLE_DEBOUNCE_MS — minimum gap between forwarded 0x8B
// (CMD_MPG_MODE_TOGGLE) bytes. The SmartPendant sends 0x8B at high rate
// (heartbeat + button-held auto-repeat). With grblHAL's split-handler
// design (stream layer enables on 0x8B, protocol layer disables on 0x8B
// when in MPG mode), each 0x8B byte toggles state — so a pendant
// streaming 0x8B at 100 Hz creates a 100 Hz MPG toggle loop. Debouncing
// to 500 ms collapses heartbeat traffic to a single event while still
// letting an intentional button press (~1 Hz) through.
#define HIMILL_MPG_POLL_MIN_MS         100
#define HIMILL_MPG_TOGGLE_DEBOUNCE_MS  500
static volatile uint32_t mpg_last_poll_ticks = 0;
static volatile uint32_t mpg_last_toggle_ticks = 0;

static const io_stream_t *mpg_uart_stream = NULL;

// Intercepts 0x8B before it can reach protocol_enqueue_realtime_command.
// Installed as mux_rt_handler in board_init. mux_rt_handler is the function
// used by:
//   1) mux_enqueue_rt_from for USB / UART2 byte streams (every realtime
//      byte from those streams routes through it)
//   2) UART4 (the MPG stream) AFTER stream_mpg_enable swaps UART4's
//      rx_handler to mux_rt_handler at grbl/stream.c:705
// So this wrapper sees every 0x8B from every stream regardless of MPG
// state. Debouncing here closes the race window where pendant heartbeat
// bytes slip through during the brief moment between stream_mpg_enable's
// handler swap and our re-install via on_execute_realtime — which was the
// remaining ~5 ms toggle source observed in /tmp/mpg4.txt (32 toggles in a
// 2-second window even WITH himill_mpg_rx_filter's debounce active).
static bool himill_intercept_rt (uint8_t c)
{
    if (c == CMD_MPG_MODE_TOGGLE) {
        uint32_t now = hal.get_elapsed_ticks ? hal.get_elapsed_ticks() : 0;
        if ((now - mpg_last_toggle_ticks) < HIMILL_MPG_TOGGLE_DEBOUNCE_MS)
            return true; // drop heartbeat / button auto-repeat
        mpg_last_toggle_ticks = now;
    }
    return protocol_enqueue_realtime_command(c);
}

// UART4 rx_handler that stays installed in BOTH MPG-OFF and MPG-ON state
// (re-installed by himill_poll_buttons after every transition).
//
// MPG OFF:
//   - 0x8B → forward to stream_mpg_check_enable (engage MPG), debounced
//   - status-report bytes → forward, rate-limited to HIMILL_MPG_POLL_MIN_MS
//   - other → forward to stream_mpg_check_enable (default branch goes
//     to protocol_enqueue_realtime_command — safe since pendant is
//     electrically driving the line, no floating-pin noise)
//
// MPG ON:
//   - 0x8B → debounced; only forward to protocol (release MPG) once per
//     HIMILL_MPG_TOGGLE_DEBOUNCE_MS window
//   - everything else → forward to protocol_enqueue_realtime_command,
//     letting jog commands ($J=...) drop into the UART4 RX buffer where
//     the parser reads them via hal.stream.read = mpg.stream.read
static bool himill_mpg_rx_filter (uint8_t c)
{
    uint32_t now = hal.get_elapsed_ticks ? hal.get_elapsed_ticks() : 0;

    if (c == CMD_MPG_MODE_TOGGLE) {
        if ((now - mpg_last_toggle_ticks) < HIMILL_MPG_TOGGLE_DEBOUNCE_MS)
            return true; // drop heartbeat / button auto-repeat
        mpg_last_toggle_ticks = now;
    }

    if (sys.mpg_mode) {
        // MPG ACTIVE: route everything (including pendant `?`, jog $J=,
        // and the now-debounced 0x8B) through the standard protocol path.
        return protocol_enqueue_realtime_command(c);
    }

    switch (c) {
        case CMD_STATUS_REPORT:
        case CMD_STATUS_REPORT_LEGACY:
        case CMD_STATUS_REPORT_ALL:
        case CMD_GCODE_REPORT:
            if ((now - mpg_last_poll_ticks) < HIMILL_MPG_POLL_MIN_MS)
                return true; // drop excess polls
            mpg_last_poll_ticks = now;
            break;
        default:
            break;
    }
    return stream_mpg_check_enable(c);
}

// USB-CDC ↔ UART2 stream switcher — wrapper version (DTR-driven).
//
// MaxMake (2026-04-25) confirmed grblHAL is single-stream and they
// switch primary based on USB DTR. We wrap hal.stream's read/write
// pointers with our own functions that delegate to whichever real
// stream is currently active. UART2 (ESP3D bridge) is the default;
// USB CDC takes over when its DTR asserts (host opens the port).
//
// We tried direct memcpy of hal.stream from one stream to another (the
// "tell grblHAL which stream is active" approach the user suggested)
// but it broke USB enumeration during init — likely something in the
// USB CDC stack expects hal.stream to be the USB stream object during
// later init steps. Wrapping is safer: hal.stream stays the way grblHAL
// set it up, and our wrapper functions silently route to whichever
// underlying stream is currently active.
static const io_stream_t *mux_usb         = NULL;
static const io_stream_t *mux_uart        = NULL;
static bool               mux_initialized = false;
static uint8_t            telnet_iac_state = 0;
static const io_stream_t *mux_rx_line_owner = NULL;
static const io_stream_t *mux_last_rx_owner = NULL;
static const io_stream_t *mux_reply_owner = NULL;
static const io_stream_t *mux_suspended_stream = NULL;
static enqueue_realtime_command_ptr mux_rt_handler = protocol_enqueue_realtime_command;
static const io_stream_t *mux_rt_owner = NULL;
static bool mux_rt_owner_locked = false;
static int32_t esp_tunnel_pending = SERIAL_NO_DATA;
// Tunnel window for [ESP*] command responses: while non-zero, UART2 RX is
// owned exclusively by the himill_poll_buttons drainer that mirrors bytes
// to USB CDC TX. mux_read suppresses its USB→UART fallback while this is
// set so it doesn't race on the same RX buffer.
static volatile uint32_t  esp_tunnel_until_ms = 0;

// Pick active based on DTR. USB if host has port open, otherwise UART.
static const io_stream_t *mux_active (void)
{
    if (mux_usb && mux_usb->is_connected && mux_usb->is_connected())
        return mux_usb;
    return mux_uart;
}

static bool mux_stream_connected (const io_stream_t *s)
{
    if (!s)
        return false;

    // UART2 is hard-wired to ESP3D. USB is considered connected only when
    // the host has asserted DTR and the USB driver has accepted it.
    return s == mux_uart || !s->is_connected || s->is_connected();
}

static const io_stream_t *mux_owned_or_active (void)
{
    if (mux_rx_line_owner && mux_stream_connected(mux_rx_line_owner))
        return mux_rx_line_owner;

    if (mux_reply_owner && mux_stream_connected(mux_reply_owner))
        return mux_reply_owner;

    return mux_active();
}

static const io_stream_t *mux_command_or_active (void)
{
    if (mux_rx_line_owner && mux_stream_connected(mux_rx_line_owner))
        return mux_rx_line_owner;

    if (mux_last_rx_owner && mux_stream_connected(mux_last_rx_owner))
        return mux_last_rx_owner;

    if (mux_reply_owner && mux_stream_connected(mux_reply_owner))
        return mux_reply_owner;

    return mux_active();
}

static const io_stream_t *mux_write_target (void)
{
    return mux_stream_connected(mux_reply_owner) ? mux_reply_owner : mux_active();
}

static void mux_note_rx_byte (const io_stream_t *s, int32_t c)
{
    if (!mux_rx_line_owner)
        mux_rx_line_owner = s;

    mux_last_rx_owner = mux_rx_line_owner;
    mux_reply_owner = mux_rx_line_owner;

    if (c == ASCII_CR || c == ASCII_LF || c == ASCII_EOF)
        mux_rx_line_owner = NULL;
}

static bool mux_enqueue_rt_from (const io_stream_t *source, uint8_t c)
{
    bool drop;
    enqueue_realtime_command_ptr handler = mux_rt_handler ? mux_rt_handler : protocol_enqueue_realtime_command;

    mux_reply_owner = source;

    if (mux_rt_owner_locked && mux_rt_owner && source != mux_rt_owner)
        drop = protocol_enqueue_realtime_command(c);
    else
        drop = handler(c);

    return drop;
}

static bool mux_usb_enqueue_rt (uint8_t c)
{
    // 0x8B (CMD_MPG_MODE_TOGGLE) is exclusive to the pendant on UART4.
    // ESP3D forwards traffic from the host (USB or telnet/WS) — if any
    // of that contains 0x8B (e.g. binary data, telnet negotiation bytes,
    // ESP3D framing) it would hit protocol.c:931 and disable MPG while
    // the pendant is still engaged → toggle loop. Pendant 0x8B comes in
    // on UART4 and is unaffected by this filter.
    if (c == CMD_MPG_MODE_TOGGLE)
        return true;
    return mux_enqueue_rt_from(mux_usb, c);
}

static bool mux_uart_enqueue_rt (uint8_t c)
{
    // See mux_usb_enqueue_rt for rationale. Critical here too: this is the
    // ESP3D-bridged path, and ESP3D was confirmed to be emitting 0x8B
    // bytes that triggered the MPG-toggle loop (mpg_good.txt isolation
    // log vs mpg2.txt mux-mode log: same firmware, same pendant; only
    // difference was whether USART2's ISR was active).
    if (c == CMD_MPG_MODE_TOGGLE)
        return true;
    return mux_enqueue_rt_from(mux_uart, c);
}

static bool mux_is_connected (void)
{
    // UART2 is hard-wired to the ESP32 bridge, so for the board's primary
    // runtime path it should count as connected even when USB DTR is low.
    return mux_stream_connected(mux_uart) || mux_stream_connected(mux_usb);
}

static int32_t mux_read (void)
{
    for (;;) {
        const io_stream_t *s = mux_rx_line_owner ? mux_rx_line_owner : mux_active();
        int32_t c = (s && s->read) ? s->read() : SERIAL_NO_DATA;

        // If USB is currently "active" but has no byte ready, fall back to UART
        // so a stale/phantom DTR assertion cannot black-hole the first telnet
        // command. SUPPRESSED while an [ESP*] response tunnel is open — during
        // the tunnel, UART2 RX is owned exclusively by the tunnel drainer in
        // himill_poll_buttons. Without this guard, two consumers race on the
        // UART RX buffer and ESP3D's JSON response gets split between the
        // tunnel (mirrored to USB) and grblHAL's parser (which then errors
        // out trying to interpret JSON as gcode).
        if (c == SERIAL_NO_DATA && mux_rx_line_owner == NULL && s == mux_usb
                && esp_tunnel_until_ms == 0
                && mux_uart && mux_uart->read) {
            s = mux_uart;
            c = mux_uart->read();
        }

        if (c == SERIAL_NO_DATA)
            return c;

        // ESP3D telnet is effectively a raw bridge. Some telnet clients still send
        // IAC negotiation bytes on connect; if we pass those to grblHAL they get
        // buffered until the first user newline and the parser returns error:1.
        if (s == mux_uart) switch (telnet_iac_state) {

            case 0: // normal data
                if ((uint8_t)c == 0xFF) {
                    telnet_iac_state = 1;
                    continue;
                }
                mux_note_rx_byte(s, c);
                return c;

            case 1: // seen IAC
                if ((uint8_t)c == 0xFF) {
                    telnet_iac_state = 0;
                    continue;
                }
                if ((uint8_t)c == 0xFA) { // subnegotiation start
                    telnet_iac_state = 3;
                    continue;
                }
                if ((uint8_t)c >= 0xFB && (uint8_t)c <= 0xFE) { // WILL/WONT/DO/DONT
                    telnet_iac_state = 2;
                    continue;
                }
                telnet_iac_state = 0;
                continue;

            case 2: // option byte for WILL/WONT/DO/DONT
                telnet_iac_state = 0;
                continue;

            case 3: // inside subnegotiation
                if ((uint8_t)c == 0xFF)
                    telnet_iac_state = 4;
                continue;

            case 4: // IAC within subnegotiation
                telnet_iac_state = (uint8_t)c == 0xF0 ? 0 : 3; // SE ends subnegotiation
                continue;
        }

        mux_note_rx_byte(s, c);
        return c;
    }
}

static void mux_write (const char *str)
{
    const io_stream_t *s = mux_write_target();
    if (s && s->write)
        s->write(str);
}

static bool mux_write_char (const uint8_t c)
{
    const io_stream_t *s = mux_write_target();
    return (s && s->write_char) ? s->write_char(c) : false;
}

static void mux_write_n (const uint8_t *data, uint16_t length)
{
    const io_stream_t *s = mux_write_target();

    if (!s)
        return;

    if (s->write_n) {
        s->write_n(data, length);
        return;
    }

    while (length--) {
        if (!(s->write_char && s->write_char(*data++)))
            break;
    }
}

static void mux_write_all (const char *str)
{
    if (mux_stream_connected(mux_uart) && mux_uart->write)
        mux_uart->write(str);

    if (mux_stream_connected(mux_usb) && mux_usb != mux_uart && mux_usb->write)
        mux_usb->write(str);

    // When MPG is engaged, the pendant requests status with `?`. That hits
    // protocol_enqueue_realtime_command, which schedules a "regular" report
    // routed through hal.stream.write_all (= this function). In standard
    // grblHAL stream_write_all iterates the connection list — which includes
    // the MPG stream — so the pendant gets the report on UART4. mux_write_all
    // doesn't iterate that list, so without this branch the pendant never
    // sees `MPG:1`, decides MPG didn't engage, and re-sends 0x8B. After our
    // 500 ms debounce window the next 0x8B passes through and toggles MPG
    // off → ~0.5 Hz toggle loop (the alternating MPG:1/MPG:0 in /tmp/mpg6.txt).
    if (sys.mpg_mode && mpg_uart_stream && mpg_uart_stream->write)
        mpg_uart_stream->write(str);
}

static uint16_t mux_get_rx_buffer_count (void)
{
    const io_stream_t *s = mux_owned_or_active();
    return (s && s->get_rx_buffer_count) ? s->get_rx_buffer_count() : 0;
}

static uint16_t mux_get_rx_buffer_free (void)
{
    const io_stream_t *s = mux_owned_or_active();
    return (s && s->get_rx_buffer_free) ? s->get_rx_buffer_free() : 0xFFFF;
}

static uint16_t mux_get_tx_buffer_count (void)
{
    const io_stream_t *s = mux_write_target();
    return (s && s->get_tx_buffer_count) ? s->get_tx_buffer_count() : 0;
}

static void mux_reset_read_buffer (void)
{
    if (mux_usb && mux_usb->reset_read_buffer)
        mux_usb->reset_read_buffer();
    if (mux_uart && mux_uart->reset_read_buffer)
        mux_uart->reset_read_buffer();

    mux_rx_line_owner = NULL;
}

// CMD_JOG_CANCEL (0x85) and other paths call this. The serial driver
// implementation injects an ASCII_CAN (0x18) into the RX buffer — that
// 0x18 is what triggers EXEC_MOTION_CANCEL when the main loop next
// reads (protocol.c:212-220). MUST land in whichever stream main loop
// is reading via mux_read (i.e. the active one). Earlier we missed
// this wrapper, the 0x18 went to USB CDC's buffer, and on telnet/WS
// jog cancel was a no-op — Z/A continuous jogs ran to completion
// regardless of release.
static void mux_cancel_read_buffer (void)
{
    const io_stream_t *s = mux_owned_or_active();
    if (s && s->cancel_read_buffer)
        s->cancel_read_buffer();

    mux_rx_line_owner = NULL;
}

static void mux_reset_write_buffer (void)
{
    if (mux_usb && mux_usb->reset_write_buffer)
        mux_usb->reset_write_buffer();
    if (mux_uart && mux_uart->reset_write_buffer)
        mux_uart->reset_write_buffer();
}

static bool mux_disable_rx (bool disable)
{
    bool ok = false;

    if (mux_usb && mux_usb->disable_rx)
        ok = mux_usb->disable_rx(disable) || ok;
    if (mux_uart && mux_uart->disable_rx)
        ok = mux_uart->disable_rx(disable) || ok;

    return ok;
}

static bool mux_suspend_read (bool suspend)
{
    const io_stream_t *s = suspend ? mux_command_or_active()
                                   : (mux_suspended_stream ? mux_suspended_stream : mux_owned_or_active());
    bool ok = (s && s->suspend_read) ? s->suspend_read(suspend) : false;

    if (ok)
        mux_suspended_stream = suspend ? s : NULL;

    return ok;
}

static enqueue_realtime_command_ptr mux_set_enqueue_rt_handler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = mux_rt_handler;

    if (handler) {
        mux_rt_handler = handler;
        mux_rt_owner = mux_command_or_active();
        mux_rt_owner_locked = handler != protocol_enqueue_realtime_command;
    }

    return prev;
}

static bool mux_find_uart (io_stream_properties_t const *props, void *data)
{
    if (props->type == StreamType_Serial && props->instance == 0
            && props->flags.claimable && !props->flags.claimed) {
        mux_uart = props->claim(115200);  // matches ESP3D side
        return true;
    }
    return false;
}

// Polled cycle-start trigger on PC4 (the parallel-wired front-panel +
// inside-door BUTTONS). Both wires short to GND when pressed; the pin
// idles HIGH via the GPIO pull-up. PC4 can't be IRQ-bound — it shares
// EXTI line 4 with PB4 (e-stop), and on this MCU each EXTI line listens
// to one port only. The driver gives up PC4's IRQ in favor of e-stop's
// (correctly), which leaves us with polled-only access. The macros
// plugin requires IRQ capability so it can't bind here either; we read
// the pin ourselves on every grbl.on_execute_realtime tick and enqueue
// a CMD_CYCLE_START realtime command on the falling edge. Net effect
// is identical to the macros plugin's "Cycle start" button action,
// just dispatched through our own poll instead.
//
// The same approach is wired up for PB1 (door switch) → safety-door
// realtime command. Door is dual-purpose on this board (interlock + a
// secondary toolchange-confirm path) and can't bind through the regular
// SAFETY_DOOR_ENABLE path either, for the same EXTI-collision reason
// (PB1 vs PC1 / Y limit).
static bool button_pc4_prev_pressed = false;
static bool button_pb1_prev_pressed = false;

static bool himill_prepare_toolchange_button (sys_state_t state)
{
    if (state != STATE_TOOL_CHANGE || !gc_state.tool_change)
        return true;

#ifndef HIMILL_SKIP_MUX
    // M6 first suspends stream input and waits for CMD_TOOL_ACK. Until that
    // ack is processed, grblHAL has not installed the cycle-start trap that
    // runs the toolsetter probe. If the sender disconnects before the ack is
    // delivered, or the physical button lands before the ack path completes,
    // a normal cycle-start would clear gc_state.tool_change and resume the
    // job without probing.
    if (hal.control.interrupt_callback == control_interrupt_handler) {
        if (hal.stream.read != mux_read && mux_rt_handler) {
            if (!mux_rt_handler(CMD_TOOL_ACK))
                return false;
        } else if (grbl.on_toolchange_ack)
            grbl.on_toolchange_ack();
    }
#endif

    return hal.control.interrupt_callback != control_interrupt_handler;
}

// [LASER_MODE:0|1] — mirror spindle PWM/enable to laser hardware.
//
// HiMill hardware has two PWM driver chips: PB8/PB9 (CNC spindle motor)
// and PB0/PC5 (laser). Both can't run simultaneously — they share the
// "spindle" command surface. MaxMake's vendor firmware exposed
// `[LASER_MODE:1]` to switch between them.
//
// We mirror approach: TIM3_CH3 PWM on PB0 runs in lock-step with the
// existing spindle TIM4_CH3 PWM on PB8 (same period/prescaler/duty),
// and the realtime tick copies PB9's enable state to PC5. M3/M5 and
// spindle PWM commands then drive both heads identically. The user
// physically connects whichever is in use (CNC motor OR laser head)
// and toggles `[LASER_MODE]` to wire the enable line accordingly.
static volatile bool laser_mode_active = false;

static void laser_pwm_init (void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // PB0 → TIM3_CH3 alternate-function push-pull (no remap needed).
    GPIO_InitTypeDef gpio = {
        .Pin = GPIO_PIN_0,
        .Mode = GPIO_MODE_AF_PP,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };
    HAL_GPIO_Init(GPIOB, &gpio);

    // PC5 = LASER_EN as plain output, idle low.
    gpio.Pin = GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

    // CRITICAL: suppress PB8 so the BLDC spindle ESC sees no PWM signal.
    // The brushless driver reads the PWM directly and starts spinning
    // above ~20% duty even with PB9 enable held low. Reconfigure PB8
    // from TIM4_CH3 alternate-function to plain output, idle HIGH
    // (active-low PWM idle = spindle off). TIM4 keeps running so
    // grblHAL's spindle commands work normally — just nothing reaches
    // the ESC pin.
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);  // idle high = off

    // Mirror TIM4's prescaler/period so PB0's PWM has the same
    // frequency as PB8's. CCR3 is updated continuously in the
    // realtime tick.
    TIM3->CR1   &= ~TIM_CR1_CEN;
    TIM3->PSC   = TIM4->PSC;
    TIM3->ARR   = TIM4->ARR;
    TIM3->CCR3  = 0;

    // Channel 3 → PWM mode 1 (active when CCR > CNT).
    TIM3->CCMR2 &= ~TIM_CCMR2_OC3M;
    TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
    TIM3->CCMR2 |= TIM_CCMR2_OC3PE;  // preload enable

    // Match TIM4's polarity (active-low PWM per HiMill spec — TIM4 is
    // configured this way already by spindleConfig().)
    if (TIM4->CCER & TIM_CCER_CC3P)
        TIM3->CCER |= TIM_CCER_CC3P;
    else
        TIM3->CCER &= ~TIM_CCER_CC3P;

    TIM3->CCER |= TIM_CCER_CC3E;
    TIM3->EGR   = TIM_EGR_UG;
    TIM3->CR1  |= TIM_CR1_CEN;
}

static void laser_pwm_deinit (void)
{
    // Stop TIM3 PWM and idle outputs.
    TIM3->CCR3 = 0;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->CCER &= ~TIM_CCER_CC3E;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

    // PB0 back to plain input — drops the laser PWM line clean.
    GPIO_InitTypeDef gpio = {
        .Pin = GPIO_PIN_0,
        .Mode = GPIO_MODE_INPUT,
        .Pull = GPIO_NOPULL
    };
    HAL_GPIO_Init(GPIOB, &gpio);

    // Restore PB8 to TIM4_CH3 alternate-function so spindle PWM reaches
    // the ESC again (matches what spindleConfig set up at boot).
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static inline void laser_pwm_mirror (void)
{
    // Copy spindle PWM duty cycle (TIM4_CH3) to laser PWM (TIM3_CH3).
    uint32_t ccr = TIM4->CCR3;
    TIM3->CCR3 = ccr;

    // Drive LASER_EN (PC5) based on whether the PWM is active.
    // PB8 is GPIO-suppressed in laser mode (idle high = ESC sees no
    // PWM, never spins), so we don't need to also force PB9 low —
    // the ESC is already isolated from the spindle command path.
    GPIOC->BSRR = ccr ? GPIO_PIN_5 : ((uint32_t)GPIO_PIN_5 << 16);
}

// [ESP*] command forwarding from USB CDC → UART2 (ESP3D).
//
// Use case: a user with no WiFi configured plugs in USB serial and runs
// the standard ESP3D bootstrap sequence:
//     [ESP110]WIFI-STA json
//     [ESP100]<ssid> json
//     [ESP101]<password> json
//     [ESP444]RESTART json
//
// ESP3D listens on UART2, not USB CDC. We bridge: write the line to
// UART2 with a newline, then for a brief window mirror UART2 RX bytes
// back to USB CDC TX so the user sees ESP3D's JSON response. After the
// window expires we stop tunneling so normal UART traffic (telnet etc.)
// goes back to its usual path.
//
// `[ESP420] json` is also useful — returns ESP3D's full system info
// JSON which includes the device's IP / WiFi state.
// (esp_tunnel_until_ms is declared up at file scope so mux_read can see it.)
#define ESP_TUNNEL_WINDOW_MS 20000

static void himill_poll_buttons (sys_state_t state)
{
#if MPG_ENABLE == 2 && !defined(HIMILL_SKIP_MUX)
    // Re-install our filter as UART4's rx_handler. stream_mpg_enable swaps
    // it for protocol_enqueue_realtime_command on every transition (grbl/
    // stream.c:705 and :723 alternately install protocol-handler when MPG
    // engages and our filter when MPG disengages). Without this re-install,
    // 0x8B bytes from the pendant during MPG-ON state hit protocol_enqueue
    // _realtime_command directly → protocol.c:931 disables MPG → next 0x8B
    // re-enables → ~200 Hz toggle loop. Idempotent and cheap.
    if (mpg_uart_stream && mpg_uart_stream->set_enqueue_rt_handler)
        mpg_uart_stream->set_enqueue_rt_handler(himill_mpg_rx_filter);
#endif

#if RGB_LED_ENABLE == 2
    himill_rgb_flush_deferred_state(state);
#endif

    // PC4 = front-panel + inside-door buttons (parallel-wired, active-low).
    // PB1 = door switch.
    //
    // We dispatch button presses through `hal.control.interrupt_callback`
    // (the SAME path a hardware aux-input pin would take), NOT through
    // `grbl.enqueue_realtime_command()`. Why: during M6 wait state grblHAL
    // installs `trap_control_cycle_start` on hal.control.interrupt_callback
    // which advances the toolchange state machine (probe / return-from-G30).
    // Going through grbl.enqueue_realtime_command bypasses that trap on this
    // mux configuration — the byte hits protocol_enqueue_realtime_command
    // raw, which during M6 is interpreted as "cancel the toolchange". The
    // FlexiHAL behavior of "physical CYCLE_START button confirms M6" only
    // works because that path uses the control-signal callback. Match it.
    bool pc4_pressed = !DIGITAL_IN(GPIOC, 4);
    if (pc4_pressed && !button_pc4_prev_pressed && hal.control.interrupt_callback
            && himill_prepare_toolchange_button(state)) {
        control_signals_t signals = {0};
        signals.cycle_start = On;
        hal.control.interrupt_callback(signals);
    }
    button_pc4_prev_pressed = pc4_pressed;

    bool pb1_pressed = !DIGITAL_IN(GPIOB, 1);
    if (pb1_pressed && !button_pb1_prev_pressed && hal.control.interrupt_callback) {
        control_signals_t signals = {0};
        signals.safety_door_ajar = On;
        hal.control.interrupt_callback(signals);
    }
    button_pb1_prev_pressed = pb1_pressed;

    // Laser-mode mirror: copy spindle PWM duty + enable to laser hardware
    // so M3/M5/spindle commands drive PB0/PC5 in lock-step with PB8/PB9.
    if (laser_mode_active)
        laser_pwm_mirror();

    // [ESP*] response tunnel: while open, drain UART2 RX directly to USB CDC
    // TX so the user sees ESP3D's JSON answer on the USB serial they sent
    // the command from. Bypasses mux_active so it works while DTR-mux is
    // pointed at USB.
    if (esp_tunnel_until_ms != 0) {
        if (esp_tunnel_pending == SERIAL_NO_DATA
                && (int32_t)(hal.get_elapsed_ticks() - esp_tunnel_until_ms) >= 0) {
            esp_tunnel_until_ms = 0;
        } else if (mux_uart && mux_uart->read && mux_usb && mux_usb->write_char) {
            for (uint16_t i = 0; i < 64; i++) {  // bounded per-tick drain
                int32_t c = esp_tunnel_pending;
                if (c == SERIAL_NO_DATA)
                    c = mux_uart->read();
                if (c == SERIAL_NO_DATA)
                    break;
                if (mux_usb->write_char((uint8_t)c))
                    esp_tunnel_pending = SERIAL_NO_DATA;
                else {
                    esp_tunnel_pending = c;
                    break;
                }
            }
        }
    }

    if (on_execute_realtime)
        on_execute_realtime(state);
}

// Tear down the running firmware's IRQs + peripherals + clocks and hand
// control to a bootloader at `base_addr`, leaving the MCU in a state
// equivalent to a fresh power-on so the bootloader's SystemInit doesn't
// trip over an already-running PLL or live USB peripheral.
//
// Called in two variants: [BOOT] goes to HiMill's bootloader, [DFU]
// goes to STM32's ROM DFU.
static void jump_to_bootloader (uint32_t base_addr)
{
    // 1) Disable all interrupts and clear pending. Stops any ISR from
    //    firing while we tear down the rest of the running state.
    __disable_irq();
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    // 2) Stop SysTick so HAL's tick callback can't fire mid-jump.
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // 3) Force a USB disconnect by holding the peripheral in reset and
    //    pulling D+ low long enough for the host to register it (>3ms).
    //    `HAL_PCD_MspDeInit` would also work but we don't have a PCD
    //    handle here — direct register manipulation is fine.
    __HAL_RCC_USB_FORCE_RESET();
    for (volatile uint32_t i = 0; i < 240000; i++);  // ~10 ms at 72 MHz
    __HAL_RCC_USB_RELEASE_RESET();

    // 4) Reset HAL + clock tree to power-on defaults (HSI @ 8 MHz, no
    //    PLL). The HiMill bootloader's SystemInit() expects this state
    //    and reconfigures clocks for itself. Skipping this step leaves
    //    the bootloader running with PLL artifacts from grblHAL and its
    //    USB stack fails to enumerate.
    HAL_RCC_DeInit();
    HAL_DeInit();

    // 5) Remap vectors to the bootloader's table.
    SCB->VTOR = base_addr;

    // 6) Load the bootloader's initial stack and reset handler from its
    //    vector table.
    uint32_t sp = *((volatile uint32_t *)base_addr);
    uint32_t pc = *((volatile uint32_t *)(base_addr + 4));

    __set_MSP(sp);
    __enable_irq();

    // Jump — this doesn't return.
    ((void (*)(void))pc)();
}

static status_code_t himill_user_command (char *line)
{
    if (strncmp(line, "[ESP", 4) == 0) {
        // Direction-aware [ESP*] handling:
        //   - From USB CDC: user wants to talk to ESP3D (WiFi config etc.).
        //     Forward the line to UART2 and open a short response-tunnel
        //     window so the user sees ESP3D's JSON answer on USB CDC.
        //   - From UART2: ESP3D's own probe noise leaking through. Silently
        //     consume (ESP3D filters [ESP*] from network clients before
        //     forwarding to STM32, so anything reaching us came from
        //     ESP3D itself).
        if (mux_last_rx_owner == mux_usb && mux_uart && mux_uart->write && mux_uart->write_char) {
            mux_uart->write(line);
            mux_uart->write_char('\n');
            esp_tunnel_pending = SERIAL_NO_DATA;
            esp_tunnel_until_ms = hal.get_elapsed_ticks() + ESP_TUNNEL_WINDOW_MS;
        }
        return Status_OK;
    }

    // [LASER_MODE:0|1] — enable / disable laser PWM mirror on PB0/PC5.
    // Mirrors MaxMake's vendor command. While active, the realtime tick
    // copies the spindle's PWM duty cycle and enable state to the laser
    // hardware, so M3/M5/spindle commands drive both heads identically.
    // The user is responsible for physically connecting only one head
    // at a time (CNC motor OR laser).
    if (strcmp(line, "[LASER_MODE:1]") == 0) {
        if (!laser_mode_active) {
            laser_pwm_init();
            laser_mode_active = true;
        }
        hal.stream.write_all("[MSG:Laser mode ON]" ASCII_EOL);
        return Status_OK;
    }
    if (strcmp(line, "[LASER_MODE:0]") == 0) {
        if (laser_mode_active) {
            laser_mode_active = false;
            laser_pwm_deinit();
        }
        hal.stream.write_all("[MSG:Laser mode OFF]" ASCII_EOL);
        return Status_OK;
    }

    // Exact match — don't swallow [BOOTSOMETHING]
    if (strcmp(line, "[BOOT]") == 0) {
        // Match the vendor app's bootloader handoff: write magic to the
        // reserved top-of-SRAM word, then request a system reset. Interrupts
        // are disabled before the write so no ISR stack traffic can collide
        // with the handoff word before reset.
        __disable_irq();
        *HIMILL_BOOTFLAG_ADDR = HIMILL_BOOTFLAG_MAGIC;
        __DSB();
        for (volatile uint32_t i = 0; i < 720000; i++);  // ~10 ms at 72 MHz

        __set_FAULTMASK(1);
        SCB->AIRCR = (SCB->AIRCR & 0x00000700UL) | 0x05FA0004UL;
        __DSB();
        for (;;)
            ;
        return Status_OK; // not reached
    }

    if (strcmp(line, "[DFU]") == 0) {
        hal.stream.write_all("[MSG:Entering ROM DFU bootloader]" ASCII_EOL);
        if (hal.stream.get_tx_buffer_count) {
            while (hal.stream.get_tx_buffer_count())
                ;
        }
        jump_to_bootloader(STM32_ROM_BOOTLOADER);
        return Status_OK; // not reached
    }

    return on_user_command ? on_user_command(line) : Status_Unhandled;
}

// Pause-on-SD-file-run: when an SD-card stream starts, drop in an M1
// (optional stop) so the operator must explicitly issue cycle-start to
// begin motion. Mirrors the grblHAL/Templates/my_plugin/Pause_on_SD_file_run
// behaviour. Inlined here to keep board scaffolding self-contained.
static void himill_stream_changed (stream_type_t type)
{
    if (on_stream_changed)
        on_stream_changed(type);

    if (type == StreamType_SDCard) {
        char m1[3] = "M1";
        gc_execute_block(m1);
    }
}

// ---- Toolsetter cleaning before M6 ----
//
// $450=1 → before each M6 (after the first one of a session) hover the
// spindle 5 mm above the toolsetter at G59.3 X/Y, run S13000 for 3 s,
// stop. The spindle airflow blows aluminium chips off the toolsetter so
// the standard $341=3 probe has a clean reference. Skipped on the first
// toolchange (no TLR captured yet ⇒ haven't cut anything ⇒ nothing to
// clean). Default Off — only enable if the tester actually needs it.
//
// Coords are read live from G59.3, which is seeded to D1S factory
// defaults at boot/$RST by seed_himill_coord_defaults() below.
// RPM/dwell/lift are compile-time constants today; promote to settings
// later if testers report different needs.

#define HIMILL_CLEAN_RPM      13000.0f
#define HIMILL_CLEAN_DWELL_S  3.0f
#define HIMILL_CLEAN_Z_LIFT   5.0f

#define HIMILL_PROFILE_STATUS_REPORT_MASK       6143
#define HIMILL_PROFILE_FS_OPTIONS               3
#define HIMILL_PROFILE_TOOLCHANGE_PROBE_DIST    65.0f

#define HIMILL_SETTING_STEPPER_SPREADCYCLE 0x01
#define HIMILL_SETTING_RGB_STATUS_ENABLE   0x02

typedef struct {
    bool enable;
    uint8_t flags;
} himill_settings_t;

static himill_settings_t himill_settings;
static nvs_address_t himill_nvs_address;
static on_tool_selected_ptr on_tool_selected;

static const setting_group_detail_t himill_setting_groups[] = {
    { Group_Root, Group_UserSettings, "HiMill D1S" }
};

// PD2 = TMC STEPPER_MODE pin. HIGH = SpreadCycle, LOW = StealthChop.
// TMCs are in standalone mode on D1S — current/microsteps are wired in
// hardware. STEPPER_MODE is the only chopper knob the MCU can toggle.
// SpreadCycle is the default for cutting (full torque at speed, immediate
// accel response). StealthChop is useful for slow probing, drag-knife,
// or quiet jogging — at the cost of reduced high-speed torque.
static void apply_stepper_mode (bool spreadcycle)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, spreadcycle ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void himill_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(himill_nvs_address, (uint8_t *)&himill_settings, sizeof(himill_settings_t), true);
}

static status_code_t himill_set_clean (setting_id_t id, uint_fast16_t value)
{
    himill_settings.enable = !!value;
    himill_settings_save();
    return Status_OK;
}

static uint32_t himill_get_clean (setting_id_t id)
{
    return himill_settings.enable ? 1 : 0;
}

static status_code_t himill_set_stepper_mode (setting_id_t id, uint_fast16_t value)
{
    if (value)
        himill_settings.flags |= HIMILL_SETTING_STEPPER_SPREADCYCLE;
    else
        himill_settings.flags &= ~HIMILL_SETTING_STEPPER_SPREADCYCLE;
    himill_settings_save();
    apply_stepper_mode(!!(himill_settings.flags & HIMILL_SETTING_STEPPER_SPREADCYCLE));
    return Status_OK;
}

static uint32_t himill_get_stepper_mode (setting_id_t id)
{
    return (himill_settings.flags & HIMILL_SETTING_STEPPER_SPREADCYCLE) ? 1 : 0;
}

#if RGB_LED_ENABLE == 2
static status_code_t himill_set_rgb_status (setting_id_t id, uint_fast16_t value)
{
    if (value)
        himill_settings.flags |= HIMILL_SETTING_RGB_STATUS_ENABLE;
    else
        himill_settings.flags &= ~HIMILL_SETTING_RGB_STATUS_ENABLE;
    himill_settings_save();
    himill_rgb_set_enabled(!!(himill_settings.flags & HIMILL_SETTING_RGB_STATUS_ENABLE));
    return Status_OK;
}

static uint32_t himill_get_rgb_status (setting_id_t id)
{
    return (himill_settings.flags & HIMILL_SETTING_RGB_STATUS_ENABLE) ? 1 : 0;
}
#endif

static const setting_detail_t himill_setting_detail[] = {
    { Setting_UserDefined_0, Group_UserSettings, "Clean toolsetter before M6", NULL, Format_Bool, NULL, NULL, NULL, Setting_NonCoreFn, himill_set_clean, himill_get_clean, NULL },
    { Setting_UserDefined_1, Group_UserSettings, "TMC chopper mode", NULL, Format_RadioButtons, "StealthChop,SpreadCycle", NULL, NULL, Setting_NonCoreFn, himill_set_stepper_mode, himill_get_stepper_mode, NULL },
#if RGB_LED_ENABLE == 2
    { Setting_UserDefined_2, Group_UserSettings, "RGB", NULL, Format_Bool, NULL, NULL, NULL, Setting_NonCoreFn, himill_set_rgb_status, himill_get_rgb_status, NULL }
#endif
};

static void himill_profile_update_soft_limits (void)
{
    sys.soft_limits.mask = 0;

    if (settings.limits.soft_enabled.mask) {
        for (uint_fast8_t idx = N_AXIS; idx;) {
            idx--;
            if (bit_istrue(settings.limits.soft_enabled.mask, bit(idx)) && settings.axis[idx].max_travel < -0.0f)
                bit_true(sys.soft_limits.mask, bit(idx));
        }
    }
}

static bool himill_profile_repair (void)
{
    bool changed = false;

    if (settings.status_report.mask != HIMILL_PROFILE_STATUS_REPORT_MASK) {
        settings.status_report.mask = HIMILL_PROFILE_STATUS_REPORT_MASK;
        changed = true;
    }

    if (settings.fs_options.mask != HIMILL_PROFILE_FS_OPTIONS) {
        settings.fs_options.mask = HIMILL_PROFILE_FS_OPTIONS;
        changed = true;
    }

    if (settings.limits.soft_enabled.mask != AXES_BITMASK) {
        settings.limits.soft_enabled.mask = AXES_BITMASK;
        himill_profile_update_soft_limits();
        changed = true;
    }

    if (!settings.limits.flags.hard_enabled || settings.limits.flags.check_at_init) {
        settings.limits.flags.hard_enabled = On;
        settings.limits.flags.check_at_init = Off;
        sys.hard_limits.mask = AXES_BITMASK;
        hal.limits.enable(true, (axes_signals_t){0});
        changed = true;
    }

    if (!settings.limits.flags.jog_soft_limited) {
        settings.limits.flags.jog_soft_limited = On;
        changed = true;
    }

    if (!settings.homing.flags.force_set_origin) {
        settings.homing.flags.force_set_origin = On;
        changed = true;
    }

    if (!settings.probe.soft_limited || !settings.probe.toolsetter_auto_select ||
        settings.probe.allow_feed_override || settings.probe.probe2_auto_select || settings.probe.enable_protection) {
        settings.probe.allow_feed_override = Off;
        settings.probe.soft_limited = On;
        settings.probe.toolsetter_auto_select = On;
        settings.probe.probe2_auto_select = Off;
        settings.probe.enable_protection = Off;
        if (hal.probe.configure)
            hal.probe.configure(false, false);
        changed = true;
    }

    if (settings.tool_change.mode != ToolChange_SemiAutomatic) {
        settings.tool_change.mode = ToolChange_SemiAutomatic;
        changed = true;
    }

    if (settings.tool_change.probing_distance != HIMILL_PROFILE_TOOLCHANGE_PROBE_DIST) {
        settings.tool_change.probing_distance = HIMILL_PROFILE_TOOLCHANGE_PROBE_DIST;
        changed = true;
    }

    if (settings.flags.no_restore_position_after_M6 ||
        !settings.flags.tool_change_at_g30 || !settings.flags.tool_change_fast_pulloff) {
        settings.flags.no_restore_position_after_M6 = Off;
        settings.flags.tool_change_at_g30 = On;
        settings.flags.tool_change_fast_pulloff = On;
        changed = true;
    }

    if (changed) {
        settings_write_global();
        hal.stream.write_all("[MSG:D1S repaired]" ASCII_EOL);
    }

    return changed;
}

// Seed G30 (tool-change parking position) and G59.3 (toolsetter location)
// to D1S factory defaults if either is uninitialized (all axes == 0).
//
// Why: G30 and G59.3 live in the parameter NVS block (`$#` output), not
// the settings block (`$$` output). They survive `$RST=$` but get wiped
// by `$RST=#`, `$RST=*`, and a fresh flash. The standard "save your
// settings" workflow grblHAL users follow doesn't capture them, so a
// re-flash or factory reset leaves a new D1S without the toolsetter
// position the rest of the M6 / probe pipeline expects to find.
//
// Called from both himill_settings_load (every boot — re-seeds after
// `$RST=#` once user reboots) and himill_settings_restore (fires on
// `$RST=*` / `$RST=&` and on first-boot version mismatch via the
// settings_restore(settings_all) path in settings.c:3476). Idempotent:
// only writes if values are zero, so calling it twice on first boot
// (once from restore, once from load) is fine.
//
// Edge case: a user who legitimately wants G30 or G59.3 at exactly
// (0,0,0) will get re-seeded on next boot. Acceptable trade-off — for
// the D1S geometry, both positions are non-zero by design.
static void seed_himill_coord_defaults (void)
{
    coord_system_data_t data;

    // G59.3 = (0, -7, -15) — toolsetter location in machine coords.
    // Validated 2026-05-01.
    if (settings_read_coord_data(CoordinateSystem_G59_3, &data)) {
        if (data.coord.x == 0.0f && data.coord.y == 0.0f && data.coord.z == 0.0f) {
            memset(&data, 0, sizeof(data));
            data.coord.x = 0.0f;
            data.coord.y = -7.0f;
            data.coord.z = -15.0f;
            settings_write_coord_data(CoordinateSystem_G59_3, &data);
        }
    }

    // G30 = (30, -7, 0) — tool-change parking position in machine coords.
    if (settings_read_coord_data(CoordinateSystem_G30, &data)) {
        if (data.coord.x == 0.0f && data.coord.y == 0.0f && data.coord.z == 0.0f) {
            memset(&data, 0, sizeof(data));
            data.coord.x = 30.0f;
            data.coord.y = -7.0f;
            data.coord.z = 0.0f;
            settings_write_coord_data(CoordinateSystem_G30, &data);
        }
    }
}

static void himill_settings_restore (void)
{
    himill_settings.enable = false;
    himill_settings.flags = HIMILL_SETTING_STEPPER_SPREADCYCLE; // default: SpreadCycle, RGB off
    himill_settings_save();
    apply_stepper_mode(!!(himill_settings.flags & HIMILL_SETTING_STEPPER_SPREADCYCLE));
#if RGB_LED_ENABLE == 2
    himill_rgb_set_enabled(!!(himill_settings.flags & HIMILL_SETTING_RGB_STATUS_ENABLE));
#endif
    himill_profile_repair();
    seed_himill_coord_defaults();
}

static void himill_settings_load (void)
{
    if (hal.nvs.memcpy_from_nvs((uint8_t *)&himill_settings, himill_nvs_address, sizeof(himill_settings_t), true) != NVS_TransferResult_OK)
        himill_settings_restore();

    apply_stepper_mode(!!(himill_settings.flags & HIMILL_SETTING_STEPPER_SPREADCYCLE));
#if RGB_LED_ENABLE == 2
    himill_rgb_set_enabled(!!(himill_settings.flags & HIMILL_SETTING_RGB_STATUS_ENABLE));
#endif
    himill_profile_repair();
    seed_himill_coord_defaults();
}

static setting_details_t himill_setting_details = {
    .groups = himill_setting_groups,
    .n_groups = sizeof(himill_setting_groups) / sizeof(setting_group_detail_t),
    .settings = himill_setting_detail,
    .n_settings = sizeof(himill_setting_detail) / sizeof(setting_detail_t),
    .save = settings_write_global,
    .load = himill_settings_load,
    .restore = himill_settings_restore
};

static void himill_clean_toolsetter (tool_data_t *tool)
{
    coord_system_data_t g59_3;
    plan_line_data_t plan_data;
    float target[N_AXIS];

    if (!himill_settings.enable)
        goto chain;
    // Skip on first M6 of the session: no TLR yet ⇒ nothing has been cut
    // yet ⇒ no chips to blow off.
    if (!(sys.tlo_reference_set.mask & bit(Z_AXIS)))
        goto chain;
    if (!settings_read_coord_data(CoordinateSystem_G59_3, &g59_3))
        goto chain;

    // clean_z = TLR + current TLO + 5 mm  (machine coords).
    // TLR is stored as steps; convert via Z's steps/mm.
    float tlr_mm = (float)sys.tlo_reference[Z_AXIS] / settings.axis[Z_AXIS].steps_per_mm;
    float clean_z = tlr_mm + gc_state.modal.tool_length_offset[Z_AXIS] + HIMILL_CLEAN_Z_LIFT;

    system_convert_array_steps_to_mpos(target, sys.position);
    target[X_AXIS] = g59_3.coord.values[X_AXIS];
    target[Y_AXIS] = g59_3.coord.values[Y_AXIS];
    target[Z_AXIS] = clean_z;

    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    if (!mc_line(target, &plan_data))
        goto chain;
    if (!protocol_buffer_synchronize())
        goto chain;

    spindle_t *sp = gc_spindle_get(-1);
    if (sp && sp->hal) {
        spindle_state_t on  = { .on = 1 };
        spindle_state_t off = { 0 };
        spindle_set_state(sp->hal, on, HIMILL_CLEAN_RPM);
        delay_sec(HIMILL_CLEAN_DWELL_S, DelayMode_Dwell);
        spindle_set_state(sp->hal, off, 0.0f);
    }

chain:
    if (on_tool_selected)
        on_tool_selected(tool);
}

static void on_report_options_himill (bool newopt)
{
    if (on_report_options)
        on_report_options(newopt);

    if (!newopt)
        hal.stream.write("[PLUGIN:HIMILL_D1S v0.3]" ASCII_EOL);
}

void board_init (void)
{
    // PD2 = TMC STEPPER_MODE pin. Initial level is driven HIGH (SpreadCycle)
    // here; himill_settings_load will reapply the user-selected mode once
    // NVS is up. Worst case if NVS read fails we stay on SpreadCycle, which
    // is the cutting default.
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef gpio_init = {
        .Pin = GPIO_PIN_2,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Speed = GPIO_SPEED_FREQ_LOW
    };
    HAL_GPIO_Init(GPIOD, &gpio_init);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

#if RGB_LED_ENABLE == 2
    himill_rgb_init();
#endif

    on_user_command         = grbl.on_user_command;
    grbl.on_user_command    = himill_user_command;

    on_report_options       = grbl.on_report_options;
    grbl.on_report_options  = on_report_options_himill;

    on_execute_realtime     = grbl.on_execute_realtime;
    grbl.on_execute_realtime = himill_poll_buttons;

    on_stream_changed       = grbl.on_stream_changed;
    grbl.on_stream_changed  = himill_stream_changed;

    // Toolsetter cleaning hook — fires before each M6, gated by $450.
    if ((himill_nvs_address = nvs_alloc(sizeof(himill_settings_t)))) {
        on_tool_selected      = grbl.on_tool_selected;
        grbl.on_tool_selected = himill_clean_toolsetter;
        settings_register(&himill_setting_details);
    }

    // Stream multiplexer: claim UART2 alongside USB CDC and re-route
    // hal.stream's read/write to a mux that polls both. Done here in
    // board_init (which runs AFTER stream_connect(usbInit())) so we
    // know USB CDC is up; if claim fails, leave hal.stream untouched
    // and we fall back to USB-only behaviour.
#ifndef HIMILL_SKIP_MUX
    mux_usb = stream_get_base();
    if (mux_usb) {
        stream_enumerate_streams(mux_find_uart, NULL);
        if (mux_uart) {
            if (mux_usb->set_enqueue_rt_handler)
                mux_usb->set_enqueue_rt_handler(mux_usb_enqueue_rt);
            if (mux_uart->set_enqueue_rt_handler)
                mux_uart->set_enqueue_rt_handler(mux_uart_enqueue_rt);

            // Install our wrapper functions on hal.stream. Input is owned
            // per line so USB and ESP3D bytes cannot be interleaved into one
            // parser command; ordinary replies follow that owner, while
            // write_all broadcasts to connected outputs.
            hal.stream.is_connected        = mux_is_connected;
            hal.stream.read                = mux_read;
            hal.stream.write               = mux_write;
            hal.stream.write_char          = mux_write_char;
            hal.stream.write_n             = mux_write_n;
            hal.stream.write_all           = mux_write_all;
            hal.stream.set_enqueue_rt_handler = mux_set_enqueue_rt_handler;
            hal.stream.suspend_read        = mux_suspend_read;
            hal.stream.disable_rx          = mux_disable_rx;
            hal.stream.get_rx_buffer_count = mux_get_rx_buffer_count;
            hal.stream.get_rx_buffer_free  = mux_get_rx_buffer_free;
            hal.stream.get_tx_buffer_count = mux_get_tx_buffer_count;
            hal.stream.reset_read_buffer   = mux_reset_read_buffer;
            hal.stream.cancel_read_buffer  = mux_cancel_read_buffer;
            hal.stream.reset_write_buffer  = mux_reset_write_buffer;

            // Disable grblHAL's onLinestateChanged hook by clearing
            // the flag. That hook schedules output_welcome_message
            // (200ms after each DTR-assert) which produces spurious
            // banners when USB peripheral has phantom DTR events. We
            // don't need re-prints of the banner mid-session — boot
            // banner is enough.
            hal.stream.state.linestate_event = Off;
            hal.stream.on_linestate_changed = NULL;

            mux_initialized = true;
        }
    }
#endif // !HIMILL_SKIP_MUX

    // MPG init runs whether or not the mux is installed. In mux mode, we
    // pre-register MPG to undo the hal.stream.write_all clobber from
    // stream_mpg_register. In isolation mode (HIMILL_SKIP_MUX), MPG runs
    // on top of stock grblHAL streams (USB CDC primary, UART4 secondary)
    // — useful to bisect whether the himill mux is contributing to the
    // SmartPendant MPG-toggle bug observed on D1S vs flexihal F4.
#if MPG_ENABLE == 2
    mpg_uart_stream = stream_open_instance(MPG_STREAM, 115200, NULL, NULL);
    hal.driver_cap.mpg_mode = stream_mpg_register(
        mpg_uart_stream, false, himill_mpg_rx_filter);
    if (hal.driver_cap.mpg_mode && mux_initialized)
        hal.stream.write_all = mux_write_all;
    // Install 0x8B debounce as mux_rt_handler. Only meaningful when the
    // mux's enqueue-rt wrappers are wired up (i.e. mux mode). In isolation
    // mode this static is set but unused — UART4-during-MPG-active will
    // route 0x8B through whatever stream_mpg_enable's swap installs.
    if (hal.driver_cap.mpg_mode && mux_initialized) {
        mux_rt_handler = himill_intercept_rt;
        mux_rt_owner_locked = false;
    }
#endif
}

#endif // BOARD_HIMILL_D1S
