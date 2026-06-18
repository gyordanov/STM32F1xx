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
#include "grbl/spindle_control.h"
#include "grbl/state_machine.h"
#include "grbl/nvs_buffer.h"
#include "grbl/system.h"
#if FS_ENABLE & FS_SDCARD
#include "ff.h"
#include "diskio.h"
#include "grbl/stream_file.h"
#include "grbl/vfs.h"
#include "sdcard/sdcard.h"

#ifndef D1S_DISKIO_DIAG_ENABLE
#define D1S_DISKIO_DIAG_ENABLE 0
#endif

#if D1S_DISKIO_DIAG_ENABLE
extern uint32_t diskio_d1s_diagnostics (void);
#endif
#endif

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
#define HIMILL_D1S_PLUGIN_NAME    "HIMILL_D1S"
#define HIMILL_D1S_PLUGIN_VERSION "0.12"
#define D1S_PROBE_PROTECT_POLL_MS 10U
#define D1S_PROBE_PROTECT_SAMPLES 2U

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
static on_report_handlers_init_ptr on_report_handlers_init;
static on_realtime_report_ptr  on_realtime_report;
static status_message_ptr      on_status_message;
static uint32_t d1s_probe_protect_last_ms = 0;
static uint8_t d1s_probe_protect_samples = 0;
// Protection arms only after the probe pin has read clear since the last probe
// cycle, so a G38 that stops in contact isn't misread as a crash. Default off
// is fail-safe: it self-arms on the first clear reading during normal motion.
static bool d1s_probe_protect_armed = false;

extern void *_sbrk (int incr);

static uint32_t himill_get_free_mem (void)
{
    register char *stack_ptr asm("sp");
    char *heap_end = (char *)_sbrk(0);

    return heap_end < stack_ptr ? (uint32_t)(stack_ptr - heap_end) : 0;
}

// Compact D1S reset/crash breadcrumbs.  LR=reset, LC=crash, LCC=clear.
#define D1S_RD_MAGIC 0xD15C0103UL
#ifndef D1S_RESET_DIAG_CLEAR_COMMAND
#define D1S_RESET_DIAG_CLEAR_COMMAND 1
#endif

enum {
    D1S_RD_MAGIC_W = 0,
    D1S_RD_SEQ_W,
    D1S_RD_CSR_W,
    D1S_RD_FAULT_W,
    D1S_RD_PC_W,
    D1S_RD_RA_W,
    D1S_RD_CS_W,
    D1S_RD_HS_W,
    D1S_RD_MS_W,
    D1S_RD_WORDS
};

static void d1s_rd_access (void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
}

static volatile uint32_t *d1s_bkp_dr (uint_fast8_t n)
{
    return n < 10 ? (&BKP->DR1 + n) : (&BKP->DR11 + (n - 10));
}

static uint32_t d1s_rd_get (uint_fast8_t word)
{
    uint_fast8_t reg = word << 1;
    return (*d1s_bkp_dr(reg) & 0xFFFFUL) | ((*d1s_bkp_dr(reg + 1) & 0xFFFFUL) << 16);
}

static void d1s_rd_set (uint_fast8_t word, uint32_t value)
{
    uint_fast8_t reg = word << 1;
    *d1s_bkp_dr(reg) = value & 0xFFFFUL;
    *d1s_bkp_dr(reg + 1) = value >> 16;
}

static void d1s_rd_init_if_needed (void)
{
    if(d1s_rd_get(D1S_RD_MAGIC_W) == D1S_RD_MAGIC)
        return;
    for(uint_fast8_t i = 0; i < D1S_RD_WORDS; i++)
        d1s_rd_set(i, 0);
    d1s_rd_set(D1S_RD_MAGIC_W, D1S_RD_MAGIC);
}

static char *d1s_hex32 (uint32_t v)
{
    static char buf[9];
    static const char hex[] = "0123456789ABCDEF";
    for(int_fast8_t i = 7; i >= 0; i--) {
        buf[i] = hex[v & 0x0FUL];
        v >>= 4;
    }
    buf[8] = '\0';
    return buf;
}

static void d1s_wrsephex (uint32_t v)
{
    hal.stream.write(",");
    hal.stream.write(d1s_hex32(v));
}

void himill_d1s_reset_diag_boot_capture (void)
{
    d1s_rd_access();
    uint32_t csr = RCC->CSR;
    d1s_rd_init_if_needed();
    d1s_rd_set(D1S_RD_SEQ_W, d1s_rd_get(D1S_RD_SEQ_W) + 1);
    d1s_rd_set(D1S_RD_CSR_W, csr);
    __DSB();
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

void himill_d1s_reset_diag_fault (uint32_t fault_type, uint32_t *stack)
{
    d1s_rd_access();
    d1s_rd_init_if_needed();
    d1s_rd_set(D1S_RD_FAULT_W, fault_type);
    d1s_rd_set(D1S_RD_PC_W, stack ? stack[6] : 0);
    d1s_rd_set(D1S_RD_RA_W, stack ? stack[5] : 0);
    d1s_rd_set(D1S_RD_CS_W, SCB->CFSR);
    d1s_rd_set(D1S_RD_HS_W, SCB->HFSR);
    d1s_rd_set(D1S_RD_MS_W, HAL_GetTick());
    __DSB();
    NVIC_SystemReset();
}

static void d1s_reset_diag_report (void)
{
    d1s_rd_access();
    d1s_rd_init_if_needed();

    hal.stream.write("[LR:");
    hal.stream.write(d1s_hex32(d1s_rd_get(D1S_RD_CSR_W)));
    hal.stream.write(",");
    hal.stream.write(uitoa(d1s_rd_get(D1S_RD_SEQ_W)));
    hal.stream.write("]" ASCII_EOL);

    uint32_t fault = d1s_rd_get(D1S_RD_FAULT_W);
    if(fault) {
        hal.stream.write("[LC:");
        hal.stream.write(uitoa(fault));
        d1s_wrsephex(d1s_rd_get(D1S_RD_PC_W));
        d1s_wrsephex(d1s_rd_get(D1S_RD_RA_W));
        d1s_wrsephex(d1s_rd_get(D1S_RD_CS_W));
        d1s_wrsephex(d1s_rd_get(D1S_RD_HS_W));
        hal.stream.write(",");
        hal.stream.write(uitoa(d1s_rd_get(D1S_RD_MS_W)));
        hal.stream.write(",");
        hal.stream.write(uitoa(d1s_rd_get(D1S_RD_SEQ_W)));
        hal.stream.write("]" ASCII_EOL);
    }
}

static void d1s_report_uptime (void)
{
    if(!hal.get_elapsed_ticks)
        return;

    uint32_t sec = hal.get_elapsed_ticks() / 1000U;
    uint32_t h = sec / 3600U;
    uint32_t m = (sec % 3600U) / 60U;
    uint32_t s = sec % 60U;

    hal.stream.write("[UPTIME:");
    hal.stream.write(uitoa(h));
    hal.stream.write("h ");
    hal.stream.write(uitoa(m));
    hal.stream.write("m ");
    hal.stream.write(uitoa(s));
    hal.stream.write("s]" ASCII_EOL);
}

#if D1S_RESET_DIAG_CLEAR_COMMAND
static status_code_t d1s_lcc_command (sys_state_t state, char *args)
{
    UNUSED(state);
    UNUSED(args);
    d1s_rd_access();
    d1s_rd_init_if_needed();
    for(uint_fast8_t i = D1S_RD_FAULT_W; i < D1S_RD_WORDS; i++)
        d1s_rd_set(i, 0);
    hal.stream.write("[LCC:OK]" ASCII_EOL);
    return Status_OK;
}

static const sys_command_t d1s_reset_diag_command_list[] = {
    {"LCC", d1s_lcc_command, { .noargs = On }, { .str = "" } }
};

static sys_commands_t d1s_reset_diag_commands = {
    .n_commands = sizeof(d1s_reset_diag_command_list) / sizeof(sys_command_t),
    .commands = d1s_reset_diag_command_list
};
#endif

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

// --- Single-active-controller arbitration (dual-connect lock) -------------
// NEWEST-WINS, matching grblHAL's stream_select on FlexiHAL: the most recently
// CONNECTED transport becomes the sole controller; the other becomes a
// read-only observer. Observer line input is DROPPED (not buffered/replayed —
// FlexiHAL replays the dormant stream's buffered input on hand-back, which is
// surprising; we discard instead), command responses go only to the controller,
// but mux_write_all status broadcasts still reach the observer (live DRO).
// Realtime (?/!/reset/overrides) is never gated here — it is stripped in the
// enqueue handler before mux_read.
//
// Claims fire on CONNECT EDGES, treating the two transports symmetrically:
//   - USB present  = DTR asserted (is_connected). Rising edge -> USB takes
//     control; falling edge -> hand back to NET if still present, else NONE.
//   - NET present  = ESP3D's [DN:U]/[DN:D] owner signals (the network-side
//     equivalent of USB's DTR). [DN:U] -> NET takes control; [DN:D] -> hand
//     back to USB if DTR still asserted, else NONE.
// USB edges are polled in himill_poll_buttons (mux_update_usb_presence); NET
// edges are handled where [DN:*] is parsed (himill_user_command). [BOOT] from
// USB always bypasses the lock (bracket-line exception in mux_line_gate) so
// flashing is never locked out. Graceful: until ESP3D emits [DN:*], NET never
// claims and behaviour matches the pre-lock mux.
//
// KNOWN LIMITATION (deferred 2026-06-16): the DTR-rising-edge takeover means a
// transient/phantom USB DTR assert (a host-side port scan/probe while a network
// session is active) can briefly steal control. A line-based USB claim (claim
// on the first USB DATA LINE instead of the DTR edge) was tried to fix that, but
// it HW-reproducibly made USB/desktop streaming choppy — steady-state mux_controller
// ends up identical, so the mechanism is not understood (both the steal and the
// choppiness smell like USB-DTR flapping under load). Reverted to this smooth
// rising-edge behaviour; revisit only with live mux_controller instrumentation
// (NGC-off debug build), not another blind change.
typedef enum {
    MUX_CTRL_NONE = 0,
    MUX_CTRL_USB,
    MUX_CTRL_NET
} mux_controller_t;

static mux_controller_t mux_controller = MUX_CTRL_NONE;
static bool d1s_net_present = false;       // NET present (ESP3D [DN:U] seen, no [DN:D] yet)
static bool mux_usb_was_connected = false; // previous USB DTR state, for edge detection
static bool mux_drop_line = false;         // dropping the current observer line in mux_read
static uint8_t mux_dtr_edges = 0;          // USB DTR edges seen (wraps 0-255); |MX: telemetry — rapid churn = phantom DTR flapping
static int32_t esp_tunnel_pending = SERIAL_NO_DATA;
static bool esp_tunnel_json_seen = false;
static uint8_t esp_tunnel_json_depth = 0;
static bool esp_tunnel_json_in_string = false;
static bool esp_tunnel_json_escape = false;
// Tunnel window for [ESP*] command responses: while non-zero, UART2 RX is
// owned exclusively by the himill_poll_buttons drainer that mirrors bytes
// to USB CDC TX. mux_read suppresses its USB→UART fallback while this is
// set so it doesn't race on the same RX buffer.
static volatile uint32_t  esp_tunnel_until_ms = 0;

// Guards the [ESP*] response drainer against re-entrancy: mux_usb->write_char
// blocks when USB CDC TX is full and spins on hal.stream_blocking_callback ->
// protocol_execute_realtime -> himill_poll_buttons, re-entering the drainer.
// A nested drain writes the next rxbuf bytes ahead of the outer pending byte
// and reorders ESP3D's reply (garbled JSON on >512 B replies — the DMA-RX
// build batches rxbuf fills, so a re-entrant drain always finds bytes to
// scramble; the per-byte ISR build trickled, masking it). True while the
// drainer runs so the re-entrant call skips and the outer write completes in
// order. USB TX still drains via the host/USB ISR, so the outer write unblocks.
static bool esp_tunnel_draining = false;

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
    // Under the single-controller lock, command responses (ok/error/reports)
    // go ONLY to the controller — the observer must not receive them (that is
    // what makes its sender show "another sender in control"). Status pushes
    // still reach the observer via mux_write_all. When unowned, fall back to the
    // legacy last-sender/active behaviour (no regression for single-client use).
    switch (mux_controller) {

        case MUX_CTRL_USB:
            if (mux_stream_connected(mux_usb))
                return mux_usb;
            break;

        case MUX_CTRL_NET:
            if (mux_stream_connected(mux_uart))
                return mux_uart;
            break;

        default:
            break;
    }

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

// Poll USB DTR for a connect/disconnect edge and apply newest-wins: a USB
// connect (DTR rising) takes control; a USB disconnect (DTR falling) hands
// control back to NET if it is still present, else NONE. Called every realtime
// tick from himill_poll_buttons so a USB connect takes over promptly even while
// USB is silent. is_connected() carries a 15 ms DTR debounce, so edges here are
// already debounced.
// Newest-wins takeover is allowed only when the machine is IDLE or in ALARM —
// never mid-operation. A second client connecting while a job runs (CYCLE), is
// paused (HOLD), jogging or homing does NOT steal control; it becomes a
// read-only observer and its sender sees the non-idle status and backs off
// (restores the pre-arbitration behaviour). ALARM is takeover-able so a fresh
// client can recover ($X). Gates only NEW claims — the existing controller
// keeps control across all states once claimed.
static bool mux_takeover_allowed (void)
{
    sys_state_t s = state_get();
    return s == STATE_IDLE || s == STATE_ALARM;
}

static void mux_update_usb_presence (void)
{
    bool now = mux_stream_connected(mux_usb);

    if (now == mux_usb_was_connected)
        return;

    mux_dtr_edges++;                               // telemetry: a real DTR transition

    if (now) {                                     // USB connect edge
        if (mux_takeover_allowed())                // newest-wins, but never steal a running job
            mux_controller = MUX_CTRL_USB;
    } else if (mux_controller == MUX_CTRL_USB)     // USB disconnect edge
        mux_controller = d1s_net_present ? MUX_CTRL_NET : MUX_CTRL_NONE;

    mux_usb_was_connected = now;
}

// Single-controller line gate. Returns true if byte c from stream s should be
// delivered to grbl's line parser, false if it must be swallowed (observer line
// dropped — NOT buffered). Claims are decided on connect edges elsewhere
// (mux_update_usb_presence for USB, [DN:*] for NET); this only enforces the
// drop. Only USB and the ESP3D UART are arbitrated; realtime bytes never reach
// here. Bracket-command lines ('[') from an observer are NOT dropped, so
// [BOOT]/[ESP*]/[D1S_*]/[DN:*] still reach their self-gating handlers. MUST be
// called BEFORE mux_note_rx_byte (it reads the pre-update mux_rx_line_owner to
// detect a line start).
static bool mux_line_gate (const io_stream_t *s, int32_t c)
{
    mux_controller_t cand = s == mux_usb  ? MUX_CTRL_USB
                          : s == mux_uart ? MUX_CTRL_NET
                          : MUX_CTRL_NONE;
    if (cand == MUX_CTRL_NONE)
        return true;

    if (mux_rx_line_owner == NULL)                 // first byte of a fresh line:
        mux_drop_line = mux_controller != MUX_CTRL_NONE   // observer? drop unless
                     && mux_controller != cand            // it is a bracket cmd
                     && (uint8_t)c != '[';

    return !mux_drop_line;
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
                {
                    bool deliver = mux_line_gate(s, c);
                    mux_note_rx_byte(s, c);
                    if (deliver)
                        return c;
                    continue;
                }

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

        {
            bool deliver = mux_line_gate(s, c);
            mux_note_rx_byte(s, c);
            if (deliver)
                return c;
            continue;
        }
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

    // Broadcast periodic/auto status to the MPG pendant (UART4) whenever it's
    // present — NOT only when MPG is engaged. Stock grblHAL's stream_write_all
    // iterates all connected streams (incl. the MPG), which is how a pendant
    // shows live DRO even while the sender is driving. mux_write_all doesn't
    // iterate that list; gating this branch on sys.mpg_mode left the D1S pendant
    // DRO stuck at 0 until MPG was toggled active, and not tracking sender jogs.
    // Broadcasting always is a superset of the old engage-only behaviour and
    // does NOT reintroduce the ~0.5 Hz 0x8B toggle loop — that loop was caused
    // by the pendant MISSING the MPG:1 report (too few reports), not too many.
    if (mpg_uart_stream && mpg_uart_stream->write)
        mpg_uart_stream->write(str);
}

static void mux_uart_write_direct (const char *str)
{
    if (mux_uart && mux_uart->write)
        mux_uart->write(str);
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
        mux_uart = props->claim(115200);  // matches stock ESP3D side
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

#if FS_ENABLE & FS_SDCARD
#define D1S_SD_LOCK_HEARTBEAT_MS 1000

static bool d1s_sd_lock_reported = false;
static uint32_t d1s_sd_lock_last_ms = 0;

static bool himill_sd_job_active (void);

static bool himill_sd_lock_write (const char *line)
{
    if(!(mux_uart && mux_uart->write))
        return false;

    mux_uart_write_direct(line);
    mux_uart_write_direct(ASCII_EOL);

    return true;
}

static void himill_sd_lock_poll (void)
{
    if(!hal.get_elapsed_ticks)
        return;

    uint32_t now = hal.get_elapsed_ticks();
    bool active = himill_sd_job_active();

    if(active) {
        if((!d1s_sd_lock_reported ||
             (now - d1s_sd_lock_last_ms) >= D1S_SD_LOCK_HEARTBEAT_MS) &&
            himill_sd_lock_write("[D1S_SD_LOCK:file]")) {
            d1s_sd_lock_reported = true;
            d1s_sd_lock_last_ms = now;
        }
    } else if(d1s_sd_lock_reported && himill_sd_lock_write("[D1S_SD_UNLOCK]")) {
        d1s_sd_lock_reported = false;
    }
}
#endif

static void himill_probe_protection_poll (sys_state_t state)
{
    if(!settings.probe.enable_protection || !hal.get_elapsed_ticks ||
        !hal.probe.is_triggered || !hal.control.interrupt_callback) {
        d1s_probe_protect_samples = 0;
        return;
    }

    // Disarm protection while a probe cycle is active. A G38 that stops AT
    // contact leaves the probe still triggered after sys.probing_state has
    // returned to Off (the machine is briefly still in CYCLE for the sender's
    // back-off rapid before the pin clears). Reading that legitimate post-probe
    // contact as a crash is what false-fired Alarm 13 mid-probe. Protection
    // re-arms only once the pin reads clear again (below), so back-off speed
    // and decel timing no longer matter.
    if(sys.probing_state != Probing_Off)
        d1s_probe_protect_armed = false;

    if(sys.probing_state != Probing_Off ||
        gc_state.tool_change || !(state & (STATE_CYCLE|STATE_JOG))
#if FS_ENABLE & FS_SDCARD
        || himill_sd_job_active()
#endif
    ) {
        d1s_probe_protect_samples = 0;
        return;
    }

    uint32_t now = hal.get_elapsed_ticks();
    if((now - d1s_probe_protect_last_ms) < D1S_PROBE_PROTECT_POLL_MS)
        return;

    d1s_probe_protect_last_ms = now;

    // 3D probe (PB2): guarded only while the spindle is off — a spinning tool
    // is expected to be in contact with stock. Toolsetter (PA1): guarded
    // regardless of spindle state — a drag knife (or any tool) driven into the
    // bed toolsetter outside a toolchange is always a crash. The legitimate
    // toolsetter tap during an M6 touch-off runs as a probing cycle ($341=3
    // semi-auto, toolsetter_auto_select), so it is already excluded above via
    // sys.probing_state. is_triggered() reads each probe by id (independent of
    // the selected probe), so the toolsetter is seen even when not selected.
    bool triggered = (!spindle_is_on() && hal.probe.is_triggered(Probe_Default))
                     || (hal.driver_cap.toolsetter && hal.probe.is_triggered(Probe_Toolsetter));

    if(!triggered) {
        d1s_probe_protect_armed = true;
        d1s_probe_protect_samples = 0;
        return;
    }

    // Triggered but not yet re-armed = the probe is still resting in contact
    // from the cycle that just finished. Ignore until it reads clear once.
    if(!d1s_probe_protect_armed) {
        d1s_probe_protect_samples = 0;
        return;
    }

    if(++d1s_probe_protect_samples < D1S_PROBE_PROTECT_SAMPLES)
        return;

    d1s_probe_protect_samples = 0;

    control_signals_t signals = {0};
    if(hal.control.get_state)
        signals.mask = hal.control.get_state().mask;
    signals.probe_triggered = On;
    hal.control.interrupt_callback(signals);
}

static void esp_tunnel_reset_response_state (void)
{
    esp_tunnel_json_seen = false;
    esp_tunnel_json_depth = 0;
    esp_tunnel_json_in_string = false;
    esp_tunnel_json_escape = false;
}

static bool esp_tunnel_note_response_byte (uint8_t c)
{
    if (esp_tunnel_json_in_string) {
        if (esp_tunnel_json_escape) {
            esp_tunnel_json_escape = false;
        } else if (c == '\\') {
            esp_tunnel_json_escape = true;
        } else if (c == '"') {
            esp_tunnel_json_in_string = false;
        }
        return false;
    }

    if (esp_tunnel_json_depth > 0 && c == '"') {
        esp_tunnel_json_in_string = true;
        return false;
    }

    if (c == '{') {
        esp_tunnel_json_seen = true;
        if (esp_tunnel_json_depth < UINT8_MAX)
            esp_tunnel_json_depth++;
        return false;
    }

    if (c == '}' && esp_tunnel_json_depth > 0) {
        esp_tunnel_json_depth--;
        return esp_tunnel_json_seen && esp_tunnel_json_depth == 0;
    }

    return false;
}

static void esp_tunnel_close (void)
{
    esp_tunnel_until_ms = 0;
    esp_tunnel_pending = SERIAL_NO_DATA;
    esp_tunnel_reset_response_state();
}

static void himill_poll_buttons (sys_state_t state)
{
    // Newest-wins arbitration: catch USB DTR connect/disconnect edges promptly
    // (even while USB is silent) so connecting a USB client takes control and
    // unplugging hands control back to the network side.
    mux_update_usb_presence();

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
    if (esp_tunnel_until_ms != 0 && !esp_tunnel_draining) {
        if ((int32_t)(hal.get_elapsed_ticks() - esp_tunnel_until_ms) >= 0) {
            esp_tunnel_close();
        } else if (mux_uart && mux_uart->read && mux_usb && mux_usb->write_char) {
            // Re-entrancy guard (see esp_tunnel_draining): write_char blocking on
            // a full USB TX re-enters himill_poll_buttons via the realtime
            // callback; a nested drain reorders the reply. The outer call owns
            // the drain; re-entrant calls skip (outer write still completes).
            esp_tunnel_draining = true;
            for (uint16_t i = 0; i < 64; i++) {  // bounded per-tick drain
                int32_t c = esp_tunnel_pending;
                if (c == SERIAL_NO_DATA)
                    c = mux_uart->read();
                if (c == SERIAL_NO_DATA)
                    break;
                if (mux_usb->write_char((uint8_t)c)) {
                    esp_tunnel_pending = SERIAL_NO_DATA;
                    if (esp_tunnel_note_response_byte((uint8_t)c)) {
                        esp_tunnel_close();
                        break;
                    }
                } else {
                    esp_tunnel_pending = c;
                    break;
                }
            }
            esp_tunnel_draining = false;
        }
    }

#if FS_ENABLE & FS_SDCARD
    himill_sd_lock_poll();
#endif

    himill_probe_protection_poll(state);

    if (on_execute_realtime)
        on_execute_realtime(state);
}

#if FS_ENABLE & FS_SDCARD
// Maximum time grblHAL will wait for ESP3D to return the shared SD bus when
// a grblHAL-side SD command is requested.
#define D1S_SD_TAKE_TIMEOUT_MS 1500

static bool d1s_sd_suppress_status = false;

static status_code_t himill_status_message (status_code_t status_code)
{
    if (d1s_sd_suppress_status) {
        d1s_sd_suppress_status = false;
        if (status_code == Status_OK || status_code == Status_Handled)
            return Status_OK;
    }

    return on_status_message ? on_status_message(status_code) : status_code;
}

static void himill_install_status_handler (void)
{
    if (grbl.report.status_message != himill_status_message) {
        on_status_message = grbl.report.status_message;
        grbl.report.status_message = himill_status_message;
    }
}

static void himill_report_handlers_init (void)
{
    if (on_report_handlers_init)
        on_report_handlers_init();

    himill_install_status_handler();
}

static bool himill_sd_job_active (void)
{
    // stream_is_file() only means file input is the current stream.
    // gc_state.file_stream stays true until the SD stream job is ended,
    // covering cycle-start waits, read-ahead/EOF gaps, buffered motion, and
    // spindle delays that can otherwise look idle to state_get().
    return stream_is_file() || gc_state.file_stream;
}

static bool himill_sd_reclaim_from_esp3d (void)
{
    if(!(mux_uart && mux_uart->write && mux_uart->read && hal.get_elapsed_ticks))
        return false;

    char line[32];
    uint_fast8_t line_len = 0;
    uint32_t start = hal.get_elapsed_ticks();

    // While waiting for the ACK/BUSY reply this loop consumes UART2 bytes
    // wholesale: realtime characters never reach the ring (mux_uart_enqueue_rt
    // intercepts them at enqueue time), but ordinary command lines ESP3D
    // forwards inside this window (<= D1S_SD_TAKE_TIMEOUT_MS) are dropped.
    // ESP3D must pause client forwarding between receiving [D1S_SD_TAKE]
    // and replying.
    mux_uart_write_direct("[D1S_SD_TAKE]" ASCII_EOL);

    do {
        int32_t c;

        while((c = mux_uart->read()) != SERIAL_NO_DATA) {
            if(c == ASCII_CR || c == ASCII_LF) {
                if(line_len == 0)
                    continue;

                line[line_len] = '\0';
                line_len = 0;

                if(!strcmp(line, "[D1S_SD_ACK]")) {
                    BYTE power = 0;
                    disk_ioctl(0, CTRL_POWER, &power);
                    return true;
                }

                if(!strncmp(line, "[D1S_SD_BUSY", 12))
                    return false;
            } else if(line_len < (sizeof(line) - 1))
                line[line_len++] = (char)c;
            else
                line_len = 0;
        }

        if(!protocol_execute_realtime())
            return false;

        hal.delay_ms(2, NULL);

    } while((hal.get_elapsed_ticks() - start) < D1S_SD_TAKE_TIMEOUT_MS);

    return false;
}

static status_code_t himill_sd_private_command_handled (void)
{
    d1s_sd_suppress_status = true;
    return Status_OK;
}

#define D1S_UPLOAD_PATH_MAX 128
#define D1S_UPLOAD_CHUNK_MAX 168
#define D1S_UPLOAD_ACK_WINDOW 8

typedef struct {
    bool active;
    vfs_file_t *file;
    char path[D1S_UPLOAD_PATH_MAX];
    uint32_t received;
    uint32_t seq;
    uint16_t crc;
} d1s_upload_t;

static d1s_upload_t d1s_upload = {0};

static uint16_t d1s_crc16_update (uint16_t crc, const uint8_t *data, size_t len)
{
    while(len--) {
        crc ^= (uint16_t)*data++;
        for(uint_fast8_t i = 0; i < 8; i++)
            crc = (crc >> 1) ^ (0xA001U & -(int16_t)(crc & 1));
    }

    return crc;
}

static void d1s_upload_send_line (const char *line)
{
    mux_uart_write_direct(line);
    mux_uart_write_direct(ASCII_EOL);
}

static void d1s_upload_send_error (const char *reason)
{
    mux_uart_write_direct("[D1SU:ERR:");
    mux_uart_write_direct(reason);
    d1s_upload_send_line("]");
}

static void d1s_upload_send_error_code (const char *reason, int code)
{
    mux_uart_write_direct("[D1SU:ERR:");
    mux_uart_write_direct(reason);
    mux_uart_write_direct(":");
    if(code < 0) {
        mux_uart_write_direct("-");
        code = -code;
    }
    mux_uart_write_direct(uitoa((uint32_t)code));
#if D1S_DISKIO_DIAG_ENABLE
    uint32_t diag = diskio_d1s_diagnostics();
    if(diag) {
        char hex[9];
        for(int i = 7; i >= 0; i--) {
            hex[i] = "0123456789ABCDEF"[diag & 0x0F];
            diag >>= 4;
        }
        hex[8] = '\0';
        mux_uart_write_direct(":");
        mux_uart_write_direct(hex);
    }
#endif
    d1s_upload_send_line("]");
}

static void d1s_upload_close (bool remove_file)
{
    char path[D1S_UPLOAD_PATH_MAX];

    strcpy(path, d1s_upload.path);
    if(d1s_upload.file) {
        vfs_close(d1s_upload.file);
        d1s_upload.file = NULL;
    }

    if(remove_file && *path)
        vfs_unlink(path);

    memset(&d1s_upload, 0, sizeof(d1s_upload));
    system_set_exec_state_flag(EXEC_STATUS_REPORT);
}

static bool d1s_upload_valid_path (const char *path)
{
    size_t len = strlen(path);

    if(len < 2 || path[0] != '/' || len >= D1S_UPLOAD_PATH_MAX)
        return false;

    while(*path) {
        if(*path == '[' || *path == ']' || *path == '|' || *path == ':' || *path == ASCII_CR || *path == ASCII_LF)
            return false;
        path++;
    }

    return true;
}

static bool d1s_upload_make_dirs (const char *path)
{
    char dir[D1S_UPLOAD_PATH_MAX];

    strcpy(dir, path);
    for(char *slash = dir + 1; *slash; slash++) {
        if(*slash == '/') {
            vfs_stat_t st;

            *slash = '\0';
            if(vfs_stat(dir, &st) != 0) {
                if(vfs_mkdir(dir) != 0)
                    return false;
            } else if(!st.st_mode.directory)
                return false;
            *slash = '/';
        }
    }

    return true;
}

static int d1s_hexval (char c)
{
    if(c >= '0' && c <= '9')
        return c - '0';
    if(c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if(c >= 'A' && c <= 'F')
        return c - 'A' + 10;

    return -1;
}

static bool d1s_parse_hex32 (const char *s, uint32_t *value)
{
    uint32_t v = 0;
    uint_fast8_t digits = 0;

    if(!s || *s == '\0')
        return false;

    while(*s) {
        int n = d1s_hexval(*s++);

        if(n < 0 || ++digits > 8)
            return false;

        v = (v << 4) | (uint32_t)n;
    }

    *value = v;

    return true;
}

static void d1s_upload_send_data_ack (uint32_t seq)
{
    char hex[9];

    for(int i = 7; i >= 0; i--) {
        hex[i] = "0123456789ABCDEF"[seq & 0x0F];
        seq >>= 4;
    }
    hex[8] = '\0';

    mux_uart_write_direct("[D1SU:OK:D:");
    mux_uart_write_direct(hex);
    d1s_upload_send_line("]");
}

static bool d1s_upload_decode_hex (const char *hex, uint8_t *data, uint_fast16_t len)
{
    for(uint_fast16_t i = 0; i < len; i++) {
        int hi = d1s_hexval(hex[i * 2]);
        int lo = d1s_hexval(hex[i * 2 + 1]);

        if(hi < 0 || lo < 0)
            return false;

        data[i] = (uint8_t)((hi << 4) | lo);
    }

    return hex[len * 2] == '\0';
}

static int d1s_b64val (char c)
{
    if(c >= 'A' && c <= 'Z')
        return c - 'A';
    if(c >= 'a' && c <= 'z')
        return c - 'a' + 26;
    if(c >= '0' && c <= '9')
        return c - '0' + 52;
    if(c == '+')
        return 62;
    if(c == '/')
        return 63;
    if(c == '=')
        return -2;

    return -1;
}

static bool d1s_upload_decode_base64 (const char *b64, uint8_t *data, uint_fast16_t len)
{
    size_t enc_len = strlen(b64);

    if(enc_len != ((len + 2U) / 3U) * 4U)
        return false;

    uint_fast16_t out = 0;
    for(size_t i = 0; i < enc_len; i += 4) {
        int v0 = d1s_b64val(b64[i]);
        int v1 = d1s_b64val(b64[i + 1]);
        int v2 = d1s_b64val(b64[i + 2]);
        int v3 = d1s_b64val(b64[i + 3]);

        if(v0 < 0 || v1 < 0 || v2 == -1 || v3 == -1)
            return false;

        uint32_t triple = ((uint32_t)v0 << 18) | ((uint32_t)v1 << 12);

        if(v2 >= 0)
            triple |= (uint32_t)v2 << 6;
        if(v3 >= 0)
            triple |= (uint32_t)v3;

        if(out < len)
            data[out++] = (uint8_t)(triple >> 16);
        if(v2 >= 0 && out < len)
            data[out++] = (uint8_t)(triple >> 8);
        if(v3 >= 0 && out < len)
            data[out++] = (uint8_t)triple;

        if((v2 < 0 || v3 < 0) && i + 4U != enc_len)
            return false;
    }

    return out == len;
}

static bool d1s_upload_strip_command (char *line)
{
    size_t len = strlen(line);

    if(len < 8 || line[len - 1] != ']')
        return false;

    line[len - 1] = '\0';

    return true;
}

static char *d1s_upload_next_field (char **cursor)
{
    char *field = *cursor, *sep;

    if(field == NULL)
        return NULL;

    sep = strchr(field, ':');
    if(sep) {
        *sep = '\0';
        *cursor = sep + 1;
    } else
        *cursor = NULL;

    return field;
}

static status_code_t d1s_upload_start (char *path)
{
    if(d1s_upload.active) {
        d1s_upload_send_error("busy");
        return himill_sd_private_command_handled();
    }

    if(state_get() != STATE_IDLE || himill_sd_job_active()) {
        d1s_upload_send_error("state");
        return himill_sd_private_command_handled();
    }

    if(!d1s_upload_valid_path(path)) {
        d1s_upload_send_error("path");
        return himill_sd_private_command_handled();
    }

    if(!sdcard_reclaim_from_external()) {
        d1s_upload_send_error("mount");
        return himill_sd_private_command_handled();
    }

    strcpy(d1s_upload.path, path);
    if(!d1s_upload_make_dirs(d1s_upload.path)) {
        d1s_upload_send_error_code("mkdir", vfs_errno);
        memset(&d1s_upload, 0, sizeof(d1s_upload));
        return himill_sd_private_command_handled();
    }

    vfs_unlink(d1s_upload.path);

    d1s_upload.file = vfs_open(d1s_upload.path, "w");
    if(!d1s_upload.file) {
        d1s_upload_send_error_code("open", vfs_errno);
        memset(&d1s_upload, 0, sizeof(d1s_upload));
        return himill_sd_private_command_handled();
    }

    d1s_upload.active = true;
    d1s_upload.crc = 0xFFFFU;
    d1s_upload_send_line("[D1SU:OK:S]");
    system_set_exec_state_flag(EXEC_STATUS_REPORT);

    return himill_sd_private_command_handled();
}

static status_code_t d1s_upload_data (char *cursor, bool base64)
{
    uint8_t data[D1S_UPLOAD_CHUNK_MAX];
    char *seq_s = d1s_upload_next_field(&cursor);
    char *len_s = d1s_upload_next_field(&cursor);
    char *crc_s = d1s_upload_next_field(&cursor);
    char *payload = cursor;

    if(!d1s_upload.active || !d1s_upload.file) {
        d1s_upload_send_error("inactive");
        return himill_sd_private_command_handled();
    }

    if(!seq_s || !len_s || !crc_s || !payload) {
        d1s_upload_send_error("syntax");
        d1s_upload_close(true);
        return himill_sd_private_command_handled();
    }

    uint32_t seq, len, expected_crc;

    if(!d1s_parse_hex32(seq_s, &seq) || !d1s_parse_hex32(len_s, &len) ||
        !d1s_parse_hex32(crc_s, &expected_crc) ||
        seq != d1s_upload.seq || len > D1S_UPLOAD_CHUNK_MAX ||
        !(base64 ? d1s_upload_decode_base64(payload, data, (uint_fast16_t)len)
                 : d1s_upload_decode_hex(payload, data, (uint_fast16_t)len))) {
        d1s_upload_send_error("frame");
        d1s_upload_close(true);
        return himill_sd_private_command_handled();
    }

    if(d1s_crc16_update(0xFFFFU, data, len) != (uint16_t)expected_crc) {
        d1s_upload_send_error("crc");
        d1s_upload_close(true);
        return himill_sd_private_command_handled();
    }

    if(vfs_write(data, len, 1, d1s_upload.file) != len) {
        d1s_upload_send_error_code("write", vfs_errno);
        d1s_upload_close(true);
        return himill_sd_private_command_handled();
    }

    d1s_upload.crc = d1s_crc16_update(d1s_upload.crc, data, len);
    d1s_upload.received += len;
    d1s_upload.seq++;

    if((d1s_upload.seq % D1S_UPLOAD_ACK_WINDOW) == 0)
        d1s_upload_send_data_ack(seq);

    return himill_sd_private_command_handled();
}

static status_code_t d1s_upload_end (char *cursor)
{
    char *size_s = d1s_upload_next_field(&cursor);
    char *crc_s = cursor;

    if(!d1s_upload.active || !d1s_upload.file || !size_s || !crc_s) {
        d1s_upload_send_error("inactive");
        d1s_upload_close(true);
        return himill_sd_private_command_handled();
    }

    uint32_t expected_size, expected_crc;

    if(!d1s_parse_hex32(size_s, &expected_size) || !d1s_parse_hex32(crc_s, &expected_crc)) {
        d1s_upload_send_error("syntax");
        d1s_upload_close(true);
        return himill_sd_private_command_handled();
    }

    bool ok = expected_size == d1s_upload.received && (uint16_t)expected_crc == d1s_upload.crc;

    vfs_close(d1s_upload.file);
    d1s_upload.file = NULL;

    if(ok)
        d1s_upload_send_line("[D1SU:DONE]");
    else {
        vfs_unlink(d1s_upload.path);
        d1s_upload_send_error("verify");
    }

    memset(&d1s_upload, 0, sizeof(d1s_upload));
    system_set_exec_state_flag(EXEC_STATUS_REPORT);

    return himill_sd_private_command_handled();
}

static status_code_t d1s_upload_abort (void)
{
    d1s_upload_close(true);
    d1s_upload_send_line("[D1SU:OK:A]");

    return himill_sd_private_command_handled();
}

static status_code_t d1s_upload_command (char *line)
{
    if(mux_last_rx_owner != mux_uart)
        return on_user_command ? on_user_command(line) : Status_Unhandled;

    if(!d1s_upload_strip_command(line)) {
        d1s_upload_send_error("syntax");
        return himill_sd_private_command_handled();
    }

    char *cursor = line + strlen("[D1SU:");
    char *op = d1s_upload_next_field(&cursor);

    if(!op) {
        d1s_upload_send_error("syntax");
        return himill_sd_private_command_handled();
    }

    switch(*op) {
        case 'S':
            if(cursor)
                return d1s_upload_start(cursor);
            d1s_upload_send_error("syntax");
            return himill_sd_private_command_handled();

        case 'D':
            return d1s_upload_data(cursor, false);

        case 'B':
            return d1s_upload_data(cursor, true);

        case 'E':
            return d1s_upload_end(cursor);

        case 'A':
            return d1s_upload_abort();

        default:
            d1s_upload_send_error("op");
            return himill_sd_private_command_handled();
    }
}

static void d1s_proxy_send_done (void)
{
    mux_uart_write_direct("[D1SP:DONE:OK]" ASCII_EOL);
}

static void d1s_proxy_send_error (const char *reason)
{
    mux_uart_write_direct("[D1SP:ERR:");
    mux_uart_write_direct(reason);
    mux_uart_write_direct("]" ASCII_EOL);
}

static void d1s_proxy_write_u64 (uint64_t value)
{
    char buf[21];
    uint_fast8_t i = sizeof(buf);

    buf[--i] = '\0';
    do {
        buf[--i] = '0' + (value % 10U);
        value /= 10U;
    } while(value && i);

    mux_uart_write_direct(&buf[i]);
}

static bool d1s_proxy_valid_path (const char *path)
{
    size_t len = strlen(path);

    if(len == 0 || path[0] != '/' || len >= D1S_UPLOAD_PATH_MAX)
        return false;

    while(*path) {
        if(*path == '[' || *path == ']' || *path == '|' || *path == ':' ||
            *path == ASCII_CR || *path == ASCII_LF)
            return false;
        path++;
    }

    return true;
}

static bool d1s_proxy_sd_ready (void)
{
    if(d1s_upload.active || sdcard_is_released_to_external() ||
        state_get() != STATE_IDLE || himill_sd_job_active()) {
        d1s_proxy_send_error("busy");
        return false;
    }

    if(!sdcard_reclaim_from_external()) {
        d1s_proxy_send_error("mount");
        return false;
    }

    return true;
}

static status_code_t d1s_proxy_list (const char *path)
{
    if(!d1s_proxy_valid_path(path)) {
        d1s_proxy_send_error("path");
        return himill_sd_private_command_handled();
    }

    if(!d1s_proxy_sd_ready())
        return himill_sd_private_command_handled();

    vfs_dir_t *dir = vfs_opendir(path);
    if(!dir) {
        d1s_proxy_send_error("open");
        return himill_sd_private_command_handled();
    }

    vfs_dirent_t *dirent;
    while((dirent = vfs_readdir(dir)) && *dirent->name) {
        if(dirent->st_mode.hidden)
            continue;

        mux_uart_write_direct("[D1SP:FILE:");
        mux_uart_write_direct(dirent->name);
        mux_uart_write_direct("|SIZE:");
        if(dirent->st_mode.directory)
            mux_uart_write_direct("-1");
        else
            d1s_proxy_write_u64((uint64_t)dirent->size);
        mux_uart_write_direct("]" ASCII_EOL);

        grbl.on_execute_realtime(state_get());
    }

    vfs_closedir(dir);

    vfs_free_t *free = vfs_fgetfree(path);
    if(free) {
        mux_uart_write_direct("[D1SP:STAT:");
        d1s_proxy_write_u64(free->size);
        mux_uart_write_direct(":");
        d1s_proxy_write_u64(free->used);
        mux_uart_write_direct("]" ASCII_EOL);
    }

    d1s_proxy_send_done();

    return himill_sd_private_command_handled();
}

static status_code_t d1s_proxy_action (char op, const char *path)
{
    int res = -1;

    if(!d1s_proxy_valid_path(path)) {
        d1s_proxy_send_error("path");
        return himill_sd_private_command_handled();
    }

    if(!d1s_proxy_sd_ready())
        return himill_sd_private_command_handled();

    switch(op) {
        case 'D':
            res = vfs_unlink(path);
            break;
        case 'M':
            res = vfs_mkdir(path);
            break;
        case 'R':
            res = vfs_rmdir(path);
            break;
    }

    if(res == 0)
        d1s_proxy_send_done();
    else
        d1s_proxy_send_error("fs");

    return himill_sd_private_command_handled();
}

static status_code_t d1s_proxy_command (char *line)
{
    if(mux_last_rx_owner != mux_uart)
        return on_user_command ? on_user_command(line) : Status_Unhandled;

    if(!d1s_upload_strip_command(line)) {
        d1s_proxy_send_error("syntax");
        return himill_sd_private_command_handled();
    }

    char *cursor = line + strlen("[D1SP:");
    char *op = d1s_upload_next_field(&cursor);

    if(!op || !cursor) {
        d1s_proxy_send_error("syntax");
        return himill_sd_private_command_handled();
    }

    switch(*op) {
        case 'L':
            return d1s_proxy_list(cursor);
        case 'D':
        case 'M':
        case 'R':
            return d1s_proxy_action(*op, cursor);
    }

    d1s_proxy_send_error("op");
    return himill_sd_private_command_handled();
}

static void himill_realtime_report (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if(d1s_upload.active) {
        stream_write("|SDU:");
        stream_write(uitoa(d1s_upload.received));
    } else if(sdcard_is_released_to_external())
        stream_write("|SDB:ESP3D");

    // Mux arbitration telemetry: |MX:<state>,<dtr-edges>
    //   state bits: 0-1 controller (0 NONE / 1 USB / 2 NET),
    //               2 USB present (DTR), 3 NET present ([DN:U] seen).
    //   dtr-edges: USB DTR transitions seen (wraps) — rapid churn = phantom flap.
    stream_write("|MX:");
    stream_write(uitoa((uint32_t)mux_controller
                       | (mux_stream_connected(mux_usb) ? 0x04u : 0u)
                       | (d1s_net_present ? 0x08u : 0u)));
    stream_write(",");
    stream_write(uitoa(mux_dtr_edges));

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}
#endif

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
#if FS_ENABLE & FS_SDCARD
    if (!strncmp(line, "[D1SU:", 6))
        return d1s_upload_command(line);

    if (!strncmp(line, "[D1SP:", 6))
        return d1s_proxy_command(line);

    if (strcmp(line, "[D1S_SD_REQ]") == 0) {
        if (mux_last_rx_owner != mux_uart)
            return on_user_command ? on_user_command(line) : Status_Unhandled;

        if (state_get() != STATE_IDLE)
            mux_uart_write_direct("[D1S_SD_BUSY:state]" ASCII_EOL);
        else if (himill_sd_job_active())
            mux_uart_write_direct("[D1S_SD_BUSY:file]" ASCII_EOL);
        // Mid-upload release would unmount FatFs under the open upload file.
        else if (d1s_upload.active)
            mux_uart_write_direct("[D1S_SD_BUSY:upload]" ASCII_EOL);
        else if (!sdcard_release_to_external())
            mux_uart_write_direct("[D1S_SD_BUSY:unmount]" ASCII_EOL);
        else
            mux_uart_write_direct("[D1S_SD_RDY]" ASCII_EOL);

        return himill_sd_private_command_handled();
    }

    if (!strcmp(line, "[D1S_SD_ACK]") || !strncmp(line, "[D1S_SD_BUSY", 12)) {
        if (mux_last_rx_owner != mux_uart)
            return on_user_command ? on_user_command(line) : Status_Unhandled;

        return himill_sd_private_command_handled();
    }

    if (strcmp(line, "[D1S_SD_DONE]") == 0) {
        if (mux_last_rx_owner != mux_uart)
            return on_user_command ? on_user_command(line) : Status_Unhandled;

        if(sdcard_reclaim_from_external_done())
            mux_uart_write_direct("[D1S_SD_ACK]" ASCII_EOL);
        else
            mux_uart_write_direct("[D1S_SD_BUSY:mount]" ASCII_EOL);

        return himill_sd_private_command_handled();
    }

    // Network owner connect/disconnect from ESP3D's WS arbitration — the
    // network-side equivalent of USB's DTR (newest-wins, dual-connect lock).
    // Only honour these from the UART (ESP3D) side. Consume silently (no ok echo
    // — the WS sender never sent this; an echo would desync its char-counter).
    // NOTE: the silent-consume reuses the SD suppress helper, so these live
    // inside the SD build guard; D1S always ships with SD. Without SD, NET never
    // claims (graceful — USB-only behaviour).
    if (strcmp(line, "[DN:U]") == 0) {                 // NET connect
        if (mux_last_rx_owner == mux_uart) {
            d1s_net_present = true;
            if (mux_takeover_allowed())                // newest-wins, but never steal a running job
                mux_controller = MUX_CTRL_NET;
        }
        return himill_sd_private_command_handled();
    }

    if (strcmp(line, "[DN:D]") == 0) {                 // NET disconnect
        if (mux_last_rx_owner == mux_uart) {
            d1s_net_present = false;
            if (mux_controller == MUX_CTRL_NET)        // hand back to USB if present
                mux_controller = mux_stream_connected(mux_usb) ? MUX_CTRL_USB
                                                               : MUX_CTRL_NONE;
        }
        return himill_sd_private_command_handled();
    }

#endif

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
            esp_tunnel_reset_response_state();
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
        hal.stream.write_all("[MSG:Laser ON]" ASCII_EOL);
        return Status_OK;
    }
    if (strcmp(line, "[LASER_MODE:0]") == 0) {
        if (laser_mode_active) {
            laser_mode_active = false;
            laser_pwm_deinit();
        }
        hal.stream.write_all("[MSG:Laser OFF]" ASCII_EOL);
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
        hal.stream.write_all("[MSG:DFU]" ASCII_EOL);
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

#define HIMILL_SETTINGS_MAGIC 0xD1

#define HIMILL_SETTING_CLEAN_ENABLE          0x01
#define HIMILL_SETTING_STEPPER_SPREADCYCLE   0x02
#define HIMILL_SETTING_RGB_STATUS_ENABLE     0x04
#define HIMILL_SETTINGS_DEFAULT_FLAGS        HIMILL_SETTING_STEPPER_SPREADCYCLE

typedef struct {
    uint8_t magic;
    uint8_t flags;
} himill_settings_t;

static himill_settings_t himill_settings;
static nvs_address_t himill_nvs_address;
static on_tool_selected_ptr on_tool_selected;

static const setting_group_detail_t himill_setting_groups[] = {
    { Group_Root, Group_UserSettings, "D1S" }
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
    himill_settings.magic = HIMILL_SETTINGS_MAGIC;
    hal.nvs.memcpy_to_nvs(himill_nvs_address, (uint8_t *)&himill_settings, sizeof(himill_settings_t), true);
}

static status_code_t himill_set_clean (setting_id_t id, uint_fast16_t value)
{
    if (value)
        himill_settings.flags |= HIMILL_SETTING_CLEAN_ENABLE;
    else
        himill_settings.flags &= ~HIMILL_SETTING_CLEAN_ENABLE;
    himill_settings_save();
    return Status_OK;
}

static uint32_t himill_get_clean (setting_id_t id)
{
    return (himill_settings.flags & HIMILL_SETTING_CLEAN_ENABLE) ? 1 : 0;
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
    { Setting_UserDefined_0, Group_UserSettings, "Clean", NULL, Format_Bool, NULL, NULL, NULL, Setting_NonCoreFn, himill_set_clean, himill_get_clean, NULL },
    { Setting_UserDefined_1, Group_UserSettings, "TMC", NULL, Format_RadioButtons, "S,C", NULL, NULL, Setting_NonCoreFn, himill_set_stepper_mode, himill_get_stepper_mode, NULL },
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

    if (settings.steppers.dir_invert.mask != DEFAULT_DIR_SIGNALS_INVERT_MASK ||
        settings.steppers.enable_invert.mask != DEFAULT_ENABLE_SIGNALS_INVERT_MASK) {
        settings.steppers.dir_invert.mask = DEFAULT_DIR_SIGNALS_INVERT_MASK;
        settings.steppers.enable_invert.mask = DEFAULT_ENABLE_SIGNALS_INVERT_MASK;
        changed = true;
    }

    if (!settings.homing.flags.enabled) {
        settings.homing.flags.enabled = On;
        changed = true;
    }

    if (settings.homing.dir_mask.value != DEFAULT_HOMING_DIR_MASK) {
        settings.homing.dir_mask.value = DEFAULT_HOMING_DIR_MASK;
        changed = true;
    }

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
        settings.probe.allow_feed_override || settings.probe.probe2_auto_select) {
        settings.probe.allow_feed_override = Off;
        settings.probe.soft_limited = On;
        settings.probe.toolsetter_auto_select = On;
        settings.probe.probe2_auto_select = Off;
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
    himill_settings.magic = HIMILL_SETTINGS_MAGIC;
    himill_settings.flags = HIMILL_SETTINGS_DEFAULT_FLAGS; // default: SpreadCycle, RGB/tool-clean off
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
    if (hal.nvs.memcpy_from_nvs((uint8_t *)&himill_settings, himill_nvs_address, sizeof(himill_settings_t), true) != NVS_TransferResult_OK ||
        himill_settings.magic != HIMILL_SETTINGS_MAGIC) {
        himill_settings_restore();
        himill_settings.magic = 0; // RAM-only marker for $I warning; NVS was repaired.
    }

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

    if (!(himill_settings.flags & HIMILL_SETTING_CLEAN_ENABLE))
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

    if (!newopt) {
        hal.stream.write("[PLUGIN:" HIMILL_D1S_PLUGIN_NAME " v" HIMILL_D1S_PLUGIN_VERSION);
        if (himill_settings.magic != HIMILL_SETTINGS_MAGIC)
            hal.stream.write(" ERR:SET");
        hal.stream.write("]" ASCII_EOL);
        d1s_report_uptime();
        d1s_reset_diag_report();
    }
}

void board_init (void)
{
    hal.get_free_mem = himill_get_free_mem;

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

    on_report_handlers_init = grbl.on_report_handlers_init;
    grbl.on_report_handlers_init = himill_report_handlers_init;
    himill_install_status_handler();

    on_report_options       = grbl.on_report_options;
    grbl.on_report_options  = on_report_options_himill;
#if D1S_RESET_DIAG_CLEAR_COMMAND
    system_register_commands(&d1s_reset_diag_commands);
#endif

#if FS_ENABLE & FS_SDCARD
    on_realtime_report      = grbl.on_realtime_report;
    grbl.on_realtime_report = himill_realtime_report;
#endif

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

#if FS_ENABLE & FS_SDCARD
            sdcard_set_reclaim_callback(himill_sd_reclaim_from_esp3d);
#endif

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
