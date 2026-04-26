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
#include "grbl/nvs_buffer.h"

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
    return mux_enqueue_rt_from(mux_usb, c);
}

static bool mux_uart_enqueue_rt (uint8_t c)
{
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
#define ESP_TUNNEL_WINDOW_MS 3000

static void himill_poll_buttons (sys_state_t state)
{
    // PC4 = front-panel + inside-door buttons (parallel-wired, active-low).
    bool pc4_pressed = !DIGITAL_IN(GPIOC, 4);
    if (pc4_pressed && !button_pc4_prev_pressed)
        grbl.enqueue_realtime_command(CMD_CYCLE_START);
    button_pc4_prev_pressed = pc4_pressed;

    // PB1 = door switch (active-low when door open per HiMill CSV).
    bool pb1_pressed = !DIGITAL_IN(GPIOB, 1);
    if (pb1_pressed && !button_pb1_prev_pressed)
        grbl.enqueue_realtime_command(CMD_SAFETY_DOOR);
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
// Coords are read live from G59.3 (seeded by share/d1s/d1_factory.nc).
// RPM/dwell/lift are compile-time constants today; promote to settings
// later if testers report different needs.

#define HIMILL_CLEAN_RPM      13000.0f
#define HIMILL_CLEAN_DWELL_S  3.0f
#define HIMILL_CLEAN_Z_LIFT   5.0f

typedef struct {
    bool enable;
} himill_settings_t;

static himill_settings_t himill_settings;
static nvs_address_t himill_nvs_address;
static on_tool_selected_ptr on_tool_selected;

static const setting_group_detail_t himill_setting_groups[] = {
    { Group_Root, Group_UserSettings, "HiMill D1S" }
};

static status_code_t himill_set_clean (setting_id_t id, uint_fast16_t value)
{
    himill_settings.enable = !!value;
    hal.nvs.memcpy_to_nvs(himill_nvs_address, (uint8_t *)&himill_settings, sizeof(himill_settings_t), true);
    return Status_OK;
}

static uint32_t himill_get_clean (setting_id_t id)
{
    return himill_settings.enable ? 1 : 0;
}

static const setting_detail_t himill_setting_detail[] = {
    { Setting_UserDefined_0, Group_UserSettings, "Clean toolsetter before M6", NULL, Format_Bool, NULL, NULL, NULL, Setting_NonCoreFn, himill_set_clean, himill_get_clean, NULL }
};

static void himill_settings_restore (void)
{
    himill_settings.enable = false;
    hal.nvs.memcpy_to_nvs(himill_nvs_address, (uint8_t *)&himill_settings, sizeof(himill_settings_t), true);
}

static void himill_settings_load (void)
{
    if (hal.nvs.memcpy_from_nvs((uint8_t *)&himill_settings, himill_nvs_address, sizeof(himill_settings_t), true) != NVS_TransferResult_OK)
        himill_settings_restore();
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
        hal.stream.write("[PLUGIN:HIMILL_D1S v0.2]" ASCII_EOL);
}

void board_init (void)
{
    // PD2 = TMC STEPPER_MODE pin. HIGH = SpreadCycle (fast/cutting),
    // LOW = StealthChop (silent). On the D1S the cooling fan runs
    // continuously regardless of mode, so the silence advantage of
    // StealthChop is moot — go straight to fast mode for the better
    // high-speed torque + the 2000 mm/min achievable on X/Y. This
    // mirrors what MaxMake's [LASER_MODE:1] does for laser jobs, just
    // applied permanently at boot.
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef gpio_init = {
        .Pin = GPIO_PIN_2,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Speed = GPIO_SPEED_FREQ_LOW
    };
    HAL_GPIO_Init(GPIOD, &gpio_init);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

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
}

#endif // BOARD_HIMILL_D1S
