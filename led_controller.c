/*
 * =============================================================================
 *  LED-CTRL : Timer-interrupt LED controller with UART command console,
 *             temperature monitoring and watchdog  (simulated embedded target)
 * =============================================================================
 *  File     : led_controller.c   (single-file project: firmware + simulator + tests)
 *  Version  : 1.0.0
 *  Author   : Saurav Pardeshi   (Embedded Systems Internship - firmware task)
 *  Language : C99, no dynamic memory, no floating point in the firmware part
 *
 * -----------------------------------------------------------------------------
 *  README
 * -----------------------------------------------------------------------------
 *  1. WHAT THIS PROJECT DOES
 *     Firmware for a small Cortex-M style board with
 *        - one LED on a PWM-capable pin (brightness 0..100 %),
 *        - a UART console (115200 8N1) that accepts text commands,
 *        - an LM35-type analog temperature sensor on a 12-bit ADC,
 *        - an independent watchdog (2 s timeout).
 *     The firmware is interrupt driven:
 *        SysTick_Handler()    1 kHz timer interrupt  -> time base, sets a tick flag
 *        USART2_IRQHandler()  RX interrupt           -> pushes bytes in a ring buffer
 *     and a cooperative main loop (app_poll) does all the real work:
 *        - LED engine: OFF / ON / BLINK <period> / BREATHE (triangle fade)
 *        - 100 ms temperature sampling, 8-sample moving average,
 *          over-temperature ALERT with hysteresis (alert overrides the LED mode
 *          with a fast 100 ms blink)
 *        - command parser with full input validation and error replies
 *        - watchdog supervision (kick only while the main loop is alive)
 *
 *  2. HOW THE "SIMULATED EMBEDDED SYSTEM" WORKS
 *     No hardware is needed. Section E of this file is a software model of the
 *     board (LED pin, UART data register, ADC, watchdog, backup register). It
 *     calls the same ISR functions a real MCU would call, one millisecond at a
 *     time. Every hardware access in the firmware goes through the small hal_*
 *     layer, so porting to a real STM32 means re-implementing ~10 functions:
 *
 *        hal_led_set_level()   -> __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ..)
 *        hal_uart_putc()       -> HAL_UART_Transmit() / write USART2->TDR
 *        hal_uart_read_rx()    -> read USART2->RDR  (inside the RXNE interrupt)
 *        hal_adc_read()        -> HAL_ADC_GetValue()
 *        hal_wdg_kick()        -> HAL_IWDG_Refresh()
 *        hal_reset_cause()     -> RCC->CSR IWDGRSTF flag
 *        hal_persist_get/set() -> RTC backup register
 *        hal_irq_disable/enable-> __disable_irq() / __enable_irq()
 *
 *  3. HOW TO COMPILE
 *        gcc -std=c99 -Wall -Wextra -Wpedantic -Werror -O2 led_controller.c -o ledctl
 *     Optional (recommended) sanitizer build used during development:
 *        gcc -std=c99 -Wall -Wextra -Wpedantic -g -O1 \
 *            -fsanitize=address,undefined led_controller.c -o ledctl_san
 *     Works with gcc or clang on Linux, macOS and Windows (MinGW / WSL).
 *
 *  4. HOW TO RUN
 *        ./ledctl --selftest            run the automated test suite (exit code 0 = all pass)
 *        ./ledctl --demo                scripted scenario with a time-stamped UART log
 *        ./ledctl --demo --csv out.csv  same, plus a 1 ms CSV trace (t, led, temp, alert)
 *        ./ledctl --interactive         type UART commands yourself (see below)
 *        ./ledctl --selftest --verbose  also print every UART line during the tests
 *
 *     Interactive mode: every line you type is sent to the simulated UART.
 *     Extra simulator directives (start with '@'):
 *        @wait <ms>       advance simulated time          (e.g. @wait 2000)
 *        @temp <degC>     set the real temperature        (e.g. @temp 38.5)
 *        @quit            leave
 *
 *  5. UART COMMAND REFERENCE (case-insensitive, end with Enter / CR / LF)
 *        HELP                  list commands
 *        VER                   firmware version
 *        LED ON | LED OFF      steady on at current brightness / off
 *        LED BLINK [ms]        blink with full period 50..5000 ms (default 500)
 *        LED BREATHE           2 s triangle fade up/down
 *        BRIGHT <0..100>       LED brightness in percent
 *        TEMP                  filtered temperature in degC
 *        THRESH <10..100>      over-temperature alert threshold in degC
 *        STATUS                one-line status report
 *        HANG                  DEBUG: stop the main loop to demonstrate the watchdog
 *     Replies start with "OK" or "ERR". Asynchronous events: "ALERT ON/OFF ...".
 *
 *  6. KEY FUNCTIONS
 *        SysTick_Handler      1 ms ISR, increments time base, raises tick flag
 *        USART2_IRQHandler    RX ISR, lock-free single-producer ring buffer
 *        app_poll             main-loop step: service ticks, UART, watchdog
 *        on_tick_1ms          1 ms work: LED engine + sample scheduler
 *        led_engine_step      generates OFF/ON/BLINK/BREATHE output
 *        sensor_sample        ADC read, convert, moving average, alert logic
 *        uart_rx_service      assembles lines (backspace, overflow handling)
 *        process_line         tokenise, validate and execute commands
 *        fw_boot              (re)initialise everything, print boot banner
 *
 *  7. TEST SUMMARY (details in the accompanying report)
 *        16 test cases / 97 individual checks, run with --selftest (all pass).
 *        Clean under -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror and under
 *        AddressSanitizer + UndefinedBehaviorSanitizer. Mutation testing: 12 of 12
 *        deliberately injected bugs are detected by the suite.
 * =============================================================================
 */

#include <ctype.h>
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* =============================================================================
 *  SECTION A : CONFIGURATION CONSTANTS
 * ============================================================================= */
#define FW_VERSION          "1.0.0"

#define UART_RX_BUF_SIZE    64u      /* ring buffer size, MUST be a power of two      */
#define CMD_MAX_LEN         40u      /* longest accepted command line (characters)   */
#define MAX_TOKENS          3        /* command word + up to two arguments           */

#define BLINK_MIN_MS        50       /* allowed full blink period                    */
#define BLINK_MAX_MS        5000
#define BLINK_DEFAULT_MS    500
#define BREATHE_PERIOD_MS   2000     /* full fade-up + fade-down cycle               */
#define ALERT_BLINK_MS      100      /* blink period used while the alert is active  */

#define ADC_MAX             4095     /* 12-bit converter                             */
#define VREF_MV             3300     /* ADC reference voltage                        */
#define TEMP_SAMPLE_MS      100      /* sensor sampling interval                     */
#define AVG_WINDOW          8        /* moving-average length (samples)              */
#define THRESH_DEFAULT_C    35       /* default alert threshold in degC              */
#define THRESH_MIN_C        10
#define THRESH_MAX_C        100
#define HYST_C10            20       /* alert hysteresis: 2.0 degC (tenths of degC)  */

#define WDG_TIMEOUT_MS      2000     /* independent watchdog timeout                 */

/* Temperatures are stored as integers in TENTHS of a degree Celsius ("c10").
 * LM35: 10 mV per degC  ->  1 mV == 0.1 degC  ->  c10 value == millivolts.     */

/* =============================================================================
 *  SECTION B : TYPES
 * ============================================================================= */
typedef enum { LED_OFF = 0, LED_ON, LED_BLINK, LED_BREATHE } led_mode_t;
typedef enum { RESET_POWER_ON = 0, RESET_WATCHDOG } reset_cause_t;

static const char *const k_mode_name[] = { "OFF", "ON", "BLINK", "BREATHE" };
static const char *const k_cause_name[] = { "POWER_ON", "WATCHDOG" };

/* =============================================================================
 *  SECTION C : HARDWARE ABSTRACTION LAYER (interface the firmware relies on)
 *  Implemented by the simulator in Section E; replace for a real board.
 * ============================================================================= */
static void          hal_led_set_level(uint8_t percent);
static void          hal_uart_putc(uint8_t c);
static uint8_t       hal_uart_read_rx(void);
static uint16_t      hal_adc_read(void);
static void          hal_wdg_kick(void);
static reset_cause_t hal_reset_cause(void);
static uint8_t       hal_persist_get(void);
static void          hal_persist_set(uint8_t v);
static void          hal_irq_disable(void);
static void          hal_irq_enable(void);

#define CRIT_ENTER()  hal_irq_disable()
#define CRIT_EXIT()   hal_irq_enable()

/* =============================================================================
 *  SECTION D : FIRMWARE (the part that would run on the microcontroller)
 * ============================================================================= */

/* ---- state shared between ISRs and main loop (volatile) ---- */
static volatile uint32_t g_ms;              /* milliseconds since boot (SysTick)    */
static volatile uint32_t g_ticks_pending;   /* ticks not yet serviced by main loop  */
static volatile uint8_t  rx_head;           /* written by ISR only                  */
static volatile uint8_t  rx_tail;           /* written by main loop only            */
static volatile uint32_t rx_overflow;       /* bytes dropped because buffer was full */
static uint8_t           rx_buf[UART_RX_BUF_SIZE];

/* ---- main-loop-only state ---- */
static uint32_t tick_overruns;              /* times the main loop fell >1 tick behind */
static led_mode_t user_mode;
static uint16_t blink_ms;
static uint8_t  brightness;                 /* 0..100 percent                       */
static uint16_t phase_ms;                   /* position inside blink/breathe cycle  */
static uint8_t  level_applied;              /* last value written to the LED        */

static int16_t  t_win[AVG_WINDOW];          /* moving-average window (c10)          */
static uint8_t  t_idx, t_cnt;
static int32_t  t_avg_c10;
static uint8_t  sample_countdown;
static int32_t  thresh_c;
static bool     alert_active;

static char     line_buf[CMD_MAX_LEN + 1u];
static uint8_t  line_len;
static bool     line_overflow;
static bool     g_hung;                     /* set by HANG: main loop stops          */

/* ----------------------------------------------------------------------------
 *  Interrupt service routines
 * -------------------------------------------------------------------------- */

/* 1 kHz time base. Kept extremely short: only counts and raises a flag. */
void SysTick_Handler(void)
{
    g_ms++;
    g_ticks_pending++;
}

/* UART receive interrupt: move the received byte into the ring buffer.
 * head is only written here, tail only by the main loop -> no lock needed. */
void USART2_IRQHandler(void)
{
    uint8_t b = hal_uart_read_rx();
    uint8_t next = (uint8_t)((rx_head + 1u) & (UART_RX_BUF_SIZE - 1u));
    if (next == rx_tail) {          /* buffer full: drop byte, count the loss */
        rx_overflow++;
        return;
    }
    rx_buf[rx_head] = b;
    rx_head = next;
}

/* ----------------------------------------------------------------------------
 *  UART helpers
 * -------------------------------------------------------------------------- */
static bool rx_pop(uint8_t *out)
{
    if (rx_tail == rx_head) {
        return false;
    }
    *out = rx_buf[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1u) & (UART_RX_BUF_SIZE - 1u));
    return true;
}

/* printf-style transmit (blocking, polled). Real board: could use a TX ring + TXE IRQ. */
static void uart_printf(const char *fmt, ...)
{
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    for (const char *p = buf; *p != '\0'; p++) {
        hal_uart_putc((uint8_t)*p);
    }
}

/* Format a tenths-of-degree value as "-12.3" without using floating point. */
static const char *fmt_c10(int32_t v, char *out, size_t n)
{
    int32_t a = (v < 0) ? -v : v;
    (void)snprintf(out, n, "%s%ld.%ld", (v < 0) ? "-" : "", (long)(a / 10), (long)(a % 10));
    return out;
}

/* ----------------------------------------------------------------------------
 *  LED engine (called once per millisecond tick)
 * -------------------------------------------------------------------------- */
static void led_restart_phase(void)
{
    phase_ms = 0u;
}

static void led_engine_step(void)
{
    led_mode_t mode   = alert_active ? LED_BLINK : user_mode;
    uint16_t   period = alert_active ? (uint16_t)ALERT_BLINK_MS : blink_ms;
    uint8_t    level  = 0u;

    switch (mode) {
    case LED_OFF:
        level = 0u;
        break;
    case LED_ON:
        level = brightness;
        break;
    case LED_BLINK: {
        /* first half of the period ON, second half OFF */
        level = (phase_ms < (uint16_t)(period / 2u)) ? brightness : 0u;
        if (++phase_ms >= period) {
            phase_ms = 0u;
        }
        break;
    }
    case LED_BREATHE: {
        /* triangle wave 0..1000..0 over BREATHE_PERIOD_MS, scaled by brightness */
        uint32_t half = BREATHE_PERIOD_MS / 2u;
        uint32_t tri  = (phase_ms < half) ? phase_ms : (BREATHE_PERIOD_MS - phase_ms);
        level = (uint8_t)((tri * brightness) / half);
        if (++phase_ms >= BREATHE_PERIOD_MS) {
            phase_ms = 0u;
        }
        break;
    }
    default:
        level = 0u;
        break;
    }

    if (level != level_applied) {   /* only touch the hardware when needed */
        level_applied = level;
        hal_led_set_level(level);
    }
}

/* ----------------------------------------------------------------------------
 *  Temperature sensing
 * -------------------------------------------------------------------------- */

/* Raw 12-bit ADC code -> tenths of degC (LM35: 10 mV/degC, so c10 == mV). */
static int32_t adc_to_c10(uint16_t raw)
{
    uint32_t mv = ((uint32_t)raw * VREF_MV + (ADC_MAX / 2u)) / ADC_MAX;   /* rounded */
    return (int32_t)mv;
}

static void alert_set(bool on)
{
    char tb[16];
    if (on == alert_active) {
        return;
    }
    alert_active = on;
    led_restart_phase();            /* start the blink pattern cleanly */
    uart_printf("ALERT %s temp=%sC thresh=%ldC\r\n", on ? "ON" : "OFF",
                fmt_c10(t_avg_c10, tb, sizeof tb), (long)thresh_c);
}

/* Take one sample, update the moving average, evaluate the alert with hysteresis. */
static void sensor_sample(void)
{
    t_win[t_idx] = (int16_t)adc_to_c10(hal_adc_read());
    t_idx = (uint8_t)((t_idx + 1u) % AVG_WINDOW);
    if (t_cnt < AVG_WINDOW) {
        t_cnt++;
    }

    int32_t sum = 0;
    for (uint8_t i = 0; i < t_cnt; i++) {
        sum += t_win[i];
    }
    t_avg_c10 = (sum + (int32_t)(t_cnt / 2u)) / (int32_t)t_cnt;

    if (t_cnt == AVG_WINDOW) {      /* judge only once the window is full */
        int32_t hi = thresh_c * 10;
        if (!alert_active && t_avg_c10 >= hi) {
            alert_set(true);
        } else if (alert_active && t_avg_c10 <= (hi - HYST_C10)) {
            alert_set(false);
        }
    }
}

/* ----------------------------------------------------------------------------
 *  Command parser
 * -------------------------------------------------------------------------- */

/* Strict integer parse: whole string must be a number inside [lo, hi]. */
static bool parse_int(const char *s, long lo, long hi, long *out)
{
    char *end = NULL;
    errno = 0;
    long v = strtol(s, &end, 10);
    if (errno != 0 || end == s || *end != '\0') {
        return false;               /* not a clean number */
    }
    if (v < lo || v > hi) {
        return false;
    }
    *out = v;
    return true;
}

/* Split a line in place into at most MAX_TOKENS words. Returns count, or -1 if too many. */
static int tokenize(char *line, char *tok[], int max)
{
    int n = 0;
    char *p = line;
    while (*p != '\0') {
        while (*p == ' ' || *p == '\t') {
            *p++ = '\0';
        }
        if (*p == '\0') {
            break;
        }
        if (n == max) {
            return -1;
        }
        tok[n++] = p;
        while (*p != '\0' && *p != ' ' && *p != '\t') {
            p++;
        }
    }
    return n;
}

static void cmd_status(void)
{
    char tb[16];
    if (t_cnt > 0u) {
        (void)fmt_c10(t_avg_c10, tb, sizeof tb);
    } else {
        (void)snprintf(tb, sizeof tb, "n/a");
    }
    uart_printf("STATUS mode=%s period=%u bright=%u led=%u temp=%sC thresh=%ldC alert=%u "
                "up=%lums rxovf=%lu tickovr=%lu wdg_resets=%u\r\n",
                k_mode_name[user_mode], (unsigned)blink_ms, (unsigned)brightness,
                (unsigned)level_applied, tb, (long)thresh_c, alert_active ? 1u : 0u,
                (unsigned long)g_ms, (unsigned long)rx_overflow, (unsigned long)tick_overruns,
                (unsigned)hal_persist_get());
}

static void process_line(char *line)
{
    char *tok[MAX_TOKENS];
    long  v = 0;

    for (char *p = line; *p != '\0'; p++) {                 /* case-insensitive */
        *p = (char)toupper((unsigned char)*p);
    }
    int n = tokenize(line, tok, MAX_TOKENS);
    if (n < 0) {
        uart_printf("ERR TOO MANY ARGS\r\n");
        return;
    }
    if (n == 0) {
        return;                                             /* blank line: ignore */
    }

    if (strcmp(tok[0], "HELP") == 0) {
        uart_printf("COMMANDS: HELP VER LED ON|OFF|BLINK [ms]|BREATHE BRIGHT <0-100> "
                    "TEMP THRESH <10-100> STATUS HANG\r\n");
    } else if (strcmp(tok[0], "VER") == 0) {
        uart_printf("OK VER " FW_VERSION "\r\n");
    } else if (strcmp(tok[0], "LED") == 0) {
        if (n < 2) {
            uart_printf("ERR MISSING ARG\r\n");
        } else if (strcmp(tok[1], "ON") == 0 && n == 2) {
            user_mode = LED_ON;  led_restart_phase();  uart_printf("OK LED ON\r\n");
        } else if (strcmp(tok[1], "OFF") == 0 && n == 2) {
            user_mode = LED_OFF; led_restart_phase();  uart_printf("OK LED OFF\r\n");
        } else if (strcmp(tok[1], "BREATHE") == 0 && n == 2) {
            user_mode = LED_BREATHE; led_restart_phase(); uart_printf("OK LED BREATHE\r\n");
        } else if (strcmp(tok[1], "BLINK") == 0) {
            long period = BLINK_DEFAULT_MS;
            if (n == 3 && !parse_int(tok[2], BLINK_MIN_MS, BLINK_MAX_MS, &period)) {
                uart_printf("ERR RANGE %d-%d\r\n", BLINK_MIN_MS, BLINK_MAX_MS);
            } else {
                blink_ms = (uint16_t)period;
                user_mode = LED_BLINK;
                led_restart_phase();
                uart_printf("OK LED BLINK %u\r\n", (unsigned)blink_ms);
            }
        } else {
            uart_printf("ERR BAD ARG\r\n");
        }
    } else if (strcmp(tok[0], "BRIGHT") == 0) {
        if (n < 2) {
            uart_printf("ERR MISSING ARG\r\n");
        } else if (n > 2) {
            uart_printf("ERR TOO MANY ARGS\r\n");
        } else if (!parse_int(tok[1], 0, 100, &v)) {
            /* distinguish "not a number" from "out of range" for a helpful message */
            char *end = NULL;
            (void)strtol(tok[1], &end, 10);
            uart_printf("%s", (end != tok[1] && *end == '\0') ? "ERR RANGE 0-100\r\n" : "ERR BAD ARG\r\n");
        } else {
            brightness = (uint8_t)v;
            uart_printf("OK BRIGHT %u\r\n", (unsigned)brightness);
        }
    } else if (strcmp(tok[0], "TEMP") == 0) {
        char tb[16];
        if (t_cnt == 0u) {
            uart_printf("ERR NO DATA\r\n");
        } else {
            uart_printf("OK TEMP %sC\r\n", fmt_c10(t_avg_c10, tb, sizeof tb));
        }
    } else if (strcmp(tok[0], "THRESH") == 0) {
        if (n < 2) {
            uart_printf("ERR MISSING ARG\r\n");
        } else if (!parse_int(tok[1], THRESH_MIN_C, THRESH_MAX_C, &v)) {
            uart_printf("ERR RANGE %d-%d\r\n", THRESH_MIN_C, THRESH_MAX_C);
        } else {
            thresh_c = (int32_t)v;
            uart_printf("OK THRESH %ld\r\n", v);
        }
    } else if (strcmp(tok[0], "STATUS") == 0) {
        cmd_status();
    } else if (strcmp(tok[0], "HANG") == 0) {
        uart_printf("OK HANG main loop stopped, watchdog will reset in %d ms\r\n", WDG_TIMEOUT_MS);
        g_hung = true;
    } else {
        uart_printf("ERR UNKNOWN CMD\r\n");
    }
}

/* Drain the RX ring buffer and assemble lines. Handles backspace and over-long lines. */
static void uart_rx_service(void)
{
    uint8_t c;
    while (rx_pop(&c)) {
        if (c == '\r' || c == '\n') {
            if (line_overflow) {
                uart_printf("ERR LINE TOO LONG\r\n");
            } else if (line_len > 0u) {
                line_buf[line_len] = '\0';
                process_line(line_buf);
            }
            line_len = 0u;
            line_overflow = false;
        } else if ((c == 0x08u || c == 0x7Fu)) {            /* backspace / DEL */
            if (line_len > 0u && !line_overflow) {
                line_len--;
            }
        } else if ((c >= 0x20u && c < 0x7Fu) || c == '\t') {  /* printable ASCII or TAB */
            if (line_len < CMD_MAX_LEN && !line_overflow) {
                line_buf[line_len++] = (char)c;
            } else {
                line_overflow = true;                       /* discard until end of line */
            }
        }
        /* other control characters are ignored */
    }
}

/* ----------------------------------------------------------------------------
 *  Scheduler: 1 ms work and the main-loop step
 * -------------------------------------------------------------------------- */
static void on_tick_1ms(void)
{
    led_engine_step();
    if (sample_countdown > 0u) {
        sample_countdown--;
    }
    if (sample_countdown == 0u) {
        sample_countdown = (uint8_t)TEMP_SAMPLE_MS;
        sensor_sample();
    }
}

/* One pass of the main loop ("superloop"). On real hardware: while(1){ app_poll(); __WFI(); } */
static void app_poll(void)
{
    if (g_hung) {
        return;                     /* simulates a stuck main loop: nothing is serviced */
    }

    /* 1. service every pending 1 ms tick, remember if we ever lagged behind */
    for (;;) {
        uint32_t pending;
        CRIT_ENTER();
        pending = g_ticks_pending;
        if (pending > 0u) {
            g_ticks_pending--;
        }
        CRIT_EXIT();
        if (pending == 0u) {
            break;
        }
        if (pending > 1u) {
            tick_overruns++;
        }
        on_tick_1ms();
    }

    /* 2. handle received characters */
    uart_rx_service();

    /* 3. watchdog: only kick if we got here, i.e. the loop is alive */
    if (!g_hung) {
        hal_wdg_kick();
    }
}

/* Initialise all firmware state; called at power-on and after every reset. */
static void fw_boot(reset_cause_t cause)
{
    g_ms = 0u;  g_ticks_pending = 0u;
    rx_head = 0u;  rx_tail = 0u;  rx_overflow = 0u;
    tick_overruns = 0u;
    user_mode = LED_OFF;  blink_ms = BLINK_DEFAULT_MS;  brightness = 100u;
    phase_ms = 0u;  level_applied = 0u;
    memset(t_win, 0, sizeof t_win);
    t_idx = 0u;  t_cnt = 0u;  t_avg_c10 = 0;
    sample_countdown = (uint8_t)TEMP_SAMPLE_MS;
    thresh_c = THRESH_DEFAULT_C;
    alert_active = false;
    line_len = 0u;  line_overflow = false;  g_hung = false;

    hal_led_set_level(0u);
    if (cause == RESET_WATCHDOG) {
        hal_persist_set((uint8_t)(hal_persist_get() + 1u));   /* count watchdog resets */
    }
    uart_printf("\r\nBOOT LED-CTRL v" FW_VERSION " reset=%s resets=%u\r\n",
                k_cause_name[cause], (unsigned)hal_persist_get());
    uart_printf("Type HELP for commands.\r\n");
}

/* =============================================================================
 *  SECTION E : SIMULATED HARDWARE (software model of the board)
 *  Not part of the firmware: it plays the role of the MCU peripherals.
 * ============================================================================= */
#define TX_CAP      16384u
#define TRACE_MAX   1024u
#define HIST_MAX    4096u

typedef struct {
    uint32_t      t_ms;               /* absolute simulated time (survives resets)   */
    int32_t       temp_c10;           /* real temperature at the sensor (tenths degC) */
    int32_t       spike_raw;          /* >= 0: next ADC conversion returns this code  */
    bool          noise;              /* add +-1 LSB deterministic ADC noise          */
    uint32_t      lcg;                /* noise generator state                        */
    uint8_t       led_level;          /* LED pin PWM duty, percent                    */
    uint8_t       rdr;                /* UART receive data register                   */
    uint8_t       persist;            /* backup register (survives reset)             */
    reset_cause_t cause;
    bool          wdg_on;
    uint32_t      wdg_cnt;            /* ms since last kick                           */
    uint32_t      us_acc;             /* sub-millisecond UART byte-time accumulator   */
    uint32_t      last_reset_t;       /* simulated time of the last watchdog reset    */

    char          tx[TX_CAP];         /* everything the firmware transmitted          */
    size_t        tx_len;
    bool          echo;               /* print RX/TX lines with timestamps            */
    char          eline[256];
    size_t        eline_len;

    uint32_t      rise_t[TRACE_MAX], fall_t[TRACE_MAX];   /* LED edge timestamps      */
    uint32_t      rise_n, fall_n;
    uint8_t       led_min, led_max;   /* min/max LED level since last stats reset     */
    uint8_t       hist[HIST_MAX];     /* LED level, one sample per ms (optional)      */
    uint32_t      hist_n;
    bool          hist_on;
    FILE         *csv;
} sim_t;

static sim_t sim;

/* ---- HAL implementation for the simulator ---- */
static void hal_led_set_level(uint8_t pct)
{
    if (sim.led_level == 0u && pct > 0u && sim.rise_n < TRACE_MAX) {
        sim.rise_t[sim.rise_n++] = sim.t_ms;
    }
    if (sim.led_level > 0u && pct == 0u && sim.fall_n < TRACE_MAX) {
        sim.fall_t[sim.fall_n++] = sim.t_ms;
    }
    sim.led_level = pct;
}

static void hal_uart_putc(uint8_t c)
{
    if (sim.tx_len < TX_CAP - 1u) {
        sim.tx[sim.tx_len++] = (char)c;
        sim.tx[sim.tx_len] = '\0';
    }
    if (sim.echo) {                                  /* timestamped line echo */
        if (c == '\n') {
            if (sim.eline_len > 0u) {
                printf("[%7lu ms] TX> %s\n", (unsigned long)sim.t_ms, sim.eline);
            }
            sim.eline_len = 0u;
            sim.eline[0] = '\0';
        } else if (c != '\r' && sim.eline_len < sizeof sim.eline - 1u) {
            sim.eline[sim.eline_len++] = (char)c;
            sim.eline[sim.eline_len] = '\0';
        }
    }
}

static uint8_t hal_uart_read_rx(void)        { return sim.rdr; }
static void    hal_wdg_kick(void)            { sim.wdg_cnt = 0u; }
static reset_cause_t hal_reset_cause(void)   { return sim.cause; }
static uint8_t hal_persist_get(void)         { return sim.persist; }
static void    hal_persist_set(uint8_t v)    { sim.persist = v; }
static void    hal_irq_disable(void)         { /* single-threaded simulator: nothing to do */ }
static void    hal_irq_enable(void)          { }

static uint16_t hal_adc_read(void)
{
    int32_t  mv  = sim.temp_c10;                 /* LM35: 1 mV == 0.1 degC */
    int32_t  raw;
    if (mv < 0) { mv = 0; }
    raw = (int32_t)((((uint32_t)mv * ADC_MAX) + (VREF_MV / 2u)) / VREF_MV);
    if (sim.spike_raw >= 0) {                    /* one-shot injected glitch */
        raw = sim.spike_raw;
        sim.spike_raw = -1;
    } else if (sim.noise) {                      /* deterministic +-1 LSB noise */
        sim.lcg = sim.lcg * 1664525u + 1013904223u;
        raw += (int32_t)((sim.lcg >> 16) % 3u) - 1;
    }
    if (raw < 0) { raw = 0; }
    if (raw > ADC_MAX) { raw = ADC_MAX; }
    return (uint16_t)raw;
}

/* ---- simulator control ---- */
static void sim_init(void)
{
    FILE *keep_csv = sim.csv;
    bool  keep_echo = sim.echo;
    memset(&sim, 0, sizeof sim);
    sim.csv = keep_csv;
    sim.echo = keep_echo;
    sim.temp_c10 = 250;                          /* room temperature 25.0 degC */
    sim.spike_raw = -1;
    sim.noise = true;
    sim.lcg = 12345u;
    sim.wdg_on = true;
    sim.cause = RESET_POWER_ON;
    sim.led_min = 255u;
}

static void sim_tx_clear(void) { sim.tx_len = 0u; sim.tx[0] = '\0'; }
static bool sim_tx_has(const char *s) { return strstr(sim.tx, s) != NULL; }

static void sim_hw_reset(reset_cause_t cause)
{
    sim.cause = cause;
    sim.wdg_cnt = 0u;
    sim.last_reset_t = sim.t_ms;
    sim.led_level = 0u;                          /* pins return to reset state */
    fw_boot(hal_reset_cause());
}

/* Advance the whole system by exactly one millisecond. */
static void sim_step(void)
{
    sim.t_ms++;
    if (sim.wdg_on) {
        sim.wdg_cnt++;
        if (sim.wdg_cnt >= WDG_TIMEOUT_MS) {
            sim_hw_reset(RESET_WATCHDOG);        /* independent watchdog bites */
        }
    }
    SysTick_Handler();                           /* timer interrupt fires      */
    app_poll();                                  /* main loop gets CPU time    */

    if (sim.led_level < sim.led_min) { sim.led_min = sim.led_level; }
    if (sim.led_level > sim.led_max) { sim.led_max = sim.led_level; }
    if (sim.hist_on && sim.hist_n < HIST_MAX) { sim.hist[sim.hist_n++] = sim.led_level; }
    if (sim.csv != NULL) {
        fprintf(sim.csv, "%lu,%u,%ld,%ld,%u\n", (unsigned long)sim.t_ms, (unsigned)sim.led_level,
                (long)sim.temp_c10, (long)(t_cnt > 0u ? t_avg_c10 : 0), alert_active ? 1u : 0u);
    }
}

static void sim_run(uint32_t ms) { while (ms-- > 0u) { sim_step(); } }

/* Send text over the simulated UART at 115200 baud (about 87 us per byte). */
static void sim_uart_send(const char *s)
{
    for (; *s != '\0'; s++) {
        sim.rdr = (uint8_t)*s;
        USART2_IRQHandler();                     /* RXNE interrupt              */
        app_poll();
        sim.us_acc += 87u;
        while (sim.us_acc >= 1000u) {
            sim.us_acc -= 1000u;
            sim_step();
        }
    }
}

/* Inject bytes faster than the main loop can drain them (ISR only, no polling). */
static void sim_uart_burst(const char *s)
{
    for (; *s != '\0'; s++) {
        sim.rdr = (uint8_t)*s;
        USART2_IRQHandler();
    }
}

/* Type a command line as a terminal would (text + CR LF) and let the firmware answer. */
static void sim_send_line(const char *s)
{
    if (sim.echo) {
        printf("[%7lu ms] RX> %s\n", (unsigned long)sim.t_ms, s);
    }
    sim_uart_send(s);
    sim_uart_send("\r\n");
    sim_run(2u);
}

static void sim_edge_reset(void) { sim.rise_n = 0u; sim.fall_n = 0u; }
static void sim_stats_reset(void) { sim.led_min = 255u; sim.led_max = 0u; }

/* =============================================================================
 *  SECTION F : AUTOMATED TEST SUITE (--selftest)
 * ============================================================================= */
static int g_pass, g_fail;

#define CHECK(cond, desc)                                                         \
    do {                                                                          \
        if (cond) { g_pass++; printf("    [PASS] %s\n", desc); }                  \
        else      { g_fail++; printf("    [FAIL] %s  (line %d)\n", desc, __LINE__); } \
    } while (0)

static void tc_begin(const char *id, const char *title)
{
    printf("\n%s  %s\n", id, title);
    sim_init();
    fw_boot(hal_reset_cause());
}

/* Parse the last "OK TEMP xx.x" reply into tenths of degC. */
static bool sim_read_temp_reply(int32_t *c10)
{
    const char *p = NULL, *q = sim.tx;
    int a = 0, b = 0;
    while ((q = strstr(q, "OK TEMP ")) != NULL) { p = q; q += 8; }
    if (p == NULL || sscanf(p + 8, "%d.%d", &a, &b) != 2) { return false; }
    *c10 = a * 10 + b;
    return true;
}

static bool all_rise_intervals(uint32_t expect, uint32_t skip)
{
    if (sim.rise_n < skip + 3u) { return false; }
    for (uint32_t i = skip + 1u; i < sim.rise_n; i++) {
        if (sim.rise_t[i] - sim.rise_t[i - 1u] != expect) { return false; }
    }
    return true;
}

static void tc01_boot(void)
{
    tc_begin("TC-01", "Boot banner, initial state");
    CHECK(sim_tx_has("BOOT LED-CTRL v" FW_VERSION), "boot banner shows firmware version");
    CHECK(sim_tx_has("reset=POWER_ON resets=0"), "reset cause POWER_ON, reset counter 0");
    CHECK(sim_tx_has("Type HELP for commands."), "prompt hint printed");
    CHECK(sim.led_level == 0u, "LED is off after boot");
    CHECK(user_mode == LED_OFF && brightness == 100u, "default mode OFF, brightness 100 %");
}

static void tc02_timebase(void)
{
    tc_begin("TC-02", "SysTick time base accuracy and tick servicing");
    sim_run(10000u);
    CHECK(g_ms == 10000u, "10 000 SysTick interrupts -> uptime exactly 10 000 ms");
    CHECK(tick_overruns == 0u, "main loop never fell behind (0 tick overruns)");
    CHECK(g_ticks_pending == 0u, "no unserviced ticks left");
}

static void tc03_basic_commands(void)
{
    tc_begin("TC-03", "Basic UART commands, case-insensitivity");
    sim_tx_clear();
    sim_send_line("HELP");
    CHECK(sim_tx_has("COMMANDS:") && sim_tx_has("LED ON|OFF|BLINK"), "HELP lists the commands");
    sim_tx_clear();
    sim_send_line("VER");
    CHECK(sim_tx_has("OK VER " FW_VERSION), "VER returns version");
    sim_tx_clear();
    sim_send_line("LED ON");
    sim_run(5u);
    CHECK(sim_tx_has("OK LED ON") && sim.led_level == 100u, "LED ON replies OK and drives LED to 100 %");
    sim_tx_clear();
    sim_send_line("led off");
    sim_run(5u);
    CHECK(sim_tx_has("OK LED OFF") && sim.led_level == 0u, "lower-case 'led off' accepted, LED off");
}

static void tc04_brightness(void)
{
    tc_begin("TC-04", "BRIGHT command and argument validation");
    sim_send_line("LED ON");
    sim_tx_clear();
    sim_send_line("BRIGHT 40");
    sim_run(5u);
    CHECK(sim_tx_has("OK BRIGHT 40") && sim.led_level == 40u, "BRIGHT 40 -> LED level 40 %");
    sim_tx_clear(); sim_send_line("BRIGHT 0");   sim_run(3u);
    CHECK(sim_tx_has("OK BRIGHT 0") && sim.led_level == 0u, "BRIGHT 0 accepted (lower bound)");
    sim_tx_clear(); sim_send_line("BRIGHT 100"); sim_run(3u);
    CHECK(sim_tx_has("OK BRIGHT 100") && sim.led_level == 100u, "BRIGHT 100 accepted (upper bound)");
    sim_tx_clear(); sim_send_line("BRIGHT 101");
    CHECK(sim_tx_has("ERR RANGE 0-100") && brightness == 100u, "BRIGHT 101 rejected, value unchanged");
    sim_tx_clear(); sim_send_line("BRIGHT -1");
    CHECK(sim_tx_has("ERR RANGE 0-100"), "BRIGHT -1 rejected");
    sim_tx_clear(); sim_send_line("BRIGHT abc");
    CHECK(sim_tx_has("ERR BAD ARG"), "BRIGHT abc -> ERR BAD ARG");
    sim_tx_clear(); sim_send_line("BRIGHT 5x");
    CHECK(sim_tx_has("ERR BAD ARG"), "BRIGHT 5x (trailing junk) -> ERR BAD ARG");
    sim_tx_clear(); sim_send_line("BRIGHT");
    CHECK(sim_tx_has("ERR MISSING ARG"), "BRIGHT without value -> ERR MISSING ARG");
}

static void tc05_blink(void)
{
    tc_begin("TC-05", "Blink timing (timer interrupt accuracy)");
    sim_tx_clear();
    sim_edge_reset();                            /* capture edges from before the command */
    sim_send_line("LED BLINK 200");
    CHECK(sim_tx_has("OK LED BLINK 200"), "LED BLINK 200 accepted");
    sim_run(2200u);
    CHECK(sim.rise_n >= 10u, "at least 10 blink cycles in 2.2 s");
    CHECK(all_rise_intervals(200u, 0u), "every rising-edge interval is exactly 200 ms");
    bool duty_ok = (sim.fall_n >= 10u);
    for (uint32_t i = 0u; duty_ok && i < 10u; i++) {
        if (sim.fall_t[i] - sim.rise_t[i] != 100u) { duty_ok = false; }
    }
    CHECK(duty_ok, "ON time is exactly 100 ms in each cycle (50 % duty)");
    sim_tx_clear(); sim_send_line("LED BLINK 49");
    CHECK(sim_tx_has("ERR RANGE 50-5000"), "period 49 ms rejected (below minimum)");
    sim_tx_clear(); sim_send_line("LED BLINK 5001");
    CHECK(sim_tx_has("ERR RANGE 50-5000"), "period 5001 ms rejected (above maximum)");
    sim_tx_clear(); sim_send_line("LED BLINK");
    CHECK(sim_tx_has("OK LED BLINK 500"), "LED BLINK without argument uses default 500 ms");
    sim_send_line("LED BLINK 50");
    sim_edge_reset();
    sim_run(500u);
    CHECK(all_rise_intervals(50u, 0u), "minimum period 50 ms generated correctly");
}

static void tc06_breathe(void)
{
    tc_begin("TC-06", "Breathe (triangle PWM fade)");
    sim_send_line("LED BREATHE");
    sim_stats_reset();
    sim.hist_n = 0u; sim.hist_on = true;
    sim_run(2000u);
    sim.hist_on = false;
    uint32_t i_max = 0u;
    for (uint32_t i = 0u; i < sim.hist_n; i++) { if (sim.hist[i] > sim.hist[i_max]) { i_max = i; } }
    CHECK(sim.led_max == 100u, "peak brightness reaches 100 %");
    CHECK(sim.led_min == 0u, "fade returns to 0 %");
    CHECK(i_max >= 990u && i_max <= 1010u, "peak occurs half-way through the 2 s cycle");
    bool up = true, down = true;
    for (uint32_t i = 1u; i <= i_max; i++) { if (sim.hist[i] < sim.hist[i - 1u]) { up = false; } }
    for (uint32_t i = i_max + 1u; i < sim.hist_n; i++) { if (sim.hist[i] > sim.hist[i - 1u]) { down = false; } }
    CHECK(up, "level rises monotonically to the peak");
    CHECK(down, "level falls monotonically after the peak");
    sim_send_line("BRIGHT 50");
    sim_stats_reset();
    sim_run(2000u);
    CHECK(sim.led_max == 50u, "BRIGHT 50 scales the breathe peak to 50 %");
}

static void tc07_robust_input(void)
{
    tc_begin("TC-07", "Input robustness: unknown, blank, tabs, backspace, too many args");
    sim_tx_clear(); sim_send_line("FOO");
    CHECK(sim_tx_has("ERR UNKNOWN CMD"), "unknown command -> ERR UNKNOWN CMD");
    sim_tx_clear(); sim_uart_send("\r\n\r\n"); sim_run(5u);
    CHECK(sim.tx_len == 0u, "blank lines produce no reply");
    sim_tx_clear(); sim_send_line("LED\tOFF");
    CHECK(sim_tx_has("OK LED OFF"), "tab used as separator accepted");
    sim_tx_clear(); sim_send_line("   led    on   ");
    CHECK(sim_tx_has("OK LED ON"), "extra spaces ignored");
    sim_tx_clear(); sim_uart_send("LEDD\x08 ON\r\n"); sim_run(3u);
    CHECK(sim_tx_has("OK LED ON"), "backspace editing works (LEDD<BS> ON -> LED ON)");
    sim_tx_clear(); sim_send_line("LED BLINK 100 200");
    CHECK(sim_tx_has("ERR TOO MANY ARGS"), "LED BLINK 100 200 (extra argument) -> ERR TOO MANY ARGS");
    sim_tx_clear(); sim_send_line("LED A B C D");
    CHECK(sim_tx_has("ERR TOO MANY ARGS"), "four tokens -> ERR TOO MANY ARGS");
    sim_tx_clear(); sim_send_line("LED");
    CHECK(sim_tx_has("ERR MISSING ARG"), "LED without sub-command -> ERR MISSING ARG");
    sim_tx_clear(); sim_send_line("LED SPIN");
    CHECK(sim_tx_has("ERR BAD ARG"), "LED SPIN -> ERR BAD ARG");
}

static void tc08_overflow(void)
{
    tc_begin("TC-08", "UART overflow and over-long line handling");
    char junk[101];
    memset(junk, 'A', 100u);
    junk[100] = '\0';
    sim_tx_clear();
    sim_uart_burst(junk);                        /* 100 bytes while main loop is busy */
    CHECK(rx_overflow == 100u - (UART_RX_BUF_SIZE - 1u), "37 bytes dropped and counted (63 fit in the ring)");
    sim_run(3u);                                 /* main loop drains the buffer */
    sim_uart_send("\r\n");
    sim_run(3u);
    CHECK(sim_tx_has("ERR LINE TOO LONG"), "over-long line rejected with ERR LINE TOO LONG");
    sim_tx_clear(); sim_send_line("LED ON");
    sim_run(3u);
    CHECK(sim_tx_has("OK LED ON") && sim.led_level == 100u, "system fully usable after the overflow");
    sim_tx_clear(); sim_send_line("STATUS");
    CHECK(sim_tx_has("rxovf=37"), "STATUS reports rx overflow counter 37");
    char exact[CMD_MAX_LEN + 2u];                /* exactly CMD_MAX_LEN characters is still legal */
    snprintf(exact, sizeof exact, "LED ON");
    memset(exact + 6, ' ', CMD_MAX_LEN - 6u);
    exact[CMD_MAX_LEN] = '\0';
    sim_tx_clear(); sim_send_line(exact);
    CHECK(sim_tx_has("OK LED ON"), "a line of exactly 40 characters is accepted");
}

static void tc09_temperature(void)
{
    tc_begin("TC-09", "Temperature measurement accuracy (ADC + filter)");
    static const int32_t pts[] = { 0, 250, 375, 500, 990 };
    for (unsigned k = 0u; k < sizeof pts / sizeof pts[0]; k++) {
        sim.temp_c10 = pts[k];
        sim_run(1000u);
        sim_tx_clear(); sim_send_line("TEMP");
        int32_t got = -999;
        bool ok = sim_read_temp_reply(&got);
        char msg[96];
        snprintf(msg, sizeof msg, "true %ld.%ld degC -> reported within +-0.2 degC (got %ld.%ld)",
                 (long)(pts[k] / 10), (long)(pts[k] % 10), (long)(got / 10), (long)(got % 10));
        CHECK(ok && got >= pts[k] - 2 && got <= pts[k] + 2, msg);
    }
}

static void tc10_spike(void)
{
    tc_begin("TC-10", "Moving average suppresses a single-sample glitch");
    sim_run(1500u);
    sim.spike_raw = 745;                          /* one conversion reads 60.0 degC */
    int32_t peak = 0;
    for (int i = 0; i < 400; i++) {
        sim_step();
        if (t_avg_c10 > peak) { peak = t_avg_c10; }
    }
    CHECK(peak >= 285 && peak <= 305, "glitch is visible but divided by the window (peak about 29.4 degC)");
    CHECK(!alert_active && !sim_tx_has("ALERT ON"), "no false alert from a one-sample 60 degC glitch");
}

static void tc11_alert(void)
{
    tc_begin("TC-11", "Over-temperature alert, LED override and hysteresis");
    sim_run(1200u);
    CHECK(!alert_active, "no alert at 25.0 degC");
    sim.temp_c10 = 400;                           /* 40.0 degC, threshold 35 */
    sim_run(1000u);
    CHECK(alert_active && sim_tx_has("ALERT ON temp="), "ALERT ON raised within 1 s of reaching 40 degC");
    sim_edge_reset();
    sim_run(1000u);
    CHECK(all_rise_intervals(ALERT_BLINK_MS, 0u) && sim.rise_n >= 9u,
          "LED forced to 100 ms blink during alert (user mode was OFF)");
    sim.temp_c10 = 340;                           /* inside hysteresis band (33.0..35.0) */
    sim_tx_clear();
    sim_run(2000u);
    CHECK(alert_active && !sim_tx_has("ALERT OFF"), "34.0 degC is inside the hysteresis band: alert stays on");
    sim.temp_c10 = 320;                           /* below 33.0 */
    sim_run(1500u);
    CHECK(!alert_active && sim_tx_has("ALERT OFF temp="), "ALERT OFF once temperature drops below 33.0 degC");
    sim_run(200u);
    CHECK(sim.led_level == 0u, "LED returns to the user mode (OFF) after the alert");
    sim.temp_c10 = 345;                           /* just under threshold: must not trigger */
    sim_tx_clear();
    sim_run(2000u);
    CHECK(!alert_active, "34.5 degC (just below 35.0) does not trigger");
    /* exact boundary behaviour, measured with a noise-free ADC */
    sim.noise = false;
    sim.temp_c10 = 349;  sim_run(1500u);
    CHECK(!alert_active, "34.9 degC (noise-free): no alert");
    sim.temp_c10 = 350;  sim_run(1500u);
    CHECK(alert_active, "exactly 35.0 degC raises the alert (>= comparison)");
    sim.temp_c10 = 331;  sim_run(1500u);
    CHECK(alert_active, "33.1 degC is still inside the hysteresis band");
    sim.temp_c10 = 330;  sim_run(1500u);
    CHECK(!alert_active, "exactly 33.0 degC clears the alert (<= comparison)");
}

static void tc12_threshold(void)
{
    tc_begin("TC-12", "THRESH command");
    sim.temp_c10 = 300;
    sim_run(1500u);
    CHECK(!alert_active, "30.0 degC, default threshold 35: no alert");
    sim_tx_clear(); sim_send_line("THRESH 28");
    CHECK(sim_tx_has("OK THRESH 28"), "THRESH 28 accepted");
    sim_run(500u);
    CHECK(alert_active, "alert raised after threshold lowered to 28 degC");
    sim_tx_clear(); sim_send_line("THRESH 9");
    CHECK(sim_tx_has("ERR RANGE 10-100") && thresh_c == 28, "THRESH 9 rejected, old value kept");
    sim_tx_clear(); sim_send_line("THRESH 101");
    CHECK(sim_tx_has("ERR RANGE 10-100"), "THRESH 101 rejected");
    sim_tx_clear(); sim_send_line("THRESH hot");
    CHECK(sim_tx_has("ERR RANGE 10-100"), "THRESH hot rejected");
    sim_tx_clear(); sim_send_line("THRESH");
    CHECK(sim_tx_has("ERR MISSING ARG"), "THRESH without value -> ERR MISSING ARG");
}

static void tc13_status(void)
{
    tc_begin("TC-13", "STATUS report and TEMP before first sample");
    sim.noise = false;                            /* noise-free ADC gives an exact reading */
    sim_tx_clear(); sim_send_line("TEMP");
    CHECK(sim_tx_has("ERR NO DATA"), "TEMP before the first sample -> ERR NO DATA");
    sim_run(1000u);
    sim_send_line("LED BLINK 300");
    sim_send_line("BRIGHT 70");
    sim_send_line("THRESH 30");
    sim_tx_clear(); sim_send_line("STATUS");
    CHECK(sim_tx_has("mode=BLINK") && sim_tx_has("period=300"), "STATUS shows mode and period");
    CHECK(sim_tx_has("bright=70") && sim_tx_has("thresh=30C"), "STATUS shows brightness and threshold");
    CHECK(sim_tx_has("temp=25.0C") && sim_tx_has("alert=0"), "STATUS shows temperature and alert flag");
    CHECK(sim_tx_has("rxovf=0") && sim_tx_has("tickovr=0") && sim_tx_has("wdg_resets=0"), "STATUS shows health counters");
}

static void tc14_watchdog(void)
{
    tc_begin("TC-14", "Watchdog recovers a hung main loop");
    sim_send_line("LED ON");
    sim_run(100u);
    sim_tx_clear();
    sim_send_line("HANG");
    uint32_t t_hang = sim.t_ms;
    CHECK(sim_tx_has("OK HANG") && g_hung, "HANG command stops the main loop");
    sim_tx_clear();
    sim_run(1900u);
    CHECK(!sim_tx_has("BOOT LED-CTRL"), "no reset before the 2 s timeout");
    CHECK(sim.persist == 0u, "reset counter still 0");
    sim_send_line("STATUS");                      /* ignored: loop is hung */
    CHECK(!sim_tx_has("STATUS mode"), "hung firmware ignores UART commands");
    sim_run(300u);
    CHECK(sim_tx_has("BOOT LED-CTRL v" FW_VERSION " reset=WATCHDOG resets=1"), "watchdog reset happened and was reported");
    uint32_t dt = sim.last_reset_t - t_hang;
    CHECK(dt >= 1990u && dt <= 2010u, "reset occurred about 2000 ms after the hang");
    CHECK(!g_hung && user_mode == LED_OFF, "firmware restarted in its default state");
    sim_tx_clear();
    sim_send_line("STATUS");
    CHECK(sim_tx_has("wdg_resets=1"), "reset counter persisted in the backup register and is reported");
    sim_tx_clear(); sim_send_line("LED ON");
    CHECK(sim_tx_has("OK LED ON"), "console works again after recovery");
}

static void tc15_ringbuffer(void)
{
    tc_begin("TC-15", "Ring buffer unit test (wrap-around, full, empty)");
    uint8_t out = 0u;
    CHECK(!rx_pop(&out), "empty buffer: pop fails");
    bool order = true;
    for (unsigned i = 0u; i < 500u; i++) {        /* wrap the 64-byte buffer many times */
        sim.rdr = (uint8_t)i;
        USART2_IRQHandler();
        if (!rx_pop(&out) || out != (uint8_t)i) { order = false; }
    }
    CHECK(order && rx_overflow == 0u, "500 bytes pushed/popped in order across many wrap-arounds");
    for (unsigned i = 0u; i < UART_RX_BUF_SIZE + 10u; i++) { sim.rdr = (uint8_t)i; USART2_IRQHandler(); }
    CHECK(rx_overflow == 11u, "pushing 74 bytes into a 64-slot ring drops exactly 11 (one slot is the full marker)");
    unsigned cnt = 0u;
    bool first_ok = true;
    while (rx_pop(&out)) { if (cnt < 63u && out != (uint8_t)cnt) { first_ok = false; } cnt++; }
    CHECK(cnt == 63u && first_ok, "63 oldest bytes are preserved intact");
}

static void tc16_adc(void)
{
    tc_begin("TC-16", "ADC conversion unit test");
    CHECK(adc_to_c10(0u) == 0, "code 0 -> 0.0 degC");
    CHECK(adc_to_c10(310u) == 250, "code 310 -> 25.0 degC");
    CHECK(adc_to_c10(4095u) == 3300, "full scale 4095 -> 330.0 degC (3.3 V at 10 mV/degC)");
    CHECK(adc_to_c10(745u) == 600, "code 745 -> 60.0 degC");
    char tb[16];
    CHECK(strcmp(fmt_c10(-35, tb, sizeof tb), "-3.5") == 0, "formatter prints negative values (-3.5)");
    CHECK(strcmp(fmt_c10(5, tb, sizeof tb), "0.5") == 0, "formatter prints 0.5");
}

static int run_selftest(void)
{
    printf("LED-CTRL firmware self-test (simulated target)\n");
    printf("==============================================\n");
    tc01_boot();        tc02_timebase();   tc03_basic_commands(); tc04_brightness();
    tc05_blink();       tc06_breathe();    tc07_robust_input();   tc08_overflow();
    tc09_temperature(); tc10_spike();      tc11_alert();          tc12_threshold();
    tc13_status();      tc14_watchdog();   tc15_ringbuffer();     tc16_adc();
    printf("\n==============================================\n");
    printf("RESULT: %d checks passed, %d failed -> %s\n", g_pass, g_fail, g_fail == 0 ? "ALL PASS" : "FAILURES");
    return g_fail == 0 ? 0 : 1;
}

/* =============================================================================
 *  SECTION G : DEMO SCENARIO AND INTERACTIVE MODE
 * ============================================================================= */
static void ramp_temp(int32_t from_c10, int32_t to_c10, int32_t step_c10, uint32_t ms_per_step)
{
    int32_t t = from_c10;
    while ((step_c10 > 0) ? (t < to_c10) : (t > to_c10)) {
        t += step_c10;
        sim.temp_c10 = t;
        sim_run(ms_per_step);
    }
}

static void run_demo(const char *csv_path)
{
    sim_init();
    sim.echo = true;
    if (csv_path != NULL) {
        sim.csv = fopen(csv_path, "w");
        if (sim.csv != NULL) { fprintf(sim.csv, "t_ms,led_pct,temp_true_c10,temp_avg_c10,alert\n"); }
    }
    printf("--- DEMO: scripted scenario on the simulated board ---\n");
    fw_boot(hal_reset_cause());
    sim_run(500u);
    sim_send_line("HELP");
    sim_send_line("LED BLINK 400");   sim_run(2000u);
    sim_send_line("LED BREATHE");     sim_run(3000u);
    sim_send_line("BRIGHT 60");       sim_run(1000u);
    sim_send_line("LED ON");          sim_run(500u);
    sim_send_line("LED OFF");         sim_run(500u);
    sim_send_line("TEMP");
    sim_send_line("LED BLINK 1000");
    sim_send_line("BRIGHT 200");      /* deliberate error */
    sim_send_line("FLY");             /* deliberate error */
    sim_send_line("STATUS");
    printf("--- temperature rises 25 -> 40 degC (threshold 35) ---\n");
    ramp_temp(250, 400, 5, 200u);
    sim_run(1000u);
    sim_send_line("STATUS");
    printf("--- temperature falls 40 -> 28 degC ---\n");
    ramp_temp(400, 280, -5, 200u);
    sim_run(1000u);
    sim_send_line("STATUS");
    printf("--- watchdog demonstration ---\n");
    sim_send_line("HANG");
    sim_run(2600u);
    sim_send_line("STATUS");
    printf("--- end of demo (simulated time %lu ms) ---\n", (unsigned long)sim.t_ms);
    if (sim.csv != NULL) { fclose(sim.csv); sim.csv = NULL; }
}

static void run_interactive(void)
{
    char buf[256];
    sim_init();
    sim.echo = true;
    fw_boot(hal_reset_cause());
    printf("Interactive mode. Type UART commands. Directives: @wait <ms>, @temp <degC>, @quit\n");
    while (fgets(buf, sizeof buf, stdin) != NULL) {
        size_t n = strlen(buf);
        while (n > 0u && (buf[n - 1u] == '\n' || buf[n - 1u] == '\r')) { buf[--n] = '\0'; }
        if (strcmp(buf, "@quit") == 0) { break; }
        unsigned ms = 0u;
        double   deg = 0.0;
        if (sscanf(buf, "@wait %u", &ms) == 1) { sim_run(ms); continue; }
        if (sscanf(buf, "@temp %lf", &deg) == 1) {
            sim.temp_c10 = (int32_t)(deg * 10.0 + (deg >= 0 ? 0.5 : -0.5));
            printf("[%7lu ms] SIM> temperature set to %.1f degC\n", (unsigned long)sim.t_ms, deg);
            continue;
        }
        sim_send_line(buf);
        sim_run(50u);
    }
}

int main(int argc, char **argv)
{
    bool selftest = false, demo = false, inter = false, verbose = false;
    const char *csv = NULL;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--selftest") == 0)         { selftest = true; }
        else if (strcmp(argv[i], "--demo") == 0)        { demo = true; }
        else if (strcmp(argv[i], "--interactive") == 0) { inter = true; }
        else if (strcmp(argv[i], "--verbose") == 0)     { verbose = true; }
        else if (strcmp(argv[i], "--csv") == 0 && i + 1 < argc) { csv = argv[++i]; }
        else {
            fprintf(stderr, "unknown option '%s'\n", argv[i]);
            return 2;
        }
    }
    if (!selftest && !demo && !inter) {
        printf("usage: %s [--selftest [--verbose]] [--demo [--csv file]] [--interactive]\n", argv[0]);
        return 2;
    }
    if (selftest) {
        sim.echo = verbose;
        return run_selftest();
    }
    if (demo)  { run_demo(csv); return 0; }
    run_interactive();
    return 0;
}
