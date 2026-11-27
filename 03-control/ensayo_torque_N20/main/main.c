/*
 * ensayo_torque_N20 - banco de ensayo de torque para un N20 sobre la MOD_DRV.
 *
 * Un motor a la vez. PWM de 10 kHz con decaimiento lento (avance/freno): el
 * voltaje medio en bornes es ~ciclo x Vbus y la corriente es casi continua.
 * Todo corre en un lazo de 10 ms; los calculos los hace el panel.
 *
 * Comandos (una linea): HB | ID | SEL,m | CAL,milesimas | HX,0|1 | STOP | BRAKE
 *   RUN,dir,duty,ms            vacio
 *   PULSE,dir,duty,ms,imax     bloqueo (duty <= 650, ms <= 3000)
 *   LIFT,dir,duty,cuentas,imax levantar pesa; termina en freno
 *   HOLDI,dir,mA,segundos      corriente constante (duty <= 650)
 * duty en milesimas, corrientes en mA, dir = 1 o -1.
 * Telemetria 100 Hz: T,ms,motor,modo,duty,cuentas,mA,hx,hx_sec,nfault
 * Fin de ensayo: E,razon,ms,cuentas
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "current_sense.h"
#include "encoder.h"
#include "pin_map.h"

#define VERSION "1"
#define UART_PORT UART_NUM_0
#define BAUD 460800
#define TICK_MS 10
#define PERIOD SENSE_PWM_PERIOD_TICKS      /* 10 kHz a 20 MHz, como pata_MOD_DRV */

#define DUTY_MAX_BLOCKED 650               /* bloqueado: nunca 12 V */
#define PULSE_MAX_MS 3000
#define RUN_MAX_MS 15000
#define LIFT_MAX_COUNTS 40000
#define HOLD_MAX_S 900
#define IMAX_HARD_MA 600
#define HOLD_MAX_MA 450
#define RAMP_RUN_MS 300
#define RAMP_PULSE_MS 100
#define HOST_TIMEOUT_MS 600
#define OVERCURRENT_MS 50
#define SENSE_LOST_MS 100
#define STALL_GRACE_MS 600
#define STALL_WINDOW_MS 300
#define STALL_MIN_COUNTS 28
#define HOLD_SATURATED_MS 2000
#define HOLD_KI 6.7f                       /* ciclo/(A s) */

/* HX711 opcional en el zocalo J4 SIN el modulo CAN: SCK = IO21, DOUT = IO22. */
#define HX_SCK CAN_TX_GPIO
#define HX_DOUT CAN_RX_GPIO

typedef enum { M_IDLE, M_BRAKE, M_RUN, M_PULSE, M_LIFT, M_HOLDI } run_mode_t;
static const char *k_mode[] = {"IDLE", "BRAKE", "RUN", "PULSE", "LIFT", "HOLDI"};

static const int k_in1[] = MOTOR_PIN_IN1, k_in2[] = MOTOR_PIN_IN2, k_nf[] = MOTOR_PIN_NFAULT;
static mcpwm_gen_handle_t gen[MOTOR_COUNT][2];
static mcpwm_cmpr_handle_t cmp[MOTOR_COUNT][2];

static struct {
    run_mode_t mode;
    int motor, dir, duty_target, duty, imax_ma, hold_ma, limit_ms;
    int64_t target_counts, start_counts, start_us, stall_us, stall_counts;
    int over_ms, lost_ms, sat_ms, nfault_ms;
    float hold_duty;
} st = {.dir = 1};
static float cs_scale = 1.0f;
static int64_t host_us;
static bool hx_on;
static int32_t hx_value;
static uint32_t hx_seq;

static void say(const char *fmt, ...)
{
    char b[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    if (n > 0) uart_write_bytes(UART_PORT, b, n < (int)sizeof b ? n : (int)sizeof b - 1);
}

/* ---------------------------------------------------------------- puente H */
static void level(int m, int input, int forced)
{
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(gen[m][input], forced, true));
}

static void coast(int m) { level(m, 0, 0); level(m, 1, 0); }

/* Avance: la entrada activa fija en alto y la otra en alto (1 - D): avance/freno. */
static void drive(int m, int dir, int duty)
{
    int on = dir > 0 ? 0 : 1, sw = 1 - on;
    if (duty <= 0) { coast(m); return; }
    if (duty >= 1000) level(m, sw, 0);
    else {
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp[m][sw], (uint32_t)(PERIOD * (1000 - duty) / 1000)));
        level(m, sw, -1);
    }
    level(m, on, 1);
}

static void brake(int m) { level(m, 0, 1); level(m, 1, 1); }

static void pwm_init(void)
{
    mcpwm_timer_handle_t timer;
    mcpwm_timer_config_t tc = {.group_id = 0, .clk_src = MCPWM_TIMER_CLK_SRC_PLL160M,
        .resolution_hz = SENSE_PWM_TIMER_HZ, .period_ticks = PERIOD, .count_mode = MCPWM_TIMER_COUNT_MODE_UP};
    ESP_ERROR_CHECK(mcpwm_new_timer(&tc, &timer));
    for (int m = 0; m < MOTOR_COUNT; m++) {
        mcpwm_oper_handle_t op;
        mcpwm_operator_config_t oc = {.group_id = 0};
        ESP_ERROR_CHECK(mcpwm_new_operator(&oc, &op));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(op, timer));
        for (int i = 0; i < 2; i++) {
            mcpwm_comparator_config_t cc = {.flags.update_cmp_on_tez = true};
            ESP_ERROR_CHECK(mcpwm_new_comparator(op, &cc, &cmp[m][i]));
            mcpwm_generator_config_t gc = {.gen_gpio_num = i ? k_in2[m] : k_in1[m]};
            ESP_ERROR_CHECK(mcpwm_new_generator(op, &gc, &gen[m][i]));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp[m][i], 0));
            ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen[m][i],
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
            ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen[m][i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmp[m][i], MCPWM_GEN_ACTION_LOW)));
        }
        coast(m);
    }
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

/* ------------------------------------------------------------------- HX711 */
static portMUX_TYPE hx_mux = portMUX_INITIALIZER_UNLOCKED;

static void hx_enable(bool on)
{
    if (on) {
        gpio_config_t o = {.pin_bit_mask = 1ULL << HX_SCK, .mode = GPIO_MODE_OUTPUT};
        gpio_config_t i = {.pin_bit_mask = 1ULL << HX_DOUT, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE};
        ESP_ERROR_CHECK(gpio_set_level(HX_SCK, 0));
        ESP_ERROR_CHECK(gpio_config(&o));
        ESP_ERROR_CHECK(gpio_config(&i));
    } else if (hx_on) {
        ESP_ERROR_CHECK(gpio_reset_pin(HX_SCK));
    }
    hx_on = on;
    hx_seq = 0;
}

static void hx_poll(void)
{
    if (!hx_on || gpio_get_level(HX_DOUT)) return;
    uint32_t v = 0;
    portENTER_CRITICAL(&hx_mux);            /* SCK alto > 60 us apagaria el HX711 */
    for (int i = 0; i < 25; i++) {          /* 24 bits + 1: canal A, ganancia 128 */
        gpio_set_level(HX_SCK, 1);
        esp_rom_delay_us(1);
        if (i < 24) v = (v << 1) | (uint32_t)gpio_get_level(HX_DOUT);
        gpio_set_level(HX_SCK, 0);
        esp_rom_delay_us(1);
    }
    portEXIT_CRITICAL(&hx_mux);
    hx_value = (int32_t)(v << 8) >> 8;
    hx_seq++;
}

/* ----------------------------------------------------------------- ensayos */
static bool active(void) { return st.mode >= M_RUN; }
static int64_t counts_now(void) { return encoder_sample(st.motor).count_x4; }

static void finish(const char *reason, bool hold)
{
    bool was = active();
    if (hold) brake(st.motor); else coast(st.motor);
    st.mode = hold ? M_BRAKE : M_IDLE;
    st.duty = 0;
    if (was) say("E,%s,%lld,%lld\n", reason, (long long)((esp_timer_get_time() - st.start_us) / 1000),
                 (long long)(counts_now() - st.start_counts));
}

static void start(run_mode_t mode, int dir, int duty, int imax, int limit_ms)
{
    int motor = st.motor;
    coast(motor);
    memset(&st, 0, sizeof st);
    st.motor = motor; st.dir = dir; st.duty_target = duty;
    st.imax_ma = imax; st.limit_ms = limit_ms;
    st.start_us = st.stall_us = esp_timer_get_time();
    st.start_counts = st.stall_counts = counts_now();
    st.mode = mode;
}

static void step(float amps, bool amps_ok, int64_t counts)
{
    int64_t now = esp_timer_get_time();
    int ms = (int)((now - st.start_us) / 1000);
    bool nf = gpio_get_level(k_nf[st.motor]) == 0;
    st.nfault_ms = nf ? st.nfault_ms + TICK_MS : 0;
    if (!active()) return;
    if (st.nfault_ms >= 2 * TICK_MS) { finish("NFAULT", false); return; }
    if (now - host_us > HOST_TIMEOUT_MS * 1000LL) { finish("SIN_PANEL", false); return; }
    st.lost_ms = amps_ok ? 0 : st.lost_ms + TICK_MS;
    if (st.lost_ms >= SENSE_LOST_MS) { finish("SIN_CORRIENTE", false); return; }
    st.over_ms = amps_ok && amps * 1000.0f > st.imax_ma ? st.over_ms + TICK_MS : 0;
    if (st.over_ms >= OVERCURRENT_MS) { finish("SOBRECORRIENTE", false); return; }
    if (st.mode == M_LIFT && llabs(counts - st.start_counts) >= st.target_counts) { finish("OK", true); return; }
    if (ms >= st.limit_ms) { finish(st.mode == M_LIFT ? "TIEMPO" : "OK", st.mode == M_LIFT); return; }
    if ((st.mode == M_RUN || st.mode == M_LIFT) && ms >= STALL_GRACE_MS &&
        now - st.stall_us >= STALL_WINDOW_MS * 1000LL) {
        if (llabs(counts - st.stall_counts) < STALL_MIN_COUNTS) { finish("BLOQUEADO", false); return; }
        st.stall_us = now;
        st.stall_counts = counts;
    }
    if (st.mode == M_HOLDI) {
        if (amps_ok) st.hold_duty += HOLD_KI * (st.hold_ma / 1000.0f - amps) * (TICK_MS / 1000.0f);
        float max = DUTY_MAX_BLOCKED / 1000.0f;
        if (st.hold_duty < 0.0f) st.hold_duty = 0.0f;
        st.sat_ms = st.hold_duty >= max ? st.sat_ms + TICK_MS : 0;
        if (st.hold_duty > max) st.hold_duty = max;
        if (st.sat_ms >= HOLD_SATURATED_MS) { finish("LIMITE_VOLTAJE", false); return; }
        st.duty = (int)(st.hold_duty * 1000.0f);
    } else {
        int ramp = st.mode == M_PULSE ? RAMP_PULSE_MS : RAMP_RUN_MS;
        st.duty = ms >= ramp ? st.duty_target : st.duty_target * (ms + TICK_MS) / (ramp + TICK_MS);
    }
    drive(st.motor, st.dir, st.duty);
}

/* ---------------------------------------------------------------- comandos */
static bool dir_ok(int d) { return d == 1 || d == -1; }

static void command(char *s)
{
    int a, b, c, d;
    char x;
    if (!strcmp(s, "HB")) { host_us = esp_timer_get_time(); return; }
    if (!strcmp(s, "STOP")) { finish("STOP", false); say("ACK,STOP\n"); }
    else if (!strcmp(s, "BRAKE")) { finish("STOP", true); say("ACK,BRAKE\n"); }
    else if (!strcmp(s, "ID")) say("ID,%s,%s,motor=%d,cpr=%u,cal=%d,hx=%d\n", BOARD_NAME, VERSION, st.motor + 1,
                                   (unsigned)ENCODER_COUNTS_PER_OUTPUT_REV, (int)lroundf(cs_scale * 1000), hx_on);
    else if (active()) { say("ERR,OCUPADO\n"); }
    else if (sscanf(s, "SEL,%d%c", &a, &x) == 1 && a >= 1 && a <= MOTOR_COUNT) {
        coast(st.motor); st.mode = M_IDLE; st.motor = a - 1; say("ACK,SEL,%d\n", a);
    } else if (sscanf(s, "CAL,%d%c", &a, &x) == 1 && a >= 500 && a <= 2000) {
        cs_scale = a / 1000.0f; say("ACK,CAL,%d\n", a);
    } else if (sscanf(s, "HX,%d%c", &a, &x) == 1 && (a == 0 || a == 1)) {
        hx_enable(a); say("ACK,HX,%d\n", a);
    } else if (sscanf(s, "RUN,%d,%d,%d%c", &a, &b, &c, &x) == 3 && dir_ok(a) && b > 0 && b <= 1000 &&
               c > 0 && c <= RUN_MAX_MS) {
        start(M_RUN, a, b, IMAX_HARD_MA, c); say("ACK,RUN\n");
    } else if (sscanf(s, "PULSE,%d,%d,%d,%d%c", &a, &b, &c, &d, &x) == 4 && dir_ok(a) && b > 0 &&
               b <= DUTY_MAX_BLOCKED && c > 0 && c <= PULSE_MAX_MS && d >= 50 && d <= IMAX_HARD_MA) {
        start(M_PULSE, a, b, d, c); say("ACK,PULSE\n");
    } else if (sscanf(s, "LIFT,%d,%d,%d,%d%c", &a, &b, &c, &d, &x) == 4 && dir_ok(a) && b > 0 && b <= 1000 &&
               c > 0 && c <= LIFT_MAX_COUNTS && d >= 50 && d <= IMAX_HARD_MA) {
        start(M_LIFT, a, b, d, RUN_MAX_MS); st.target_counts = c; say("ACK,LIFT\n");
    } else if (sscanf(s, "HOLDI,%d,%d,%d%c", &a, &b, &c, &x) == 3 && dir_ok(a) && b >= 50 && b <= HOLD_MAX_MA &&
               c > 0 && c <= HOLD_MAX_S) {
        start(M_HOLDI, a, 0, IMAX_HARD_MA, c * 1000); st.hold_ma = b; say("ACK,HOLDI\n");
    } else { say("ERR,COMANDO\n"); return; }
    host_us = esp_timer_get_time();
}

static void serial_poll(void)
{
    static char line[96];
    static size_t len;
    static bool skip;
    uint8_t buf[128];
    int n = uart_read_bytes(UART_PORT, buf, sizeof buf, 0);
    for (int i = 0; i < n; i++) {
        char ch = (char)buf[i];
        if (ch == '\r') continue;
        if (ch == '\n') {
            line[len] = 0;
            if (skip) say("ERR,LINEA\n");
            else if (len) command(line);
            len = 0; skip = false;
        } else if (len < sizeof line - 1) line[len++] = ch;
        else skip = true;
    }
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);
    pwm_init();                              /* primero: los puentes quedan sueltos */
    uart_config_t uc = {.baud_rate = BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 1024, 4096, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uc));
    for (int m = 0; m < MOTOR_COUNT; m++) {
        gpio_config_t g = {.pin_bit_mask = 1ULL << k_nf[m], .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE};
        ESP_ERROR_CHECK(gpio_config(&g));
    }
    encoder_init();
    current_sense_init();
    say("BOOT,%s,%s\n", BOARD_NAME, VERSION);

    TickType_t wake = xTaskGetTickCount();
    while (true) {
        serial_poll();
        hx_poll();
        current_sample_t cs = current_sense_read(st.motor);
        bool ok = current_sample_usable(cs.status) && isfinite(cs.current_mean_a);
        float amps = ok ? cs.current_mean_a * cs_scale : 0.0f;
        int64_t counts = counts_now();
        step(amps, ok, counts);
        char hx[16] = "x";
        if (hx_on && hx_seq) snprintf(hx, sizeof hx, "%ld", (long)hx_value);
        say("T,%lld,%d,%s,%d,%lld,%.1f,%s,%lu,%d\n", (long long)(esp_timer_get_time() / 1000), st.motor + 1,
            k_mode[st.mode], st.dir * st.duty, (long long)counts, ok ? (double)(amps * 1000.0f) : -1.0, hx,
            (unsigned long)hx_seq, gpio_get_level(k_nf[st.motor]) == 0);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(TICK_MS));
    }
}
