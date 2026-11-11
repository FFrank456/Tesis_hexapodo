#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include "sdkconfig.h"
#include "pin_map.h"
#include "encoder.h"
#include "pwm_sequence.h"
#include "current_sense.h"
#include "current_pi.h"
#include "speed_pi.h"
#include "position_pi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "hal/adc_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

/*
 * pata_MOD_DRV - cascada corriente/velocidad/posicion para los 3 motores de
 * una pata (Coxa, Femur, Tibia) funcionando a la vez en la placa MOD_DRV
 * (ESP32-WROOM-32E + 3x DRV8876). Mismo codigo que pata_banco1; la placa se
 * define en pin_map.h (pines, V/A del IPROPI y nFAULT por driver).
 *
 * Basado en ControlCurrentN20 (un motor). Cada motor conserva el mismo lazo:
 * - MCPWM grupo 0: un timer de 10 kHz y un operador por motor (IN1/IN2).
 *   El evento "timer empty" despierta la tarea de control cada 10 periodos.
 * - ADC1 DMA continuo: CH0/CH3/CH6 a 64 kS/s por canal (192 kS/s total, limite
 *   del SAR del ESP32), media de 64 muestras por canal cada 1 ms (current_sense.c).
 * - PCNT: una unidad x4 por encoder (encoder.c).
 * - OPEN aplica PWM; PID regula CS; SPEED: PI velocidad -> PI CS;
 *   POSITION: PI posicion -> PI velocidad -> PI CS. Igual que el original.
 *
 * Pines en pin_map.h. Protocolo serial en README.md:
 * - "M1,", "M2,", "M3," (o COXA/FEMUR/TIBIA, ALL) antes de un comando lo
 *   dirige a ese motor; la respuesta lleva el mismo prefijo.
 * - Sin prefijo, el comando va al motor seleccionado con SEL (compatibilidad
 *   con el dashboard Processing de un motor). STOP sin prefijo para TODOS.
 * - TEL,LEGACY: DATA/SPEED/POSITION v6 del motor seleccionado (Processing).
 *   TEL,MULTI: linea compacta "T" con los 3 motores a 100 Hz (dashboard web).
 */

#define PWM_FREQUENCY_HZ                    10000U
#define PWM_PERIOD_TICKS                    SENSE_PWM_PERIOD_TICKS
#define PWM_MAX_DUTY                        PWM_PERIOD_TICKS
#define MCPWM_TIMER_RESOLUTION_HZ           SENSE_PWM_TIMER_HZ
#define MCPWM_GROUP_ID                      0

#define SNAPSHOT_RATE_HZ                  1000U
#define PROCESSING_RATE_HZ                  100U
#define PWM_CYCLES_PER_ADC_SAMPLE           (PWM_FREQUENCY_HZ / SNAPSHOT_RATE_HZ)
#define PROCESSING_PERIOD_MS                (1000U / PROCESSING_RATE_HZ)
#define COMMAND_BUFFER_SIZE                 512

#define PID_OUTPUT_MIN_PERCENT              0.0f
#define PID_OUTPUT_MAX_PERCENT              100.0f
#define CURRENT_REF_INITIAL_A               0.0f
/* Below this hold current the gravity hold falls back to the brake. */
#define GRAVITY_HOLD_MIN_A                  0.01f
/* Cascade: brake (instead of coasting) when the axis runs this much faster
 * than the speed reference and the current loop has nothing to drive, e.g. a
 * leg lowering under its own weight. */
#define OVERSPEED_BRAKE_RAD_S               0.5f

_Static_assert(PWM_FREQUENCY_HZ % SNAPSHOT_RATE_HZ == 0,
               "SNAPSHOT_RATE_HZ must divide PWM_FREQUENCY_HZ");
_Static_assert(MCPWM_TIMER_RESOLUTION_HZ == PWM_FREQUENCY_HZ * PWM_PERIOD_TICKS,
               "PWM period must agree with the realizable timer resolution");
_Static_assert(MOTOR_COUNT <= 3, "MCPWM group 0 has three operators");
#ifndef BOARD_NAME
#define BOARD_NAME "pata"
#endif
#ifndef MOTOR_PIN_NFAULT
#define MOTOR_PIN_NFAULT {-1, -1, -1}
#endif
#ifndef DRV_NFAULT_FILTER_MS
#define DRV_NFAULT_FILTER_MS 3
#endif

typedef enum {
    MOTOR_STOP = 0,   /* coast: IN1 = IN2 = 0 */
    MOTOR_CW,
    MOTOR_CCW,
    MOTOR_BRAKE,      /* brake (low-side slow decay): IN1 = IN2 = 1 */
} motor_direction_t;

typedef enum {
    CONTROL_MODE_OPEN = 0,
    CONTROL_MODE_PID,
    CONTROL_MODE_SPEED,
    CONTROL_MODE_POSITION,
} control_mode_t;

typedef enum {
    TELEMETRY_LEGACY = 0,
    TELEMETRY_MULTI = 1,
} telemetry_mode_t;

typedef struct {
    uint32_t time_ms;
    int64_t sample_time_us;
    int64_t encoder_count;
    encoder_sample_t encoder;
    uint32_t sequence_id;
    int sequence_index;
    int64_t sequence_start_us;
    int64_t sequence_stage_start_us;
    uint32_t adc_sample_count;
    int adc_raw;
    float adc_voltage;
    bool adc_calibrated;
    float current_raw_a;
    float adc_raw_mean;
    float adc_voltage_mean;
    float current_mean_a;
    uint32_t dma_block_samples;
    float current_ref_a;
    float current_error_a;
    float control_output_percent;
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float pid_integrator;
    float pid_output_min_percent;
    float pid_output_max_percent;
    control_mode_t control_mode;
    motor_direction_t direction;
    uint32_t duty_cw;
    uint32_t duty_ccw;
    float duty_percent;
    current_status_t current_status;
    uint32_t adc_read_us;
    int32_t sample_phase_us;
    uint32_t current_valid_count;
    uint32_t sample_attempt_count;
    uint32_t sample_missed_count;
    float pwm_frequency_observed_hz;
    float acquisition_rate_observed_hz;
    float adc_rate_observed_hz;
    uint32_t dma_overflows;
    uint32_t rejected_window;
    uint32_t rejected_quality;
    speed_pi_t speed;
    position_pi_t position;
    int control_direction_sign;
    bool position_reference_set;
    bool sequence_active;
    bool nfault_active;
    bool hold_brake;
    bool gravity_holding;
    float friction_pwm;
    float gravity_a;
    float gravity_offset_rad;
} telemetry_t;

typedef struct {
    int index;
    const char *name;
    int gpio_cw;   /* IN1 */
    int gpio_ccw;  /* IN2 */
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t comparator_cw;
    mcpwm_cmpr_handle_t comparator_ccw;
    mcpwm_gen_handle_t generator_cw;
    mcpwm_gen_handle_t generator_ccw;

    pwm_sequence_t sequence;
    int64_t output_changed_us;
    uint32_t previous_dma_overflows;
    current_status_t pending_current_fault;
    bool pending_drv_fault;
    int nfault_pin;           /* -1: not wired */
    uint32_t nfault_low_ms;   /* consecutive 1 ms samples with nFAULT low */
    bool pending_speed_fault;
    bool pending_position_fault;
    bool position_reference_set;
    uint32_t position_reference_id;
    position_pi_t position;
    speed_pi_t speed;
    pid_controller_t pid;
    motor_direction_t direction;
    uint32_t duty_cw;
    uint32_t duty_ccw;
    float duty_percent;
    control_mode_t control_mode;
    motor_direction_t control_direction;
    /* Position hold and compensation (pata_MOD_DRV). */
    bool hold_brake;          /* brake instead of coast when POSITION is not tracking */
    float friction_pwm;       /* % PWM added while the cascade drives (SPEED/POSITION) */
    float gravity_a;          /* CS-equivalent A needed to hold the link at cos()=1 */
    float gravity_offset_rad; /* angle added to the position before cos() */
    bool gravity_holding;     /* POSITION at rest, current = gravity hold only (display) */
    float speed_current_a;    /* speed-loop output, magnitude in the motion direction */
    motor_direction_t drive_direction; /* direction the current loop actually drives */
    telemetry_t telemetry;
} motor_t;

static const char *const k_motor_names[MOTOR_COUNT] = MOTOR_NAMES;
static const int k_pin_in1[MOTOR_COUNT] = MOTOR_PIN_IN1;
static const int k_pin_in2[MOTOR_COUNT] = MOTOR_PIN_IN2;
static const int k_pin_cs[MOTOR_COUNT] = MOTOR_PIN_CS;
static const int k_adc_channel[MOTOR_COUNT] = MOTOR_ADC_CHANNELS;
static const int k_pin_enc_a[MOTOR_COUNT] = MOTOR_PIN_ENC_A;
static const int k_pin_enc_b[MOTOR_COUNT] = MOTOR_PIN_ENC_B;
static const int k_pin_nfault[MOTOR_COUNT] = MOTOR_PIN_NFAULT;

static SemaphoreHandle_t g_motor_lock;
static mcpwm_timer_handle_t g_pwm_timer;
static TaskHandle_t g_adc_task_handle = NULL;
static portMUX_TYPE g_state_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE g_clock_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t g_pwm_cycle_count;
static uint32_t g_sequence_id = 0;
static motor_t g_motors[MOTOR_COUNT];
static int g_selected = DEFAULT_SELECTED_MOTOR; /* pin_map.h */
static telemetry_mode_t g_telemetry_mode = TELEMETRY_LEGACY;
static char g_reply_prefix[16] = "";

static const char *motor_direction_to_text(motor_direction_t direction)
{
    switch (direction) {
    case MOTOR_CW:
        return "CW";
    case MOTOR_CCW:
        return "CCW";
    case MOTOR_BRAKE:
        return "BRAKE";
    case MOTOR_STOP:
    default:
        return "STOP";
    }
}

static const char *control_mode_to_text(control_mode_t mode)
{
    return mode == CONTROL_MODE_POSITION ? "POSITION" : mode == CONTROL_MODE_SPEED ? "SPEED" : mode == CONTROL_MODE_PID ? "PID" : "OPEN";
}

/* Every command reply carries the prefix of the motor it addressed ("M2,"),
 * or none for unprefixed commands, so the legacy dashboard still sees "ACK,". */
static void reply(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    fputs(g_reply_prefix, stdout);
    vprintf(format, args);
    va_end(args);
}

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static uint32_t clamp_duty(uint32_t duty)
{
    return duty > PWM_MAX_DUTY ? PWM_MAX_DUTY : duty;
}

static uint32_t percent_to_duty(float percent)
{
    percent = clamp_float(percent, 0.0f, 100.0f);
    return (uint32_t)((percent * (float)PWM_MAX_DUTY / 100.0f) + 0.5f);
}

static float duty_to_percent(uint32_t duty)
{
    duty = clamp_duty(duty);
    return ((float)duty * 100.0f) / (float)PWM_MAX_DUTY;
}

static void serial_command_init(void)
{
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
}

static void mcpwm_set_generator_for_duty(mcpwm_gen_handle_t generator,
                                         mcpwm_cmpr_handle_t comparator,
                                         uint32_t duty)
{
    duty = clamp_duty(duty);

    if (duty == PWM_MAX_DUTY) {
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generator, 1, true));
        return;
    }

    if (duty == 0) {
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generator, 0, true));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 0));
        return;
    }

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, duty));
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generator, -1, true));
}

static bool motor_driving(const motor_t *m)
{
    return m->direction == MOTOR_CW || m->direction == MOTOR_CCW;
}

static void motor_set_output(motor_t *m, motor_direction_t direction, uint32_t duty)
{
    uint32_t duty_cw = 0;
    uint32_t duty_ccw = 0;

    if (direction == MOTOR_BRAKE) {
        /* Both inputs high: DRV8876 PWM mode brakes through the low-side FETs.
         * The motor's back-EMF stops it fast and resists being back-driven. */
        if (m->direction == MOTOR_BRAKE) return;
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->generator_cw, 0, true));
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->generator_ccw, 0, true));
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->generator_cw, 1, true));
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->generator_ccw, 1, true));
        m->output_changed_us = esp_timer_get_time();
        portENTER_CRITICAL(&g_state_lock);
        m->direction = MOTOR_BRAKE;
        m->duty_cw = 0;
        m->duty_ccw = 0;
        m->duty_percent = 0.0f;
        portEXIT_CRITICAL(&g_state_lock);
        return;
    }

    duty = clamp_duty(duty);
    if (duty == 0) direction = MOTOR_STOP;
    if (direction == MOTOR_CW) {
        duty_cw = duty;
    } else if (direction == MOTOR_CCW) {
        duty_ccw = duty;
    } else {
        direction = MOTOR_STOP;
    }

    if (duty_cw == m->duty_cw && duty_ccw == m->duty_ccw && direction == m->direction) return;
    /* Break before reversing. Same-direction updates use the shadow comparator
     * without inserting a low glitch on every PID iteration. */
    if (direction != m->direction) {
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->generator_cw, 0, true));
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->generator_ccw, 0, true));
    }
    if (duty_cw != m->duty_cw)
        mcpwm_set_generator_for_duty(m->generator_cw, m->comparator_cw, duty_cw);
    if (duty_ccw != m->duty_ccw)
        mcpwm_set_generator_for_duty(m->generator_ccw, m->comparator_ccw, duty_ccw);
    m->output_changed_us = esp_timer_get_time();

    portENTER_CRITICAL(&g_state_lock);
    m->direction = direction;
    m->duty_cw = duty_cw;
    m->duty_ccw = duty_ccw;
    m->duty_percent = duty_to_percent(duty_cw) - duty_to_percent(duty_ccw);
    portEXIT_CRITICAL(&g_state_lock);
}

static void motor_set_output_percent(motor_t *m, motor_direction_t direction, float percent)
{
    motor_set_output(m, direction, percent_to_duty(percent));
}

static void control_set_mode(motor_t *m, control_mode_t mode)
{
    portENTER_CRITICAL(&g_state_lock);
    m->sequence.active = false;
    m->sequence.index = -1;
    m->control_mode = mode;
    m->gravity_holding = false;
    m->speed_current_a = 0;
    position_pi_reset(&m->position);
    speed_pi_reset(&m->speed);
    if (mode == CONTROL_MODE_OPEN) {
        pid_reset(&m->pid);
    }
    portEXIT_CRITICAL(&g_state_lock);
}

static void control_stop(motor_t *m)
{
    control_set_mode(m, CONTROL_MODE_OPEN);
    motor_set_output(m, MOTOR_STOP, 0);
}

static bool motor_is_stopped(const motor_t *m)
{
    return !m->sequence.active && m->control_mode == CONTROL_MODE_OPEN && !motor_driving(m);
}

/* Caller owns g_motor_lock. Same-direction manual output is tracked on entry.
 * Opposite-direction activation requires STOP first. Zero reference always coasts. */
static bool control_start_pid(motor_t *m, motor_direction_t direction, float reference_a)
{
    if ((direction != MOTOR_CW && direction != MOTOR_CCW) || !pi_reference_valid(reference_a)) {
        reply("ERR,PI,expected_CW_or_CCW_and_reference_0_to_1A_eq\n");
        return false;
    }
    if (motor_driving(m) && m->direction != direction) {
        reply("ERR,PI,STOP_before_reversing\n");
        return false;
    }
    current_sample_t current = current_sense_read(m->index);
    if (!current_sample_usable(current.status) || current.dma_block_samples < SENSE_MIN_BLOCK_SAMPLES ||
        !isfinite(current.current_mean_a)) {
        control_stop(m);
        reply("ERR,PI,valid_DMA_feedback_required\n");
        return false;
    }
    float tracking = fabsf(m->duty_percent);
    bool was_running = motor_driving(m);
    portENTER_CRITICAL(&g_state_lock);
    m->sequence.active = false;
    m->sequence.index = -1;
    m->control_mode = CONTROL_MODE_PID;
    position_pi_reset(&m->position);
    speed_pi_reset(&m->speed);
    m->control_direction = direction;
    m->drive_direction = direction;
    m->speed_current_a = 0;
    pid_reset(&m->pid);
    m->pid.current_ref_a = reference_a;
    m->pid.last_sample_us = current.sample_time_us;
    m->pid.error_a = reference_a-current.current_mean_a;
    if (was_running && reference_a > 0) {
        m->pid.output_percent = clamp_float(tracking, m->pid.output_min_percent, m->pid.output_max_percent);
        m->pid.integrator = m->pid.output_percent-m->pid.kp*m->pid.error_a;
    }
    portEXIT_CRITICAL(&g_state_lock);
    if (reference_a == 0) motor_set_output(m, MOTOR_STOP, 0);
    return true;
}

/* Enter only after both feedback paths and a coast-compatible direction check. */
static bool control_start_speed(motor_t *m, motor_direction_t direction)
{
    encoder_sample_t enc=encoder_sample(m->index);
    if (!enc.speed_valid || !isfinite(enc.output_speed_rad_s)) {
        control_stop(m);
        reply("ERR,SPEED,valid_encoder_required\n");
        return false;
    }
    float sign=direction==MOTOR_CCW ? -1.0f : 1.0f;
    if (sign*enc.output_speed_rad_s < -SPEED_REVERSAL_MAX_RAD_S) {
        reply("ERR,SPEED,wait_for_coast_before_reversing\n");
        return false;
    }
    if (!control_start_pid(m, direction, 0)) return false;
    portENTER_CRITICAL(&g_state_lock);
    m->control_mode=CONTROL_MODE_SPEED;
    speed_pi_reset(&m->speed);
    m->speed.last_update_us=enc.time_us;
    portEXIT_CRITICAL(&g_state_lock);
    return true;
}

/* Position starts only from a stopped, explicitly targeted axis. */
static bool control_start_position(motor_t *m)
{
    encoder_sample_t enc=encoder_sample(m->index);
    if(m->control_mode!=CONTROL_MODE_OPEN || motor_driving(m)) {
        reply("ERR,POSITION,STOP_before_activation\n");return false;
    }
    if(!m->position_reference_set || enc.reference_id!=m->position_reference_id) {
        reply("ERR,POSITION,set_PREF_after_encoder_zero\n");return false;
    }
    if(!enc.speed_valid || !isfinite(enc.output_position_rad) || !isfinite(enc.output_speed_rad_s) ||
       fabs(enc.output_speed_rad_s)>POSITION_STOP_RAD_S) {
        reply("ERR,POSITION,valid_stopped_encoder_required\n");return false;
    }
    motor_direction_t dir=m->position.reference_rad>=enc.output_position_rad ? MOTOR_CW : MOTOR_CCW;
    if(!control_start_pid(m, dir, 0))return false;
    portENTER_CRITICAL(&g_state_lock);
    m->control_mode=CONTROL_MODE_POSITION;
    m->position.direction=dir==MOTOR_CW ? 1 : -1;
    position_pi_reset(&m->position);m->position.last_update_us=enc.time_us;
    m->speed.reference_rad_s=0;m->speed.last_update_us=enc.time_us;
    portEXIT_CRITICAL(&g_state_lock);
    return true;
}

/* Fires at every PWM period start (counter == 0) of the shared timer. */
static bool pwm_timer_empty_callback(mcpwm_timer_handle_t timer,
                                     const mcpwm_timer_event_data_t *edata,
                                     void *user_ctx)
{
    static uint32_t pwm_cycle_count = 0;
    BaseType_t high_task_woken = pdFALSE;

    (void)timer;
    (void)edata;
    (void)user_ctx;

    pwm_cycle_count++;
    portENTER_CRITICAL_ISR(&g_clock_lock);
    g_pwm_cycle_count++;
    portEXIT_CRITICAL_ISR(&g_clock_lock);
    if (pwm_cycle_count >= PWM_CYCLES_PER_ADC_SAMPLE) {
        pwm_cycle_count = 0;
        if (g_adc_task_handle != NULL) {
            vTaskNotifyGiveFromISR(g_adc_task_handle, &high_task_woken);
        }
    }

    return high_task_woken == pdTRUE;
}

static void mcpwm_init_motor(motor_t *m)
{
    mcpwm_operator_config_t operator_config = {0};
    operator_config.group_id = MCPWM_GROUP_ID;
    operator_config.flags.update_gen_action_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &m->oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(m->oper, g_pwm_timer));

    mcpwm_comparator_config_t comparator_config = {0};
    comparator_config.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(m->oper, &comparator_config, &m->comparator_cw));
    ESP_ERROR_CHECK(mcpwm_new_comparator(m->oper, &comparator_config, &m->comparator_ccw));

    mcpwm_generator_config_t generator_cw_config = {0};
    generator_cw_config.gen_gpio_num = m->gpio_cw;
    ESP_ERROR_CHECK(mcpwm_new_generator(m->oper, &generator_cw_config, &m->generator_cw));

    mcpwm_generator_config_t generator_ccw_config = {0};
    generator_ccw_config.gen_gpio_num = m->gpio_ccw;
    ESP_ERROR_CHECK(mcpwm_new_generator(m->oper, &generator_ccw_config, &m->generator_ccw));
    ESP_ERROR_CHECK(gpio_input_enable(m->gpio_cw));
    ESP_ERROR_CHECK(gpio_input_enable(m->gpio_ccw));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m->comparator_cw, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m->comparator_ccw, 0));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        m->generator_cw,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        m->generator_cw,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       m->comparator_cw,
                                       MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        m->generator_ccw,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        m->generator_ccw,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       m->comparator_ccw,
                                       MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->generator_cw, 0, true));
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->generator_ccw, 0, true));
}

static void mcpwm_init(void)
{
    mcpwm_timer_config_t timer_config = {0};
    timer_config.group_id = MCPWM_GROUP_ID;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_PLL160M;
    timer_config.resolution_hz = MCPWM_TIMER_RESOLUTION_HZ;
    timer_config.period_ticks = PWM_PERIOD_TICKS;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &g_pwm_timer));

    for (int i = 0; i < MOTOR_COUNT; ++i) mcpwm_init_motor(&g_motors[i]);

    mcpwm_timer_event_callbacks_t timer_callbacks = {0};
    timer_callbacks.on_empty = pwm_timer_empty_callback;
    ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(g_pwm_timer, &timer_callbacks, NULL));
    ESP_ERROR_CHECK(mcpwm_timer_enable(g_pwm_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(g_pwm_timer, MCPWM_TIMER_START_NO_STOP));
}

typedef struct {
    uint32_t attempt_count;
    uint32_t missed_count;
    float observed_pwm_hz;
    float observed_attempt_hz;
    float observed_adc_hz;
} control_rates_t;

/* Speed-loop output plus the gravity feed-forward, projected on the active
 * direction (the current loop only drives a magnitude in control_direction). */
/* Signed cascade current: speed-loop output (a magnitude in the motion
 * direction) plus the signed gravity term GRAV*cos(position+offset).
 * Returns the magnitude for the current loop and the direction to drive it.
 * Lowering a weighted link this way commands LESS lifting current instead of
 * coasting, so the speed loop keeps control while gravity does the work. */
static float cascade_current(const motor_t *m, float speed_part_a, double position_rad,
                             motor_direction_t *drive, float *gravity_term_a)
{
    float limit = m->speed.current_limit_a;
    float hold = (m->gravity_a != 0.0f && isfinite(position_rad)) ?
        m->gravity_a * cosf((float)position_rad + m->gravity_offset_rad) : 0.0f;
    if (gravity_term_a) *gravity_term_a = hold;
    if (hold == 0.0f) {
        *drive = m->control_direction;
        return clamp_float(speed_part_a, 0.0f, limit);
    }
    float net = (m->control_direction == MOTOR_CCW ? -speed_part_a : speed_part_a) + hold;
    *drive = net >= 0.0f ? MOTOR_CW : MOTOR_CCW;
    float magnitude = fabsf(net);
    return magnitude < GRAVITY_HOLD_MIN_A ? 0.0f : fminf(magnitude, limit);
}

/* One 1 ms control step of one motor. Caller owns g_motor_lock. Same logic as
 * the single-motor adc_sample_task, with every state variable taken from m. */
static void motor_control_step(motor_t *m, int64_t now_us, const control_rates_t *rates)
{
    float sequence_pwm;
    if (sequence_tick(&m->sequence, now_us, &sequence_pwm)) {
        motor_direction_t direction = sequence_pwm > 0 ? MOTOR_CW : sequence_pwm < 0 ? MOTOR_CCW : MOTOR_STOP;
        motor_set_output_percent(m, direction, fabsf(sequence_pwm));
    }
    uint32_t duty_ticks = m->duty_cw + m->duty_ccw;
    current_sample_t current = current_sense_read(m->index);
    int64_t sample_time_us = current.sample_time_us;
    /* Encoder timing belongs to the PCNT read, not the ADC DMA block. */
    encoder_sample_t encoder = encoder_sample(m->index);
    const bool valid = current_sample_usable(current.status) &&
        current.dma_block_samples >= SENSE_MIN_BLOCK_SAMPLES && isfinite(current.current_mean_a);

    bool apply_pid_output = false;
    bool stop_for_sensing = current.status == CURRENT_ADC_ERROR || current.status == CURRENT_DMA_OVERFLOW;
    stop_for_sensing |= current.overflows != m->previous_dma_overflows;
    m->previous_dma_overflows=current.overflows;
    /* DRV8876 nFAULT (open drain, active low): a sustained low stops this motor only. */
    bool drv_fault = false;
#if DRV_NFAULT_ENABLED
    if (m->nfault_pin >= 0) {
        if (gpio_get_level(m->nfault_pin) == 0) {
            if (m->nfault_low_ms < 1000000U) m->nfault_low_ms++;
        } else m->nfault_low_ms = 0;
        drv_fault = m->nfault_low_ms >= DRV_NFAULT_FILTER_MS;
        if (drv_fault && (duty_ticks > 0 || m->control_mode != CONTROL_MODE_OPEN)) {
            stop_for_sensing = true;
            m->pending_drv_fault = true;
        }
    }
#endif
    motor_direction_t pid_direction = MOTOR_STOP;
    float pid_output_percent = 0.0f;

    portENTER_CRITICAL(&g_state_lock);
    if(m->control_mode==CONTROL_MODE_POSITION) {
        if(!encoder.speed_valid || !isfinite(encoder.output_position_rad) ||
           !isfinite(encoder.output_speed_rad_s) || encoder.reference_id!=m->position_reference_id ||
           encoder.time_us<=0 || now_us-encoder.time_us>=SPEED_MAX_GAP_US ||
           (m->position.last_update_us && encoder.time_us-m->position.last_update_us>=POSITION_MAX_GAP_US)) {
            stop_for_sensing=true;m->pending_position_fault=true;
        } else if(!stop_for_sensing && valid && encoder.time_us-m->position.last_update_us>=POSITION_PERIOD_US) {
            bool downstream=m->speed.limited ||
                (m->pid.output_percent>=m->pid.output_max_percent-0.01f && m->pid.error_a>0);
            int previous_direction=m->position.direction;
            if(!position_pi_update(&m->position,encoder.output_position_rad,encoder.output_speed_rad_s,
                                   encoder.time_us,downstream)) {
                stop_for_sensing=true;m->pending_position_fault=true;
            } else {
                if(previous_direction!=m->position.direction || m->position.output_rad_s==0) {
                    speed_pi_reset(&m->speed);
                    /* With GRAV the current loop keeps holding the link (signed
                     * cascade): do not drop its integrator here. */
                    if(m->gravity_a==0.0f){pid_reset(&m->pid);m->pid.current_ref_a=0;}
                    m->speed_current_a=0;
                    m->speed.last_update_us=encoder.time_us;
                }
                m->control_direction=m->position.direction>0 ? MOTOR_CW : MOTOR_CCW;
                m->speed.reference_rad_s=fabsf(m->position.output_rad_s);
            }
        }
    }
    bool cascade = m->control_mode == CONTROL_MODE_SPEED || m->control_mode == CONTROL_MODE_POSITION;
    if (cascade) {
        if (!encoder.speed_valid || !isfinite(encoder.output_speed_rad_s) ||
            encoder.time_us <= 0 || now_us-encoder.time_us >= SPEED_MAX_GAP_US ||
            (m->speed.last_update_us && encoder.time_us-m->speed.last_update_us >= SPEED_MAX_GAP_US)) {
            stop_for_sensing=true;
            m->pending_speed_fault=true;
        } else if (!stop_for_sensing && valid) {
            if (m->speed.reference_rad_s == 0) {
                speed_pi_reset(&m->speed);
                m->speed.last_update_us=encoder.time_us;
                m->speed_current_a=0;
            } else if (encoder.time_us-m->speed.last_update_us >= SPEED_PERIOD_US) {
                float projected=(m->control_direction==MOTOR_CCW ? -1 : 1)*encoder.output_speed_rad_s;
                /* Inner saturation only blocks the speed integrator when the current
                 * loop drives in the motion direction. */
                bool inner_high=m->drive_direction==m->control_direction &&
                    m->pid.output_percent >= m->pid.output_max_percent-0.01f && m->pid.error_a>0;
                if (speed_pi_update(&m->speed,projected,encoder.time_us,inner_high))
                    m->speed_current_a=m->speed.output_a;
                else { stop_for_sensing=true; m->pending_speed_fault=true; }
            }
        }
    }
    /* Cascade -> current loop. With GRAV the current is signed (see
     * cascade_current): at the target it is just the gravity current, which
     * holds the link still; a brake alone cannot hold a static load. */
    bool direction_change_brake = false;
    if (cascade && !stop_for_sensing && valid) {
        motor_direction_t drive;
        float gravity_term = 0.0f;
        m->pid.current_ref_a = cascade_current(m, m->speed_current_a, encoder.output_position_rad,
                                               &drive, &gravity_term);
        m->gravity_holding = m->control_mode == CONTROL_MODE_POSITION && gravity_term != 0.0f &&
            m->pid.current_ref_a > 0 &&
            (m->position.state != POSITION_TRACK || m->speed_current_a == 0.0f);
        if (drive != m->drive_direction) {
            /* New actuation direction: start the current integrator afresh. */
            pid_reset(&m->pid);
            m->drive_direction = drive;
        }
        /* Break before make when the driven direction flips while running. */
        if (motor_driving(m) && m->drive_direction != m->direction) {
            direction_change_brake = true;
            pid_reset(&m->pid);
        }
    } else if (!cascade) {
        m->drive_direction = m->control_direction;
        m->gravity_holding = false;
    }
    if (m->control_mode != CONTROL_MODE_OPEN) {
        if (!valid || m->drive_direction == MOTOR_STOP ||
            (!direction_change_brake && motor_driving(m) && m->drive_direction != m->direction) ||
            (m->pid.last_sample_us && now_us-m->pid.last_sample_us >= SENSE_PID_LOSS_TIMEOUT_US))
            stop_for_sensing = true;
        /* POSITION outside TRACK (arriving, holding, reversing) with nothing to
         * drive: brake instead of coasting so the load cannot drift. Also brake
         * when the axis overruns the speed reference with zero current asked. */
        bool brake_hold = m->hold_brake && m->control_mode == CONTROL_MODE_POSITION &&
                          m->position.state != POSITION_TRACK;
        if (m->hold_brake && cascade && encoder.speed_valid && isfinite(encoder.output_speed_rad_s)) {
            float projected = (m->control_direction == MOTOR_CCW ? -1.0f : 1.0f) * (float)encoder.output_speed_rad_s;
            if (projected > m->speed.reference_rad_s + OVERSPEED_BRAKE_RAD_S) brake_hold = true;
        }
        if (!stop_for_sensing && direction_change_brake) {
            m->pid.last_sample_us = sample_time_us;
            pid_direction = MOTOR_BRAKE;
            apply_pid_output = true;
        } else if (!stop_for_sensing && m->pid.current_ref_a == 0) {
            pid_reset(&m->pid);
            m->pid.last_sample_us = sample_time_us;
            pid_direction = brake_hold ? MOTOR_BRAKE : MOTOR_STOP;
            apply_pid_output = true;
        } else if (!stop_for_sensing && sample_time_us > m->pid.last_sample_us) {
            if (pi_update_sample(&m->pid, current.current_mean_a, sample_time_us)) {
                pid_output_percent = m->pid.output_percent;
                /* Static-friction feed-forward: cascade only, when driving the motion
                 * (never in pure current mode, keeping the identified plant intact). */
                if (cascade && m->drive_direction == m->control_direction && m->speed_current_a > 0 &&
                    m->friction_pwm > 0)
                    pid_output_percent = fminf(m->pid.output_max_percent, pid_output_percent + m->friction_pwm);
                pid_direction = m->drive_direction;
                apply_pid_output = true;
            } else stop_for_sensing = true;
        }
    } else {
        m->pid.error_a = valid ? m->pid.current_ref_a - current.current_mean_a : NAN;
    }

    telemetry_t *t = &m->telemetry;
    t->time_ms = (uint32_t)(sample_time_us / 1000ULL);
    t->sample_time_us = sample_time_us;
    t->encoder_count = encoder.count_x4;
    t->encoder = encoder;
    t->sequence_id = m->sequence.id;
    t->sequence_index = m->sequence.index;
    t->sequence_start_us = m->sequence.start_us;
    t->sequence_stage_start_us = m->sequence.stage_start_us;
    t->sequence_active = m->sequence.active;
    t->nfault_active = drv_fault;
    t->hold_brake = m->hold_brake;
    t->gravity_holding = m->gravity_holding;
    t->friction_pwm = m->friction_pwm;
    t->gravity_a = m->gravity_a;
    t->gravity_offset_rad = m->gravity_offset_rad;
    t->adc_sample_count = current.total_samples;
    t->adc_raw_mean = current.adc_raw_mean;
    t->adc_voltage_mean = current.adc_voltage_mean;
    t->current_mean_a = current.current_mean_a;
    t->dma_block_samples = current.dma_block_samples;
    t->adc_raw = current.raw;
    t->adc_voltage = current.voltage;
    t->adc_calibrated = current.calibrated;
    t->current_raw_a = current.current_a;
    t->current_status = current.status;
    t->adc_read_us = current.read_us;
    t->dma_overflows = current.overflows;
    t->sample_phase_us = current.phase_us;
    t->current_valid_count = current.accepted_samples;
    t->rejected_window = current.rejected_window;
    t->rejected_quality = current.rejected_quality;
    t->sample_attempt_count = rates->attempt_count;
    t->sample_missed_count = rates->missed_count;
    t->pwm_frequency_observed_hz = rates->observed_pwm_hz;
    t->acquisition_rate_observed_hz = rates->observed_attempt_hz;
    t->adc_rate_observed_hz = rates->observed_adc_hz;
    t->current_ref_a = m->pid.current_ref_a;
    t->current_error_a = m->pid.error_a;
    t->control_output_percent = m->pid.output_percent;
    t->pid_kp = m->pid.kp;
    t->pid_ki = m->pid.ki;
    t->pid_kd = m->pid.kd;
    t->pid_integrator = m->pid.integrator;
    t->pid_output_min_percent = m->pid.output_min_percent;
    t->pid_output_max_percent = m->pid.output_max_percent;
    t->control_mode = m->control_mode;
    t->speed = m->speed;
    t->position = m->position;
    t->position_reference_set = m->position_reference_set &&
        encoder.reference_id == m->position_reference_id;
    t->control_direction_sign = m->control_direction==MOTOR_CCW ? -1 : 1;
    t->direction = m->direction;
    t->duty_cw = m->duty_cw;
    t->duty_ccw = m->duty_ccw;
    t->duty_percent = m->duty_percent;
    portEXIT_CRITICAL(&g_state_lock);

    if (stop_for_sensing) {
        /* Emit ERR from the UART task, never block this task on printf. */
        if (!drv_fault && !m->pending_speed_fault && !m->pending_position_fault &&
            (duty_ticks > 0 || m->control_mode != CONTROL_MODE_OPEN))
            m->pending_current_fault = valid ? CURRENT_OUTPUT_SETTLING : current.status;
        control_stop(m);
    } else if (apply_pid_output) {
        motor_set_output_percent(m, pid_direction, pid_output_percent);
    }
}

static void adc_sample_task(void *arg)
{
    control_rates_t rates = {0};
    uint32_t rate_cycles = 0, rate_attempts = 0, rate_adc = 0;
    int64_t rate_time_us = 0;

    (void)arg;

    while (true) {
        uint32_t notifications = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        if (notifications > 1) rates.missed_count += notifications - 1;
        rates.attempt_count++;

        xSemaphoreTake(g_motor_lock, portMAX_DELAY);
        int64_t now_us = esp_timer_get_time();

        /* Counts/rates are shared by all motors (one timer, one DMA stream).
         * The ADC rate is per channel (nominal 64 kS/s). */
        uint32_t adc_sample_count = g_motors[0].telemetry.adc_sample_count;
        portENTER_CRITICAL(&g_clock_lock);
        uint32_t cycles = g_pwm_cycle_count;
        int64_t clock_time_us = esp_timer_get_time();
        portEXIT_CRITICAL(&g_clock_lock);
        if (!rate_time_us) {
            rate_time_us = clock_time_us;
            rate_cycles = cycles; rate_attempts = rates.attempt_count; rate_adc = adc_sample_count;
        } else if (clock_time_us - rate_time_us >= 1000000) {
            float seconds = (clock_time_us - rate_time_us) / 1000000.0f;
            rates.observed_pwm_hz = (uint32_t)(cycles - rate_cycles) / seconds;
            rates.observed_attempt_hz = (uint32_t)(rates.attempt_count - rate_attempts) / seconds;
            rates.observed_adc_hz = (uint32_t)(adc_sample_count - rate_adc) / seconds;
            rate_time_us = clock_time_us;
            rate_cycles = cycles; rate_attempts = rates.attempt_count; rate_adc = adc_sample_count;
        }

        for (int i = 0; i < MOTOR_COUNT; ++i) motor_control_step(&g_motors[i], now_us, &rates);
        xSemaphoreGive(g_motor_lock);
    }
}

static bool parse_direction(const char *text, motor_direction_t *direction)
{
    if (!text) return false;
    if (strcmp(text, "STOP") == 0) { *direction = MOTOR_STOP; return true; }
    if (strcmp(text, "CW") == 0) { *direction = MOTOR_CW; return true; }
    if (strcmp(text, "CCW") == 0) { *direction = MOTOR_CCW; return true; }
    return false;
}

static bool parse_control_mode(const char *text, control_mode_t *mode)
{
    if (!text) return false;
    if (strcmp(text, "OPEN") == 0) { *mode = CONTROL_MODE_OPEN; return true; }
    if (strcmp(text,"POSITION")==0) { *mode=CONTROL_MODE_POSITION; return true; }
    if (strcmp(text, "SPEED") == 0) { *mode=CONTROL_MODE_SPEED; return true; }
    if (strcmp(text, "PID") == 0) { *mode = CONTROL_MODE_PID; return true; }
    return false;
}

/* "M1".."M3", "1".."3" (digits accepted so SEL,2 works), "COXA", "FEMUR", "TIBIA" -> 0..2; "ALL" -> MOTOR_COUNT; else -1. */
static int parse_motor_token(const char *text)
{
    if (!text) return -1;
    if (strcmp(text, "ALL") == 0) return MOTOR_COUNT;
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        if (strcmp(text, k_motor_names[i]) == 0) return i;
    }
    const char *digits = (text[0] == 'M' || text[0] == 'm') ? text + 1 : text;
    if (digits[0] >= '1' && digits[0] <= '0' + MOTOR_COUNT && digits[1] == '\0')
        return digits[0] - '1';
    return -1;
}

/* Per-motor command set: identical wire format to ControlCurrentN20. */
static void handle_motor_command(motor_t *m, char *command)
{
    if (command == NULL) return;

    if (strcmp(command, "SEQ") == 0) {
        pwm_sequence_t candidate;
        if (!sequence_parse(strtok(NULL, ""), &candidate)) {
            reply("ERR,SEQ,percent:ms;percent:ms,max24,total120s,min100ms\n");
            return;
        }
        control_stop(m);
        m->sequence = candidate;
        m->sequence.id = ++g_sequence_id;
        /* Each experiment starts with a firmware-owned position reference.
         * Re-zeroing does not reset raw counts or the velocity history. */
        encoder_zero_reference(m->index);
        sequence_start(&m->sequence, esp_timer_get_time());
        reply("ACK,SEQ,%lu,%d\n", (unsigned long)m->sequence.id, m->sequence.count);
        return;
    }

    if (strcmp(command, "ENCODER") == 0) {
        char *action = strtok(NULL, ",");
        if (action == NULL || strcmp(action, "ZERO") != 0 || strtok(NULL, ",") != NULL) {
            reply("ERR,ENCODER,expected_ZERO\n");
        } else if (!motor_is_stopped(m)) {
            reply("ERR,ENCODER,STOP_before_zero\n");
        } else {
            encoder_zero_reference(m->index);
            m->position_reference_set=false;
            reply("ACK,ENCODER,ZERO\n");
        }
        return;
    }

    if (strcmp(command, "ESIGN") == 0) {
        char *value = strtok(NULL, ",");
        int sign = value ? atoi(value) : 0;
        if ((sign != 1 && sign != -1) || strtok(NULL, ",") != NULL) {
            reply("ERR,ESIGN,expected_1_or_-1\n");
        } else if (!motor_is_stopped(m)) {
            reply("ERR,ESIGN,STOP_before_sign_change\n");
        } else {
            encoder_set_sign(m->index, sign);
            m->position_reference_set=false;
            reply("ACK,ESIGN,%d\n", sign);
        }
        return;
    }

    if (strcmp(command, "STOP") == 0) {
        control_stop(m);
        reply("ACK,STOP,0.000,0\n");
        return;
    }

    /* HOLD,BRAKE|COAST: what POSITION does once it is not tracking. */
    if (strcmp(command, "HOLD") == 0) {
        char *mode = strtok(NULL, ",");
        if (mode && strcmp(mode, "BRAKE") == 0 && !strtok(NULL, ",")) m->hold_brake = true;
        else if (mode && strcmp(mode, "COAST") == 0 && !strtok(NULL, ",")) m->hold_brake = false;
        else { reply("ERR,HOLD,expected_BRAKE_or_COAST\n"); return; }
        reply("ACK,HOLD,%s\n", m->hold_brake ? "BRAKE" : "COAST");
        return;
    }

    /* FRIC,pct: PWM added to the cascade output to overcome static friction. */
    if (strcmp(command, "FRIC") == 0) {
        float pct; char *value = strtok(NULL, ",");
        if (!control_parse_float(value, &pct) || pct < 0 || pct > 40 || strtok(NULL, ",")) {
            reply("ERR,FRIC,expected_0_to_40_percent\n"); return;
        }
        portENTER_CRITICAL(&g_state_lock);
        m->friction_pwm = pct;
        portEXIT_CRITICAL(&g_state_lock);
        reply("ACK,FRIC,%.2f\n", pct);
        return;
    }

    /* GRAV,amps,offset_rad: current feed-forward amps*cos(position + offset),
     * positive when gravity pulls the joint toward negative positions. */
    if (strcmp(command, "GRAV") == 0) {
        float amps, offset; char *a = strtok(NULL, ","), *b = strtok(NULL, ",");
        if (!control_parse_float(a, &amps) || !control_parse_float(b, &offset) ||
            fabsf(amps) > PI_REFERENCE_MAX_A || fabsf(offset) > 10 || strtok(NULL, ",")) {
            reply("ERR,GRAV,expected_amps_-1_to_1_and_offset_rad\n"); return;
        }
        portENTER_CRITICAL(&g_state_lock);
        m->gravity_a = amps;
        m->gravity_offset_rad = offset;
        portEXIT_CRITICAL(&g_state_lock);
        reply("ACK,GRAV,%.4f,%.4f\n", amps, offset);
        return;
    }

    if (strcmp(command, "SET") == 0) {
        char *pwm_text = strtok(NULL, "");
        char *end = NULL;
        float pwm_percent = pwm_text ? strtof(pwm_text, &end) : NAN;
        if (!pwm_text || end == pwm_text || *end != '\0' ||
            !isfinite(pwm_percent) || pwm_percent < -100.0f || pwm_percent > 100.0f) {
            reply("ERR,SET,expected_percent_-100_to_100\n");
            return;
        }
        motor_direction_t direction = pwm_percent > 0 ? MOTOR_CW :
                                      pwm_percent < 0 ? MOTOR_CCW : MOTOR_STOP;
        control_set_mode(m, CONTROL_MODE_OPEN);
        motor_set_output_percent(m, direction, fabsf(pwm_percent));
        reply("ACK,SET,%.3f\n", pwm_percent);
        return;
    }

    if (strcmp(command, "MODE") == 0) {
        char *mode_text = strtok(NULL, ",");
        char *direction_text = strtok(NULL, ",");
        control_mode_t mode = CONTROL_MODE_OPEN;
        motor_direction_t requested_direction = m->control_direction;

        if (mode_text == NULL || !parse_control_mode(mode_text, &mode)) return;
        if (direction_text != NULL) {
            if (!parse_direction(direction_text, &requested_direction)) return;
        }

        if(m->control_mode==CONTROL_MODE_POSITION && mode!=CONTROL_MODE_OPEN) {
            reply("ERR,MODE,STOP_before_leaving_or_restarting_POSITION\n");return;
        }
        if(mode==CONTROL_MODE_POSITION) {
            if(!control_start_position(m))return;
        } else if (mode == CONTROL_MODE_SPEED) {
            if (!control_start_speed(m, requested_direction)) return;
        } else if (mode == CONTROL_MODE_PID) {
            if (!control_start_pid(m, requested_direction, m->control_mode==CONTROL_MODE_SPEED ? 0 : m->pid.current_ref_a)) return;
        } else {
            if(m->control_mode==CONTROL_MODE_POSITION)control_stop(m);
            else control_set_mode(m, mode);
            m->control_direction = requested_direction;
        }
        reply("ACK,MODE,%s,%s\n", control_mode_to_text(mode), motor_direction_to_text(m->control_direction));
        return;
    }

    if (strcmp(command, "DIR") == 0) {
        char *direction_text = strtok(NULL, ",");
        motor_direction_t direction = MOTOR_STOP;

        if (direction_text == NULL || !parse_direction(direction_text, &direction)) return;

        if (m->control_mode != CONTROL_MODE_OPEN && direction != m->control_direction) {
            reply("ERR,DIR,STOP_before_changing_PI_direction\n");
            return;
        }
        portENTER_CRITICAL(&g_state_lock);
        m->control_direction = direction;
        portEXIT_CRITICAL(&g_state_lock);
        reply("ACK,DIR,%s\n", motor_direction_to_text(direction));
        return;
    }

    if (strcmp(command, "IREF") == 0) {
        if (m->control_mode==CONTROL_MODE_SPEED || m->control_mode==CONTROL_MODE_POSITION) { reply("ERR,IREF,reference_owned_by_outer_loop\n"); return; }
        char *current_text = strtok(NULL, ",");
        if (current_text == NULL) return;

        float current_ref_a;
        if (!control_parse_float(current_text, &current_ref_a) || !pi_reference_valid(current_ref_a)) {
            reply("ERR,IREF,expected_0_to_1A_eq\n");
            return;
        }

        portENTER_CRITICAL(&g_state_lock);
        m->pid.current_ref_a = current_ref_a;
        if (current_ref_a == 0) pid_reset(&m->pid);
        portEXIT_CRITICAL(&g_state_lock);
        if (current_ref_a == 0 && m->control_mode == CONTROL_MODE_PID) motor_set_output(m, MOTOR_STOP, 0);
        reply("ACK,IREF,%.4f\n", current_ref_a);
        return;
    }

    if (strcmp(command, "PID") == 0) {
        char *kp_text = strtok(NULL, ",");
        char *ki_text = strtok(NULL, ",");
        char *kd_text = strtok(NULL, ",");
        float kp = 0.0f, ki = 0.0f, kd = 0.0f;

        if (kp_text == NULL || ki_text == NULL || kd_text == NULL) return;

        if (!control_parse_float(kp_text, &kp) || !control_parse_float(ki_text, &ki) ||
            !control_parse_float(kd_text, &kd) || kp < 0 || ki < 0 || kd != 0 || strtok(NULL, ",")) {
            reply("ERR,PID,PI_requires_finite_Kp_Ki_nonnegative_Kd_zero\n");
            return;
        }

        portENTER_CRITICAL(&g_state_lock);
        m->pid.kp = kp;
        m->pid.ki = ki;
        m->pid.kd = kd;
        if (m->control_mode != CONTROL_MODE_OPEN) {
            m->pid.integrator = m->pid.output_percent-kp*m->pid.error_a;
        } else pid_reset(&m->pid);
        portEXIT_CRITICAL(&g_state_lock);
        reply("ACK,PID,%.4f,%.4f,%.4f\n", kp, ki, kd);
        return;
    }

    if (strcmp(command, "LIMIT") == 0) {
        char *min_text = strtok(NULL, ",");
        char *max_text = strtok(NULL, ",");

        if (min_text == NULL || max_text == NULL) return;

        float min_percent, max_percent;
        if (!control_parse_float(min_text, &min_percent) || !control_parse_float(max_text, &max_percent) ||
            min_percent < 0 || max_percent > 100 || max_percent < min_percent || strtok(NULL, ",")) {
            reply("ERR,LIMIT,expected_0_le_min_le_max_le_100\n");
            return;
        }
        portENTER_CRITICAL(&g_state_lock);
        m->pid.output_min_percent = min_percent;
        m->pid.output_max_percent = max_percent;
        m->pid.output_percent = clamp_float(m->pid.output_percent, min_percent, max_percent);
        if (m->control_mode != CONTROL_MODE_OPEN)
            m->pid.integrator = m->pid.output_percent-m->pid.kp*m->pid.error_a;
        portEXIT_CRITICAL(&g_state_lock);
        reply("ACK,LIMIT,%.3f,%.3f\n", min_percent, max_percent);
        return;
    }

    if(strcmp(command,"PREF")==0) {
        char *value=strtok(NULL,",");char *end=NULL;
        double ref=value ? strtod(value,&end) : NAN;
        if(!value || end==value || *end || !position_reference_valid(ref) || strtok(NULL,",")) {
            reply("ERR,PREF,expected_rad_within_10000\n");return;
        }
        encoder_sample_t enc=encoder_sample(m->index);
        portENTER_CRITICAL(&g_state_lock);
        m->position.reference_rad=ref;m->position_reference_id=enc.reference_id;m->position_reference_set=true;
        position_pi_reset(&m->position);m->position.last_update_us=enc.time_us;
        if(m->control_mode==CONTROL_MODE_POSITION) {
            speed_pi_reset(&m->speed);m->speed.reference_rad_s=0;m->speed.last_update_us=enc.time_us;
            m->speed_current_a=0;
            /* With GRAV keep holding the link while the new target is taken. */
            if(m->gravity_a==0.0f){pid_reset(&m->pid);m->pid.current_ref_a=0;}
        }
        portEXIT_CRITICAL(&g_state_lock);
        if(m->control_mode==CONTROL_MODE_POSITION && m->gravity_a==0.0f)motor_set_output(m, MOTOR_STOP,0);
        reply("ACK,PREF,%.6f\n",ref);return;
    }
    if(strcmp(command,"PPI")==0 || strcmp(command,"PLIMIT")==0) {
        if(m->control_mode==CONTROL_MODE_POSITION) {reply("ERR,POSITION,STOP_before_parameter_change\n");return;}
        bool gains=strcmp(command,"PPI")==0;
        float a,b,c=0;char *ta=strtok(NULL,","),*tb=strtok(NULL,","),*tc=gains ? NULL : strtok(NULL,",");
        bool ok=control_parse_float(ta,&a) && control_parse_float(tb,&b);
        if(!gains)ok=ok && control_parse_float(tc,&c);
        ok=ok && !strtok(NULL,",");
        if(!ok || (gains ? (a<0 || b<0) : (a<=0 || a>32 || b<=0 || b>100 || c<0.0023f || c>1))) {
            reply("ERR,POSITION,invalid_gains_or_limits\n");return;
        }
        portENTER_CRITICAL(&g_state_lock);
        if(gains){m->position.kp=a;m->position.ki=b;}
        else {m->position.max_speed_rad_s=a;m->position.accel_rad_s2=b;m->position.tolerance_rad=c;}
        position_pi_reset(&m->position);
        portEXIT_CRITICAL(&g_state_lock);
        reply("ACK,%s,%.6f,%.6f,%.6f\n",command,a,b,c);return;
    }

    if (strcmp(command,"WREF")==0) {
        if(m->control_mode==CONTROL_MODE_POSITION) {reply("ERR,WREF,use_PREF_in_POSITION\n");return;}
        float ref; char *value=strtok(NULL,",");
        if (!control_parse_float(value,&ref) || !speed_reference_valid(ref) || strtok(NULL,",")) {
            reply("ERR,WREF,expected_0_to_32_rad_s\n"); return;
        }
        portENTER_CRITICAL(&g_state_lock);
        m->speed.reference_rad_s=ref;
        if (ref==0) {
            speed_pi_reset(&m->speed);
            if (m->control_mode==CONTROL_MODE_SPEED) { m->pid.current_ref_a=0; pid_reset(&m->pid); }
        }
        portEXIT_CRITICAL(&g_state_lock);
        if (ref==0 && m->control_mode==CONTROL_MODE_SPEED) motor_set_output(m, MOTOR_STOP,0);
        reply("ACK,WREF,%.4f\n",ref); return;
    }
    if (strcmp(command,"SPI")==0) {
        float kp,ki; char *a=strtok(NULL,","), *b=strtok(NULL,",");
        if (!control_parse_float(a,&kp) || !control_parse_float(b,&ki) || kp<0 || ki<0 || strtok(NULL,",")) {
            reply("ERR,SPI,expected_finite_nonnegative_Kp_Ki\n"); return;
        }
        portENTER_CRITICAL(&g_state_lock);
        m->speed.kp=kp; m->speed.ki=ki;
        if (m->control_mode==CONTROL_MODE_SPEED || m->control_mode==CONTROL_MODE_POSITION) m->speed.integral_a=m->speed.output_a-kp*m->speed.error_rad_s;
        else speed_pi_reset(&m->speed);
        portEXIT_CRITICAL(&g_state_lock);
        reply("ACK,SPI,%.6f,%.6f\n",kp,ki); return;
    }
    if (strcmp(command,"ILIMIT")==0) {
        float cap; char *value=strtok(NULL,",");
        if (!control_parse_float(value,&cap) || cap<=0 || cap>PI_REFERENCE_MAX_A || strtok(NULL,",")) {
            reply("ERR,ILIMIT,expected_0_lt_A_eq_le_1\n"); return;
        }
        portENTER_CRITICAL(&g_state_lock);
        bool clamped=m->speed.output_a>cap;
        m->speed.current_limit_a=cap;
        m->speed.output_a=fminf(cap,m->speed.output_a);
        if (m->control_mode==CONTROL_MODE_SPEED || m->control_mode==CONTROL_MODE_POSITION) {
            m->pid.current_ref_a=fminf(cap,m->pid.current_ref_a);
            if (clamped) m->speed.integral_a=m->speed.output_a-m->speed.kp*m->speed.error_rad_s;
        }
        portEXIT_CRITICAL(&g_state_lock);
        reply("ACK,ILIMIT,%.4f\n",cap); return;
    }

    if (strcmp(command, "CTRL") == 0) {
        if(m->control_mode==CONTROL_MODE_POSITION) {reply("ERR,CTRL,STOP_before_leaving_POSITION\n");return;}
        char *direction_text = strtok(NULL, ",");
        char *current_text = strtok(NULL, ",");
        motor_direction_t direction = MOTOR_STOP;

        if (direction_text == NULL || current_text == NULL || !parse_direction(direction_text, &direction)) return;

        float current_ref_a;
        if (!control_parse_float(current_text, &current_ref_a) || !pi_reference_valid(current_ref_a)) {
            reply("ERR,IREF,expected_0_to_1A_eq\n");
            return;
        }

        if (!control_start_pid(m, direction, current_ref_a)) return;
        reply("ACK,CTRL,%s,%.4f\n", motor_direction_to_text(direction), current_ref_a);
        return;
    }

    reply("ERR,UNKNOWN,%s\n", command);
}

/* DATA version 6 removes the former filter fields; encoder values remain SI.
 * Header order exactly matches processing_print_telemetry below. */
static void processing_print_header(void)
{
    printf("HEADER,time_ms,adc_raw,adc_voltage,current_raw_a,direction_code,direction_text,"
           "duty_cw_percent,duty_ccw_percent,adc_sample_count,pwm_frequency_hz,adc_sample_rate_hz,"
           "adc_sample_tick,control_mode_code,control_mode_text,current_ref_a,"
           "current_error_a,control_output_percent,duty_percent,pid_kp,pid_ki,pid_kd,pid_integrator,"
           "adc_calibrated,sample_time_us,encoder_count,sequence_id,sequence_index,sequence_start_us,sequence_stage_start_us,"
           "current_valid,current_status,adc_read_us,sample_phase_us,current_valid_count,sample_attempt_count,"
           "sample_missed_count,pwm_frequency_observed_hz,acquisition_rate_observed_hz,adc_rate_observed_hz,dma_overflows,"
           "sensing_version,dma_rejected_window,dma_rejected_quality,dma_phase_validated,"
           "adc_raw_mean,adc_voltage_mean,current_mean_a,dma_block_samples,"
           "encoder_version,encoder_time_us,encoder_relative_count_x4,output_position_rad,"
           "output_speed_rad_s,encoder_speed_valid,encoder_speed_window_us,counts_x4_per_output_rev,"
           "encoder_reference_count_x4,encoder_reference_id\n");
}

static void processing_print_telemetry(const telemetry_t *telemetry)
{
    printf("DATA,%lu,%d,%.3f,%.3f,%d,%s,%.3f,%.3f,%lu,%lu,%lu,%lu,%d,%s,"
           "%.4f,%.4f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%d,%lld,%lld,%lu,%d,%lld,%lld,"
           "%d,%d,%lu,%ld,%lu,%lu,%lu,%.1f,%.1f,%.1f,%lu,%u,%lu,%lu,%d,%.3f,%.6f,%.6f,%lu,"
           "%u,%lld,%lld,%.9f,%.9f,%d,%lu,%u,%lld,%lu\n",
           (unsigned long)telemetry->time_ms,
           telemetry->adc_raw,
           telemetry->adc_voltage,
           telemetry->current_raw_a,
           (int)telemetry->direction,
           motor_direction_to_text(telemetry->direction),
           duty_to_percent(telemetry->duty_cw),
           duty_to_percent(telemetry->duty_ccw),
           (unsigned long)telemetry->adc_sample_count,
           (unsigned long)PWM_FREQUENCY_HZ,
           (unsigned long)SENSE_PER_CHANNEL_RATE_HZ,
           (unsigned long)0,
           (int)telemetry->control_mode,
           control_mode_to_text(telemetry->control_mode),
           telemetry->current_ref_a,
           telemetry->current_error_a,
           telemetry->control_output_percent,
           telemetry->duty_percent,
           telemetry->pid_kp,
           telemetry->pid_ki,
           telemetry->pid_kd,
           telemetry->pid_integrator,
           telemetry->adc_calibrated ? 1 : 0,
           (long long)telemetry->sample_time_us,
           (long long)telemetry->encoder_count,
           (unsigned long)telemetry->sequence_id,
           telemetry->sequence_index,
           (long long)telemetry->sequence_start_us,
           (long long)telemetry->sequence_stage_start_us,
           current_sample_usable(telemetry->current_status) ? 1 : 0,
           (int)telemetry->current_status,
           (unsigned long)telemetry->adc_read_us,
           (long)telemetry->sample_phase_us,
           (unsigned long)telemetry->current_valid_count,
           (unsigned long)telemetry->sample_attempt_count,
           (unsigned long)telemetry->sample_missed_count,
           telemetry->pwm_frequency_observed_hz,
           telemetry->acquisition_rate_observed_hz,
           telemetry->adc_rate_observed_hz,
           (unsigned long)telemetry->dma_overflows,
           (unsigned)SENSE_DATA_VERSION,
           (unsigned long)telemetry->rejected_window,
           (unsigned long)telemetry->rejected_quality,
           0,
           telemetry->adc_raw_mean,
           telemetry->adc_voltage_mean,
           telemetry->current_mean_a,
           (unsigned long)telemetry->dma_block_samples,
           (unsigned)ENCODER_DATA_VERSION,
           (long long)telemetry->encoder.time_us,
           (long long)telemetry->encoder.relative_count_x4,
           telemetry->encoder.output_position_rad,
           telemetry->encoder.speed_valid ? telemetry->encoder.output_speed_rad_s : NAN,
           telemetry->encoder.speed_valid ? 1 : 0,
           (unsigned long)telemetry->encoder.speed_window_us,
           (unsigned)ENCODER_COUNTS_PER_OUTPUT_REV,
           (long long)telemetry->encoder.reference_count_x4,
           (unsigned long)telemetry->encoder.reference_id);
}

static void processing_print_speed(const telemetry_t *t)
{
    printf("SPEED,1,%lld,%d,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%lld\n",
        (long long)t->encoder.time_us, t->control_direction_sign,
        t->speed.reference_rad_s, t->speed.error_rad_s, t->speed.output_a,
        t->speed.kp, t->speed.ki, t->speed.integral_a, t->speed.current_limit_a,
        t->speed.limited ? 1 : 0, (long long)t->speed.last_update_us);
}

static void processing_print_position(const telemetry_t *t)
{
    printf("POSITION,1,%lld,%.6f,%.6f,%.5f,%.6f,%.6f,%.5f,%.4f,%.4f,%.5f,%d,%d,%lld,%lu\n",
        (long long)t->encoder.time_us,t->position.reference_rad,t->position.error_rad,
        t->position.output_rad_s,t->position.kp,t->position.ki,t->position.integral_rad_s,
        t->position.max_speed_rad_s,t->position.accel_rad_s2,t->position.tolerance_rad,
        (int)t->position.state,t->position.limited ? 1 : 0,(long long)t->position.last_update_us,
        (unsigned long)t->encoder.reference_id);
}

/* Compact multi-motor line, 100 Hz. Per motor (prefix mN_):
 * mode, duty_percent (signed), current_mean_a, current_ref_a, position_rad,
 * speed_rad_s (nan while invalid), speed_ref_rad_s (signed), position_ref_rad,
 * position_state, current_status, flags (1=PREF valid, 2=speed valid, 4=SEQ,
 * 8=nFAULT low, 16=braking, 32=holding with the gravity current). */
static void multi_print_header(void)
{
    printf("MHEADER,time_ms,selected");
    for (int i = 1; i <= MOTOR_COUNT; ++i) {
        printf(",m%d_mode,m%d_duty_percent,m%d_current_a,m%d_current_ref_a,m%d_position_rad,"
               "m%d_speed_rad_s,m%d_speed_ref_rad_s,m%d_position_ref_rad,m%d_position_state,"
               "m%d_current_status,m%d_flags", i, i, i, i, i, i, i, i, i, i, i);
    }
    printf("\n");
}

static void multi_print_motor(const telemetry_t *t)
{
    bool speed_loop = t->control_mode == CONTROL_MODE_SPEED || t->control_mode == CONTROL_MODE_POSITION;
    float speed_ref = speed_loop ? t->speed.reference_rad_s * (float)t->control_direction_sign : 0.0f;
    int flags = (t->position_reference_set ? 1 : 0) | (t->encoder.speed_valid ? 2 : 0) |
                (t->sequence_active ? 4 : 0) | (t->nfault_active ? 8 : 0) |
                (t->direction == MOTOR_BRAKE ? 16 : 0) | (t->gravity_holding ? 32 : 0);
    printf(",%d,%.2f,%.4f,%.4f,%.5f,%.4f,%.4f,%.5f,%d,%d,%d",
           (int)t->control_mode, t->duty_percent, t->current_mean_a,
           t->control_mode == CONTROL_MODE_OPEN ? 0.0f : t->current_ref_a,
           t->encoder.output_position_rad,
           t->encoder.speed_valid ? t->encoder.output_speed_rad_s : NAN,
           speed_ref, t->position.reference_rad, (int)t->position.state,
           (int)t->current_status, flags);
}

/* Settings of one motor, round-robin at ~3 Hz per motor in MULTI mode. */
static void multi_print_config(int motor, const telemetry_t *t)
{
    printf("CFG,M%d,%s,%.4f,%.4f,%.3f,%.3f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%d,%d,%.2f,%.4f,%.4f\n",
           motor + 1, k_motor_names[motor], t->pid_kp, t->pid_ki,
           t->pid_output_min_percent, t->pid_output_max_percent,
           t->speed.kp, t->speed.ki, t->speed.current_limit_a,
           t->position.kp, t->position.ki, t->position.max_speed_rad_s,
           t->position.accel_rad_s2, t->position.tolerance_rad, encoder_get_sign(motor),
           t->hold_brake ? 1 : 0, t->friction_pwm, t->gravity_a, t->gravity_offset_rad);
}

static void print_info(void)
{
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        printf("INFO,M%d,%s,IN1=%d,IN2=%d,CS=%d,ADC1_CH%d,ENC_A=%d,ENC_B=%d,NFAULT=%d,ESIGN=%d,"
               "COUNTS_REV=%u,CS_V_PER_A=%.3f%s\n",
               i + 1, k_motor_names[i], k_pin_in1[i], k_pin_in2[i], k_pin_cs[i],
               k_adc_channel[i], k_pin_enc_a[i], k_pin_enc_b[i],
               DRV_NFAULT_ENABLED ? k_pin_nfault[i] : -1, encoder_get_sign(i),
               (unsigned)ENCODER_COUNTS_PER_OUTPUT_REV, SENSE_CS_VOLTS_PER_AMP,
               i == g_selected ? ",SELECTED" : "");
    }
    printf("INFO,BOARD,%s,TEL,%s\n", BOARD_NAME, g_telemetry_mode == TELEMETRY_MULTI ? "MULTI" : "LEGACY");
}

static void handle_serial_command(char *line)
{
    char *first = strtok(line, ",");
    if (first == NULL) return;

    int target = parse_motor_token(first);
    if (target < 0) {
        g_reply_prefix[0] = '\0';
        /* Global commands (no motor prefix). */
        if (strcmp(first, "STOP") == 0) {
            for (int i = 0; i < MOTOR_COUNT; ++i) control_stop(&g_motors[i]);
            printf("ACK,STOP,0.000,0\n");
            return;
        }
        if (strcmp(first, "SEL") == 0) {
            int selected = parse_motor_token(strtok(NULL, ","));
            if (selected < 0 || selected >= MOTOR_COUNT || strtok(NULL, ",")) {
                printf("ERR,SEL,expected_M1_M2_M3\n");
                return;
            }
            g_selected = selected;
            printf("ACK,SEL,M%d,%s\n", selected + 1, k_motor_names[selected]);
            if (g_telemetry_mode == TELEMETRY_LEGACY) processing_print_header();
            return;
        }
        if (strcmp(first, "TEL") == 0) {
            char *mode = strtok(NULL, ",");
            if (mode && strcmp(mode, "MULTI") == 0) {
                g_telemetry_mode = TELEMETRY_MULTI;
                printf("ACK,TEL,MULTI\n");
                multi_print_header();
            } else if (mode && strcmp(mode, "LEGACY") == 0) {
                g_telemetry_mode = TELEMETRY_LEGACY;
                printf("ACK,TEL,LEGACY\n");
                processing_print_header();
            } else {
                printf("ERR,TEL,expected_LEGACY_or_MULTI\n");
            }
            return;
        }
        if (strcmp(first, "INFO") == 0) { print_info(); return; }
        if (strcmp(first, "DIAG") == 0) {
            current_sense_diag_t d = current_sense_diag();
            printf("DIAG,ADC,frames=%lu,bad=%lu,bytes=%lu,overflows=%lu,calibrated=%d,"
                   "malformed_last=%lu,malformed_total=%lu,channel_mask=0x%04x\n",
                   (unsigned long)d.frames, (unsigned long)d.bad_frames, (unsigned long)d.bytes_last,
                   (unsigned long)d.overflows, d.calibrated ? 1 : 0, (unsigned long)d.malformed_last,
                   (unsigned long)d.malformed_total, (unsigned)d.channel_mask_last);
            for (int i = 0; i < MOTOR_COUNT; ++i) {
                encoder_sample_t e = encoder_sample(i);
                current_sample_t c = current_sense_read(i);
                printf("DIAG,M%d,%s,ADC1_CH%d,count=%lu,accepted=%lu,saturated=%lu,raw_min=%d,raw_max=%d,"
                       "raw_mean=%.1f,status=%d,current_a=%.4f,enc_count=%lld,pos_rad=%.4f,speed_valid=%d,nfault_low=%d,mode=%s\n",
                       i + 1, k_motor_names[i], k_adc_channel[i], (unsigned long)d.ch[i].count,
                       (unsigned long)d.ch[i].accepted, (unsigned long)d.ch[i].saturated,
                       d.ch[i].raw_min, d.ch[i].raw_max, d.ch[i].raw_mean, (int)c.status,
                       c.current_mean_a, (long long)e.count_x4, e.output_position_rad,
                       e.speed_valid ? 1 : 0, g_motors[i].telemetry.nfault_active ? 1 : 0,
                       control_mode_to_text(g_motors[i].control_mode));
            }
            return;
        }
        /* Anything else goes to the selected motor, as in the single-motor firmware. */
        handle_motor_command(&g_motors[g_selected], first);
        return;
    }

    char *rest = strtok(NULL, "");
    if (rest == NULL) return;
    int first_motor = target == MOTOR_COUNT ? 0 : target;
    int last_motor = target == MOTOR_COUNT ? MOTOR_COUNT - 1 : target;
    char copy[COMMAND_BUFFER_SIZE];
    for (int i = first_motor; i <= last_motor; ++i) {
        snprintf(g_reply_prefix, sizeof(g_reply_prefix), "M%d,", i + 1);
        strncpy(copy, rest, sizeof(copy) - 1);
        copy[sizeof(copy) - 1] = '\0';
        handle_motor_command(&g_motors[i], strtok(copy, ","));
    }
    g_reply_prefix[0] = '\0';
}

static void serial_command_poll(void)
{
    static char command_buffer[COMMAND_BUFFER_SIZE];
    static size_t command_length = 0;
    static bool discard_line = false;
    char received;

    while (read(STDIN_FILENO, &received, 1) == 1) {
        if (received == '\n' || received == '\r') {
            if (discard_line) { discard_line = false; command_length = 0; continue; }
            if (command_length > 0) {
                command_buffer[command_length] = '\0';
                xSemaphoreTake(g_motor_lock, portMAX_DELAY);
                handle_serial_command(command_buffer);
                xSemaphoreGive(g_motor_lock);
                command_length = 0;
            }
            continue;
        }

        if (discard_line) continue;
        if (command_length < COMMAND_BUFFER_SIZE - 1) {
            command_buffer[command_length++] = received;
        } else {
            discard_line = true;
            command_length = 0;
        }
    }
}

static void telemetry_snapshot(int motor, telemetry_t *out)
{
    portENTER_CRITICAL(&g_state_lock);
    *out = g_motors[motor].telemetry;
    portEXIT_CRITICAL(&g_state_lock);
}

static void motor_init_state(motor_t *m, int index)
{
    *m = (motor_t){0};
    m->index = index;
    m->name = k_motor_names[index];
    m->gpio_cw = k_pin_in1[index];
    m->gpio_ccw = k_pin_in2[index];
    m->nfault_pin = DRV_NFAULT_ENABLED ? k_pin_nfault[index] : -1;
    m->sequence.index = -1;
    m->position = (position_pi_t){.kp=POSITION_KP_INITIAL,.ki=POSITION_KI_INITIAL,
        .max_speed_rad_s=2,.accel_rad_s2=5,.tolerance_rad=0.02f,.direction=1};
    m->speed = (speed_pi_t){.kp=SPEED_KP_INITIAL, .ki=SPEED_KI_INITIAL,
        .current_limit_a=SPEED_CURRENT_LIMIT_INITIAL_A};
    m->pid = (pid_controller_t){
        .kp = PID_KP_INITIAL,
        .ki = PID_KI_INITIAL,
        .kd = PID_KD_INITIAL,
        .output_min_percent = PID_OUTPUT_MIN_PERCENT,
        .output_max_percent = PID_OUTPUT_MAX_PERCENT,
        .current_ref_a = CURRENT_REF_INITIAL_A,
    };
    m->direction = MOTOR_STOP;
    m->control_mode = CONTROL_MODE_OPEN;
    m->control_direction = MOTOR_CW;
    m->drive_direction = MOTOR_CW;
    m->pending_current_fault = CURRENT_UNINITIALIZED;
    m->hold_brake = true;
    m->friction_pwm = 0.0f;   /* tune per joint with FRIC */
    m->gravity_a = 0.0f;      /* tune per joint with GRAV */
    m->gravity_offset_rad = 0.0f;
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);

    for (int i = 0; i < MOTOR_COUNT; ++i) motor_init_state(&g_motors[i], i);
    g_motor_lock = xSemaphoreCreateMutex();
    configASSERT(g_motor_lock != NULL);
    /* PWM outputs first, forced low, so no H-bridge is driven while the rest boots. */
    mcpwm_init();
#if DRV_NFAULT_ENABLED
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        if (g_motors[i].nfault_pin < 0) continue;
        gpio_config_t nfault_config = {
            .pin_bit_mask = 1ULL << g_motors[i].nfault_pin,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,   /* plus R4 10 k on the board */
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&nfault_config));
    }
#endif
    encoder_init();
    current_sense_init();
    serial_command_init();

    BaseType_t created = xTaskCreatePinnedToCore(adc_sample_task,
                            "adc_sample_task",
                            6144,
                            NULL,
                            10,
                            &g_adc_task_handle,
                            0);
    configASSERT(created == pdPASS);

    printf("BOOT,%s,motors=%d\n", BOARD_NAME, MOTOR_COUNT);
    print_info();
    processing_print_header();

    static telemetry_t telemetry;
    TickType_t telemetry_wake = xTaskGetTickCount();
    unsigned slow_divider = 0;
    int config_motor = 0;
    while (true) {
        serial_command_poll();
        bool position_fault[MOTOR_COUNT], speed_fault[MOTOR_COUNT], drv_fault[MOTOR_COUNT];
        current_status_t fault[MOTOR_COUNT];
        xSemaphoreTake(g_motor_lock, portMAX_DELAY);
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            motor_t *m = &g_motors[i];
            position_fault[i] = m->pending_position_fault; m->pending_position_fault = false;
            drv_fault[i] = m->pending_drv_fault; m->pending_drv_fault = false;
            speed_fault[i] = m->pending_speed_fault; m->pending_speed_fault = false;
            fault[i] = m->pending_current_fault; m->pending_current_fault = CURRENT_UNINITIALIZED;
        }
        xSemaphoreGive(g_motor_lock);
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            /* The legacy dashboard sees its own axis' errors unprefixed. */
            bool plain = g_telemetry_mode == TELEMETRY_LEGACY && i == g_selected;
            char prefix[16] = "";
            if (!plain) snprintf(prefix, sizeof(prefix), "M%d,", i + 1);
            if (drv_fault[i]) printf("%sERR,DRV_NFAULT,STOP\n", prefix);
            if (position_fault[i]) printf("%sERR,POSITION_FEEDBACK,STOP\n", prefix);
            if (speed_fault[i]) printf("%sERR,SPEED_FEEDBACK,STOP\n", prefix);
            if (fault[i] != CURRENT_UNINITIALIZED) printf("%sERR,CURRENT_SENSE,%d,STOP\n", prefix, (int)fault[i]);
        }

        ++slow_divider;
        if (g_telemetry_mode == TELEMETRY_LEGACY) {
            telemetry_snapshot(g_selected, &telemetry);
            processing_print_telemetry(&telemetry);
            if (slow_divider >= 10) {
                processing_print_speed(&telemetry);
                processing_print_position(&telemetry);
                slow_divider = 0;
            }
        } else {
            printf("T,%lu,%d", (unsigned long)(esp_timer_get_time() / 1000), g_selected + 1);
            for (int i = 0; i < MOTOR_COUNT; ++i) {
                telemetry_snapshot(i, &telemetry);
                multi_print_motor(&telemetry);
            }
            printf("\n");
            if (slow_divider >= 10) {
                telemetry_snapshot(config_motor, &telemetry);
                multi_print_config(config_motor, &telemetry);
                config_motor = (config_motor + 1) % MOTOR_COUNT;
                slow_divider = 0;
            }
        }
        fflush(stdout);
        vTaskDelayUntil(&telemetry_wake, pdMS_TO_TICKS(PROCESSING_PERIOD_MS));
    }
}
