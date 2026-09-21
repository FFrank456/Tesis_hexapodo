#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include "sdkconfig.h"
#include "encoder.h"
#include "pwm_sequence.h"
#include "current_sense.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

/*
 * ControlCurrentN20 - MCPWM 10 kHz y media ADC DMA continua de 200 muestras
 *
 * Hardware:
 * - GPIO27 -> IN1/PH del DRV8876: PWM para giro horario.
 * - GPIO26 -> IN2/EN del DRV8876: PWM para giro antihorario.
 * - GPIO32 -> CS/IPROPI del DRV8876: voltaje proporcional a corriente.
 *
 * Estructura:
 * 1. MCPWM genera los dos PWM desde el mismo timer.
 * 2. Un comparador MCPWM adicional despierta la tarea cada diez periodos.
 * 3. El callback del comparador despierta la tarea adc_sample_task().
 * 4. DMA adquiere CS a 200 kS/s; current_sense_read() consulta una instantanea.
 * 5. OPEN aplica PWM; activar PID requiere validar otro modelo de realimentacion.
 * 6. app_main() atiende comandos seriales y envía datos CSV a Processing.
 *
 * Se aplica un filtro EMA a la corriente para observar una señal más útil en control.
 */

#define PWM_GPIO_CW                         27
#define PWM_GPIO_CCW                        26

#define PWM_FREQUENCY_HZ                    10000U
#define PWM_PERIOD_TICKS                    SENSE_PWM_PERIOD_TICKS
#define PWM_MAX_DUTY                        PWM_PERIOD_TICKS
#define MCPWM_TIMER_RESOLUTION_HZ           SENSE_PWM_TIMER_HZ
#define MCPWM_GROUP_ID                      0

#define CURRENT_EMA_ALPHA                   SENSE_EMA_NOMINAL_ALPHA

#define SNAPSHOT_RATE_HZ                  1000U
#define PROCESSING_RATE_HZ                  100U
#define PWM_CYCLES_PER_ADC_SAMPLE           (PWM_FREQUENCY_HZ / SNAPSHOT_RATE_HZ)
#define ADC_SAMPLING_COMPARE_TICKS          20U /* Wake-up at 1 us; NOT conversion phase. */
#define PROCESSING_PERIOD_MS                (1000U / PROCESSING_RATE_HZ)
#define COMMAND_BUFFER_SIZE                 512

/*
 * Entorno de control de corriente.
 *
 * El controlador trabaja en unidades fisicas:
 * - Entrada: error de corriente en amperios, e = I_ref - I_medida.
 * - Salida: esfuerzo de control en porcentaje PWM, u = 0..100 %.
 * - Conversion final: duty_ticks = u * 2000 / 100.
 *
 * Los valores iniciales vienen de la aproximacion en MATLAB compartida:
 *   FT_current = (0.3336*s + 0.4566)/(s^2 + 233.3*s + 786)
 *   C(s) = 8320/s
 *
 * Eso equivale a un controlador integral puro con Ki = 8320. Es un punto
 * inicial para experimentar, no una ganancia definitiva. El ESP32 arranca en
 * modo OPEN y con referencia de 0 A para evitar movimiento inesperado.
 */
#define CONTROL_LOOP_HZ                     SNAPSHOT_RATE_HZ
#define CONTROL_TS_SEC                      (1.0f / (float)CONTROL_LOOP_HZ)
#define PID_KP_INITIAL                      0.0f
#define PID_KI_INITIAL                      8320.0f
#define PID_KD_INITIAL                      0.0f
#define PID_OUTPUT_MIN_PERCENT              0.0f
#define PID_OUTPUT_MAX_PERCENT              100.0f
#define CURRENT_REF_INITIAL_A               0.0f

_Static_assert(PWM_FREQUENCY_HZ % SNAPSHOT_RATE_HZ == 0,
               "SNAPSHOT_RATE_HZ must divide PWM_FREQUENCY_HZ");
_Static_assert(MCPWM_TIMER_RESOLUTION_HZ == PWM_FREQUENCY_HZ * PWM_PERIOD_TICKS,
               "PWM period must agree with the realizable timer resolution");

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_CW,
    MOTOR_CCW,
} motor_direction_t;

typedef enum {
    CONTROL_MODE_OPEN = 0,
    CONTROL_MODE_PID,
} control_mode_t;

typedef struct {
    uint32_t time_ms;
    int64_t sample_time_us;
    int64_t encoder_count;
    uint32_t sequence_id;
    int sequence_index;
    int64_t sequence_start_us;
    int64_t sequence_stage_start_us;
    uint32_t adc_sample_count;
    int adc_raw;
    float adc_voltage;
    bool adc_calibrated;
    float current_raw_a;
    float current_ema_a;
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
} telemetry_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float previous_error;
    float output_min_percent;
    float output_max_percent;
    float current_ref_a;
    float output_percent;
    float error_a;
    bool previous_error_valid;
} pid_controller_t;

typedef struct {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t pwm_operator;
    mcpwm_oper_handle_t sample_operator;
    mcpwm_cmpr_handle_t comparator_cw;
    mcpwm_cmpr_handle_t comparator_ccw;
    mcpwm_cmpr_handle_t comparator_adc_sample;
    mcpwm_gen_handle_t generator_cw;
    mcpwm_gen_handle_t generator_ccw;
} motor_pwm_t;

static SemaphoreHandle_t g_motor_lock;
static pwm_sequence_t g_sequence = {.index = -1};
static uint32_t g_sequence_id = 0;
static motor_pwm_t g_pwm = {0};
static TaskHandle_t g_adc_task_handle = NULL;
static portMUX_TYPE g_state_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE g_clock_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t g_pwm_cycle_count;
static int64_t g_output_changed_us;
static current_status_t g_pending_current_fault;

static telemetry_t g_telemetry = {0};
static motor_direction_t g_direction = MOTOR_STOP;
static uint32_t g_duty_cw = 0;
static uint32_t g_duty_ccw = 0;
static float g_duty_percent = 0.0f;
static control_mode_t g_control_mode = CONTROL_MODE_OPEN;
static motor_direction_t g_control_direction = MOTOR_CW;
static pid_controller_t g_pid = {
    .kp = PID_KP_INITIAL,
    .ki = PID_KI_INITIAL,
    .kd = PID_KD_INITIAL,
    .integrator = 0.0f,
    .previous_error = 0.0f,
    .output_min_percent = PID_OUTPUT_MIN_PERCENT,
    .output_max_percent = PID_OUTPUT_MAX_PERCENT,
    .current_ref_a = CURRENT_REF_INITIAL_A,
    .output_percent = 0.0f,
    .error_a = 0.0f,
    .previous_error_valid = false,
};

static const char *motor_direction_to_text(motor_direction_t direction)
{
    switch (direction) {
    case MOTOR_CW:
        return "CW";
    case MOTOR_CCW:
        return "CCW";
    case MOTOR_STOP:
    default:
        return "STOP";
    }
}

static const char *control_mode_to_text(control_mode_t mode)
{
    return mode == CONTROL_MODE_PID ? "PID" : "OPEN";
}

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
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

static void pid_reset(pid_controller_t *pid)
{
    pid->integrator = 0.0f;
    pid->previous_error = 0.0f;
    pid->output_percent = 0.0f;
    pid->error_a = 0.0f;
    pid->previous_error_valid = false;
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

static void motor_set_output(motor_direction_t direction, uint32_t duty)
{
    uint32_t duty_cw = 0;
    uint32_t duty_ccw = 0;

    duty = clamp_duty(duty);
    if (duty == 0) direction = MOTOR_STOP;
    if (direction == MOTOR_CW) {
        duty_cw = duty;
    } else if (direction == MOTOR_CCW) {
        duty_ccw = duty;
    } else {
        direction = MOTOR_STOP;
    }

    if (duty_cw == g_duty_cw && duty_ccw == g_duty_ccw) return;
    /* Break before reversing. Same-direction updates use the shadow comparator
     * without inserting a low glitch on every PID iteration. */
    if (direction != g_direction) {
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(g_pwm.generator_cw, 0, true));
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(g_pwm.generator_ccw, 0, true));
    }
    if (duty_cw != g_duty_cw)
        mcpwm_set_generator_for_duty(g_pwm.generator_cw, g_pwm.comparator_cw, duty_cw);
    if (duty_ccw != g_duty_ccw)
        mcpwm_set_generator_for_duty(g_pwm.generator_ccw, g_pwm.comparator_ccw, duty_ccw);
    g_output_changed_us = esp_timer_get_time();

    portENTER_CRITICAL(&g_state_lock);
    g_direction = direction;
    g_duty_cw = duty_cw;
    g_duty_ccw = duty_ccw;
    g_duty_percent = duty_to_percent(duty_cw) - duty_to_percent(duty_ccw);
    portEXIT_CRITICAL(&g_state_lock);
}

static void motor_set_output_percent(motor_direction_t direction, float percent)
{
    motor_set_output(direction, percent_to_duty(percent));
}

static float pid_update(pid_controller_t *pid, float current_feedback_a, float dt_sec)
{
    float error = pid->current_ref_a - current_feedback_a;
    float derivative = 0.0f;

    if (pid->previous_error_valid) {
        derivative = (error - pid->previous_error) / dt_sec;
    } else {
        pid->previous_error_valid = true;
    }

    float proportional = pid->kp * error;
    float integral_candidate = pid->integrator + (pid->ki * error * dt_sec);
    float derivative_term = pid->kd * derivative;
    float unclamped = proportional + integral_candidate + derivative_term;
    float clamped = clamp_float(unclamped, pid->output_min_percent, pid->output_max_percent);

    /*
     * Anti-windup por integracion condicional:
     * si la salida satura y el error empuja mas hacia la saturacion, no se
     * acepta el nuevo integrador. Si el error ayuda a salir de saturacion, si.
     */
    bool saturated_high = unclamped > pid->output_max_percent;
    bool saturated_low = unclamped < pid->output_min_percent;
    bool drives_further_high = saturated_high && error > 0.0f;
    bool drives_further_low = saturated_low && error < 0.0f;
    if (!drives_further_high && !drives_further_low) {
        pid->integrator = integral_candidate;
    }

    pid->previous_error = error;
    pid->error_a = error;
    pid->output_percent = clamped;
    return clamped;
}

static void control_set_mode(control_mode_t mode)
{
    portENTER_CRITICAL(&g_state_lock);
    g_sequence.active = false;
    g_sequence.index = -1;
    g_control_mode = mode;
    if (mode == CONTROL_MODE_OPEN) {
        pid_reset(&g_pid);
    }
    portEXIT_CRITICAL(&g_state_lock);
}

static void control_stop(void)
{
    control_set_mode(CONTROL_MODE_OPEN);
    motor_set_output(MOTOR_STOP, 0);
}

/* Caller owns g_motor_lock. Starting blindly from zero would let the integral
 * drive up PWM while all feedback is unobservable. Require a measured manual
 * operating point in the same direction instead. */
static bool control_start_pid(motor_direction_t direction, float reference_a)
{
    (void)direction; (void)reference_a;
    printf("ERR,PID,ON_current_requires_calibration_and_controller_tuning\n");
    return false;
}

static bool adc_sample_compare_callback(mcpwm_cmpr_handle_t comparator,
                                        const mcpwm_compare_event_data_t *edata,
                                        void *user_ctx)
{
    static uint32_t pwm_cycle_count = 0;
    BaseType_t high_task_woken = pdFALSE;

    (void)comparator;
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

static void mcpwm_init(void)
{
    mcpwm_timer_config_t timer_config = {0};
    timer_config.group_id = MCPWM_GROUP_ID;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_PLL160M;
    timer_config.resolution_hz = MCPWM_TIMER_RESOLUTION_HZ;
    timer_config.period_ticks = PWM_PERIOD_TICKS;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &g_pwm.timer));

    mcpwm_operator_config_t pwm_operator_config = {0};
    pwm_operator_config.group_id = MCPWM_GROUP_ID;
    pwm_operator_config.flags.update_gen_action_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_operator(&pwm_operator_config, &g_pwm.pwm_operator));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(g_pwm.pwm_operator, g_pwm.timer));

    mcpwm_operator_config_t sample_operator_config = {0};
    sample_operator_config.group_id = MCPWM_GROUP_ID;
    ESP_ERROR_CHECK(mcpwm_new_operator(&sample_operator_config, &g_pwm.sample_operator));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(g_pwm.sample_operator, g_pwm.timer));

    mcpwm_comparator_config_t pwm_comparator_config = {0};
    pwm_comparator_config.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(g_pwm.pwm_operator, &pwm_comparator_config, &g_pwm.comparator_cw));
    ESP_ERROR_CHECK(mcpwm_new_comparator(g_pwm.pwm_operator, &pwm_comparator_config, &g_pwm.comparator_ccw));

    mcpwm_comparator_config_t sample_comparator_config = {0};
    ESP_ERROR_CHECK(mcpwm_new_comparator(g_pwm.sample_operator, &sample_comparator_config,
                                         &g_pwm.comparator_adc_sample));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(g_pwm.comparator_adc_sample,
                                                       ADC_SAMPLING_COMPARE_TICKS));

    mcpwm_generator_config_t generator_cw_config = {0};
    generator_cw_config.gen_gpio_num = PWM_GPIO_CW;
    ESP_ERROR_CHECK(mcpwm_new_generator(g_pwm.pwm_operator, &generator_cw_config, &g_pwm.generator_cw));

    mcpwm_generator_config_t generator_ccw_config = {0};
    generator_ccw_config.gen_gpio_num = PWM_GPIO_CCW;
    ESP_ERROR_CHECK(mcpwm_new_generator(g_pwm.pwm_operator, &generator_ccw_config, &g_pwm.generator_ccw));
    ESP_ERROR_CHECK(gpio_input_enable(PWM_GPIO_CW));
    ESP_ERROR_CHECK(gpio_input_enable(PWM_GPIO_CCW));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(g_pwm.comparator_cw, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(g_pwm.comparator_ccw, 0));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        g_pwm.generator_cw,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        g_pwm.generator_cw,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       g_pwm.comparator_cw,
                                       MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        g_pwm.generator_ccw,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        g_pwm.generator_ccw,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       g_pwm.comparator_ccw,
                                       MCPWM_GEN_ACTION_LOW)));

    mcpwm_comparator_event_callbacks_t sample_callbacks = {0};
    sample_callbacks.on_reach = adc_sample_compare_callback;
    ESP_ERROR_CHECK(mcpwm_comparator_register_event_callbacks(g_pwm.comparator_adc_sample,
                                                              &sample_callbacks,
                                                              NULL));

    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(g_pwm.generator_cw, 0, true));
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(g_pwm.generator_ccw, 0, true));
    ESP_ERROR_CHECK(mcpwm_timer_enable(g_pwm.timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(g_pwm.timer, MCPWM_TIMER_START_NO_STOP));
}

static void adc_sample_task(void *arg)
{
    uint32_t adc_sample_count = 0;
    uint32_t attempt_count = 0, missed_count = 0;
    uint32_t previous_dma_overflows=0;
    uint32_t rate_cycles = 0, rate_attempts = 0, rate_adc = 0;
    int64_t rate_time_us = 0, previous_sample_us = 0, last_valid_us = 0;
    float current_ema_a = 0.0f;
    bool previous_valid = false;
    float observed_pwm_hz = 0, observed_attempt_hz = 0, observed_adc_hz = 0;

    (void)arg;

    while (true) {
        uint32_t notifications = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (notifications > 1) missed_count += notifications - 1;
        attempt_count++;

        xSemaphoreTake(g_motor_lock, portMAX_DELAY);
        int64_t now_us = esp_timer_get_time();
        float sequence_pwm;
        if (sequence_tick(&g_sequence, now_us, &sequence_pwm)) {
            motor_direction_t direction = sequence_pwm > 0 ? MOTOR_CW : sequence_pwm < 0 ? MOTOR_CCW : MOTOR_STOP;
            motor_set_output_percent(direction, fabsf(sequence_pwm));
        }
        uint32_t duty_ticks = g_duty_cw + g_duty_ccw;
        current_sample_t current = current_sense_read(duty_ticks,
            g_duty_ccw ? PWM_GPIO_CCW : PWM_GPIO_CW,
            g_duty_ccw ? PWM_GPIO_CW : PWM_GPIO_CCW, g_output_changed_us);
        int64_t sample_time_us = current.sample_time_us;
        int64_t encoder_count = encoder_read();
        const bool valid = current_sample_usable(current.status);
        const bool feedback_timed_out = last_valid_us != 0 &&
            sample_time_us - last_valid_us >= SENSE_PID_LOSS_TIMEOUT_US;
        adc_sample_count=current.total_samples;
        if (valid) {
            current_ema_a=current.ema_a;
            last_valid_us=sample_time_us;
        }

        /* Counts/rates are independent of UART decimation. This diagnoses lost
         * task notifications and skipped/invalid conversions, not just CSV rows. */
        portENTER_CRITICAL(&g_clock_lock);
        uint32_t cycles = g_pwm_cycle_count;
        int64_t clock_time_us = esp_timer_get_time();
        portEXIT_CRITICAL(&g_clock_lock);
        if (!rate_time_us) {
            rate_time_us = clock_time_us;
            rate_cycles = cycles; rate_attempts = attempt_count; rate_adc = adc_sample_count;
        } else if (clock_time_us - rate_time_us >= 1000000) {
            float seconds = (clock_time_us - rate_time_us) / 1000000.0f;
            observed_pwm_hz = (uint32_t)(cycles - rate_cycles) / seconds;
            observed_attempt_hz = (uint32_t)(attempt_count - rate_attempts) / seconds;
            observed_adc_hz = (uint32_t)(adc_sample_count - rate_adc) / seconds;
            rate_time_us = clock_time_us;
            rate_cycles = cycles; rate_attempts = attempt_count; rate_adc = adc_sample_count;
        }

        bool apply_pid_output = false;
        bool stop_for_sensing = current.status == CURRENT_ADC_ERROR || current.status == CURRENT_DMA_OVERFLOW;
        stop_for_sensing |= current.overflows != previous_dma_overflows;
        previous_dma_overflows=current.overflows;
        motor_direction_t pid_direction = MOTOR_STOP;
        float pid_output_percent = 0.0f;

        portENTER_CRITICAL(&g_state_lock);
        if (g_control_mode == CONTROL_MODE_PID) {
            if (feedback_timed_out) stop_for_sensing = true;
            if (g_control_direction == MOTOR_STOP || g_control_direction != g_direction)
                stop_for_sensing = true;
            if (valid && !stop_for_sensing) {
                /* A gap freezes the integrator; never integrate the missing
                 * feedback interval or differentiate across it. */
                int64_t elapsed_us = sample_time_us - previous_sample_us;
                bool consecutive = previous_valid && notifications == 1 &&
                                   elapsed_us > 0 && elapsed_us <= 2 * SENSE_NOMINAL_INTERVAL_US;
                float dt = consecutive ? elapsed_us / 1000000.0f : CONTROL_TS_SEC;
                if (!consecutive) g_pid.previous_error_valid = false;
                pid_output_percent = pid_update(&g_pid, current.current_mean_a, dt);
                pid_direction = g_control_direction;
                apply_pid_output = true;
            } else {
                g_pid.previous_error_valid = false;
                g_pid.error_a = NAN;
                if (sample_time_us - last_valid_us >= SENSE_PID_LOSS_TIMEOUT_US)
                    stop_for_sensing = true;
            }
        } else {
            g_pid.error_a = valid ? g_pid.current_ref_a - current.current_mean_a : NAN;
        }

        g_telemetry.time_ms = (uint32_t)(sample_time_us / 1000ULL);
        g_telemetry.sample_time_us = sample_time_us;
        g_telemetry.encoder_count = encoder_count;
        g_telemetry.sequence_id = g_sequence.id;
        g_telemetry.sequence_index = g_sequence.index;
        g_telemetry.sequence_start_us = g_sequence.start_us;
        g_telemetry.sequence_stage_start_us = g_sequence.stage_start_us;
        g_telemetry.adc_sample_count = adc_sample_count;
        g_telemetry.adc_raw_mean = current.adc_raw_mean;
        g_telemetry.adc_voltage_mean = current.adc_voltage_mean;
        g_telemetry.current_mean_a = current.current_mean_a;
        g_telemetry.dma_block_samples = current.dma_block_samples;
        g_telemetry.adc_raw = current.raw;
        g_telemetry.adc_voltage = current.voltage;
        g_telemetry.adc_calibrated = current.calibrated;
        g_telemetry.current_raw_a = current.current_a;
        g_telemetry.current_ema_a = valid ? current_ema_a : NAN;
        g_telemetry.current_status = current.status;
        g_telemetry.adc_read_us = current.read_us;
        g_telemetry.dma_overflows = current.overflows;
        g_telemetry.sample_phase_us = current.phase_us;
        g_telemetry.current_valid_count = current.accepted_samples;
        g_telemetry.rejected_window = current.rejected_window;
        g_telemetry.rejected_quality = current.rejected_quality;
        g_telemetry.sample_attempt_count = attempt_count;
        g_telemetry.sample_missed_count = missed_count;
        g_telemetry.pwm_frequency_observed_hz = observed_pwm_hz;
        g_telemetry.acquisition_rate_observed_hz = observed_attempt_hz;
        g_telemetry.adc_rate_observed_hz = observed_adc_hz;
        g_telemetry.current_ref_a = g_pid.current_ref_a;
        g_telemetry.current_error_a = g_pid.error_a;
        g_telemetry.control_output_percent = g_pid.output_percent;
        g_telemetry.pid_kp = g_pid.kp;
        g_telemetry.pid_ki = g_pid.ki;
        g_telemetry.pid_kd = g_pid.kd;
        g_telemetry.pid_integrator = g_pid.integrator;
        g_telemetry.control_mode = g_control_mode;
        g_telemetry.direction = g_direction;
        g_telemetry.duty_cw = g_duty_cw;
        g_telemetry.duty_ccw = g_duty_ccw;
        g_telemetry.duty_percent = g_duty_percent;
        portEXIT_CRITICAL(&g_state_lock);

        if (stop_for_sensing) {
            /* Emit ERR from the UART task, never block this task on printf. */
            if (duty_ticks > 0 || g_control_mode == CONTROL_MODE_PID)
                g_pending_current_fault = current.status == CURRENT_VALID_ON ? CURRENT_NO_WINDOW : current.status;
            control_stop();
        } else if (apply_pid_output) {
            motor_set_output_percent(pid_direction, pid_output_percent);
        }
        previous_sample_us = sample_time_us;
        previous_valid = valid;
        xSemaphoreGive(g_motor_lock);
    }
}

static bool parse_direction(const char *text, motor_direction_t *direction)
{
    if (strcmp(text, "STOP") == 0) {
        *direction = MOTOR_STOP;
        return true;
    }
    if (strcmp(text, "CW") == 0) {
        *direction = MOTOR_CW;
        return true;
    }
    if (strcmp(text, "CCW") == 0) {
        *direction = MOTOR_CCW;
        return true;
    }
    return false;
}

static bool parse_control_mode(const char *text, control_mode_t *mode)
{
    if (strcmp(text, "OPEN") == 0) {
        *mode = CONTROL_MODE_OPEN;
        return true;
    }
    if (strcmp(text, "PID") == 0) {
        *mode = CONTROL_MODE_PID;
        return true;
    }
    return false;
}

static void handle_serial_command(char *line)
{
    char *command = strtok(line, ",");

    if (command == NULL) {
        return;
    }

    if (strcmp(command, "SEQ") == 0) {
        pwm_sequence_t candidate;
        if (!sequence_parse(strtok(NULL, ""), &candidate)) {
            printf("ERR,SEQ,percent:ms;percent:ms,max24,total120s,min100ms\n");
            return;
        }
        control_stop();
        g_sequence = candidate;
        g_sequence.id = ++g_sequence_id;
        sequence_start(&g_sequence, esp_timer_get_time());
        printf("ACK,SEQ,%lu,%d\n", (unsigned long)g_sequence.id, g_sequence.count);
        return;
    }

    if (strcmp(command, "STOP") == 0) {
        control_stop();
        printf("ACK,STOP,0.000,0\n");
        return;
    }

    if (strcmp(command, "SET") == 0) {
        char *pwm_text = strtok(NULL, "");
        char *end = NULL;
        float pwm_percent = pwm_text ? strtof(pwm_text, &end) : NAN;
        if (!pwm_text || end == pwm_text || *end != '\0' ||
            !isfinite(pwm_percent) || pwm_percent < -100.0f || pwm_percent > 100.0f) {
            printf("ERR,SET,expected_percent_-100_to_100\n");
            return;
        }
        motor_direction_t direction = pwm_percent > 0 ? MOTOR_CW :
                                      pwm_percent < 0 ? MOTOR_CCW : MOTOR_STOP;
        control_set_mode(CONTROL_MODE_OPEN);
        motor_set_output_percent(direction, fabsf(pwm_percent));
        printf("ACK,SET,%.3f\n", pwm_percent);
        return;
    }

    if (strcmp(command, "MODE") == 0) {
        char *mode_text = strtok(NULL, ",");
        char *direction_text = strtok(NULL, ",");
        control_mode_t mode = CONTROL_MODE_OPEN;
        motor_direction_t requested_direction = g_control_direction;

        if (mode_text == NULL || !parse_control_mode(mode_text, &mode)) {
            return;
        }
        if (direction_text != NULL) {
            if (!parse_direction(direction_text, &requested_direction)) return;
        }

        if (mode == CONTROL_MODE_PID) {
            if (!control_start_pid(requested_direction, g_pid.current_ref_a)) return;
        } else {
            control_set_mode(mode);
            g_control_direction = requested_direction;
        }
        printf("ACK,MODE,%s,%s\n", control_mode_to_text(mode), motor_direction_to_text(g_control_direction));
        return;
    }

    if (strcmp(command, "DIR") == 0) {
        char *direction_text = strtok(NULL, ",");
        motor_direction_t direction = MOTOR_STOP;

        if (direction_text == NULL || !parse_direction(direction_text, &direction)) {
            return;
        }

        portENTER_CRITICAL(&g_state_lock);
        g_control_direction = direction;
        portEXIT_CRITICAL(&g_state_lock);
        printf("ACK,DIR,%s\n", motor_direction_to_text(direction));
        return;
    }

    if (strcmp(command, "IREF") == 0) {
        char *current_text = strtok(NULL, ",");
        if (current_text == NULL) {
            return;
        }

        float current_ref_a = strtof(current_text, NULL);
        if (current_ref_a < 0.0f) {
            current_ref_a = 0.0f;
        }

        portENTER_CRITICAL(&g_state_lock);
        g_pid.current_ref_a = current_ref_a;
        portEXIT_CRITICAL(&g_state_lock);
        printf("ACK,IREF,%.4f\n", current_ref_a);
        return;
    }

    if (strcmp(command, "PID") == 0) {
        char *kp_text = strtok(NULL, ",");
        char *ki_text = strtok(NULL, ",");
        char *kd_text = strtok(NULL, ",");
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;

        if (kp_text == NULL || ki_text == NULL || kd_text == NULL) {
            return;
        }

        kp = strtof(kp_text, NULL);
        ki = strtof(ki_text, NULL);
        kd = strtof(kd_text, NULL);

        portENTER_CRITICAL(&g_state_lock);
        g_pid.kp = kp;
        g_pid.ki = ki;
        g_pid.kd = kd;
        pid_reset(&g_pid);
        portEXIT_CRITICAL(&g_state_lock);
        printf("ACK,PID,%.4f,%.4f,%.4f\n", kp, ki, kd);
        return;
    }

    if (strcmp(command, "LIMIT") == 0) {
        char *min_text = strtok(NULL, ",");
        char *max_text = strtok(NULL, ",");

        if (min_text == NULL || max_text == NULL) {
            return;
        }

        float min_percent = clamp_float(strtof(min_text, NULL), 0.0f, 100.0f);
        float max_percent = clamp_float(strtof(max_text, NULL), min_percent, 100.0f);
        portENTER_CRITICAL(&g_state_lock);
        g_pid.output_min_percent = min_percent;
        g_pid.output_max_percent = max_percent;
        g_pid.output_percent = clamp_float(g_pid.output_percent, min_percent, max_percent);
        g_pid.integrator = clamp_float(g_pid.integrator, min_percent, max_percent);
        portEXIT_CRITICAL(&g_state_lock);
        printf("ACK,LIMIT,%.3f,%.3f\n", min_percent, max_percent);
        return;
    }

    if (strcmp(command, "CTRL") == 0) {
        char *direction_text = strtok(NULL, ",");
        char *current_text = strtok(NULL, ",");
        motor_direction_t direction = MOTOR_STOP;

        if (direction_text == NULL || current_text == NULL || !parse_direction(direction_text, &direction)) {
            return;
        }

        float current_ref_a = strtof(current_text, NULL);
        if (current_ref_a < 0.0f) {
            current_ref_a = 0.0f;
        }

        if (!control_start_pid(direction, current_ref_a)) return;
        printf("ACK,CTRL,%s,%.4f\n", motor_direction_to_text(direction), current_ref_a);
        return;
    }
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

static telemetry_t telemetry_snapshot(void)
{
    telemetry_t snapshot;

    portENTER_CRITICAL(&g_state_lock);
    snapshot = g_telemetry;
    portEXIT_CRITICAL(&g_state_lock);

    return snapshot;
}

static void processing_print_header(void)
{
    printf("HEADER,time_ms,adc_raw,adc_voltage,current_raw_a,current_ema_a,direction_code,direction_text,"
           "duty_cw_percent,duty_ccw_percent,adc_sample_count,pwm_frequency_hz,adc_sample_rate_hz,"
           "adc_sample_tick,current_ema_alpha,control_mode_code,control_mode_text,current_ref_a,"
           "current_error_a,control_output_percent,duty_percent,pid_kp,pid_ki,pid_kd,pid_integrator,"
           "adc_calibrated,sample_time_us,encoder_count,sequence_id,sequence_index,sequence_start_us,sequence_stage_start_us,"
           "current_valid,current_status,adc_read_us,sample_phase_us,current_valid_count,sample_attempt_count,"
           "sample_missed_count,pwm_frequency_observed_hz,acquisition_rate_observed_hz,adc_rate_observed_hz,dma_overflows,adc_voltage_ema,"
           "sensing_version,dma_rejected_window,dma_rejected_quality,dma_phase_validated,"
           "adc_raw_mean,adc_voltage_mean,current_mean_a,dma_block_samples\n");
}

static void processing_print_telemetry(const telemetry_t *telemetry)
{
    printf("DATA,%lu,%d,%.3f,%.3f,%.3f,%d,%s,%.3f,%.3f,%lu,%lu,%lu,%lu,%.7f,%d,%s,"
           "%.4f,%.4f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%d,%lld,%lld,%lu,%d,%lld,%lld,"
           "%d,%d,%lu,%ld,%lu,%lu,%lu,%.1f,%.1f,%.1f,%lu,%.5f,%u,%lu,%lu,%d,%.3f,%.6f,%.6f,%lu\n",
           (unsigned long)telemetry->time_ms,
           telemetry->adc_raw,
           telemetry->adc_voltage,
           telemetry->current_raw_a,
           telemetry->current_ema_a,
           (int)telemetry->direction,
           motor_direction_to_text(telemetry->direction),
           duty_to_percent(telemetry->duty_cw),
           duty_to_percent(telemetry->duty_ccw),
           (unsigned long)telemetry->adc_sample_count,
           (unsigned long)PWM_FREQUENCY_HZ,
           (unsigned long)SENSE_CONTINUOUS_RATE_HZ,
           (unsigned long)0,
           SENSE_CONTINUOUS_ALPHA,
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
           telemetry->current_ema_a * 2.49f,
           (unsigned)SENSE_DATA_VERSION,
           (unsigned long)telemetry->rejected_window,
           (unsigned long)telemetry->rejected_quality,
           0,
           telemetry->adc_raw_mean,
           telemetry->adc_voltage_mean,
           telemetry->current_mean_a,
           (unsigned long)telemetry->dma_block_samples);
    fflush(stdout);
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);

    g_motor_lock = xSemaphoreCreateMutex();
    configASSERT(g_motor_lock != NULL);
    encoder_init();
    current_sense_init();
    serial_command_init();

    BaseType_t created = xTaskCreatePinnedToCore(adc_sample_task,
                            "adc_sample_task",
                            4096,
                            NULL,
                            10,
                            &g_adc_task_handle,
                            0);
    configASSERT(created == pdPASS);

    mcpwm_init();
    motor_set_output(MOTOR_STOP, 0);

    processing_print_header();

    TickType_t telemetry_wake = xTaskGetTickCount();
    while (true) {
        serial_command_poll();
        xSemaphoreTake(g_motor_lock, portMAX_DELAY);
        current_status_t fault = g_pending_current_fault;
        g_pending_current_fault = CURRENT_UNINITIALIZED;
        xSemaphoreGive(g_motor_lock);
        if (fault != CURRENT_UNINITIALIZED) printf("ERR,CURRENT_SENSE,%d,STOP\n", (int)fault);
        telemetry_t telemetry = telemetry_snapshot();
        processing_print_telemetry(&telemetry);
        vTaskDelayUntil(&telemetry_wake, pdMS_TO_TICKS(PROCESSING_PERIOD_MS));
    }
}
