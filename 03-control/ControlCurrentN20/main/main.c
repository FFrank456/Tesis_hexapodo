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
#include "current_pi.h"
#include "speed_pi.h"
#include "position_pi.h"
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
 * - GPIO26 -> EN/IN1 del DRV8876: PWM para giro horario.
 * - GPIO27 -> PH/IN2 del DRV8876: PWM para giro antihorario.
 * - GPIO32 -> IPROPI del DRV8876 (ADC1_CH4): voltaje proporcional a corriente.
 * - GPIO5  -> nFAULT del DRV8876 (open-drain, activo en bajo).
 * - GPIO35/GPIO25 -> Encoder A/B.
 *
 * Estructura:
 * 1. MCPWM genera los dos PWM desde el mismo timer.
 * 2. Un comparador MCPWM adicional despierta la tarea cada diez periodos.
 * 3. El callback del comparador despierta la tarea adc_sample_task().
 * 4. DMA adquiere CS a 200 kS/s; current_sense_read() consulta una instantanea.
 *    encoder_sample() calcula posicion y velocidad de salida con su propio reloj.
 * 5. OPEN aplica PWM; PID regula CS; SPEED ejecuta PI velocidad -> PI CS.
 * 6. app_main() atiende comandos seriales y envía datos CSV a Processing.
 *
 * La corriente publicada para el sensado es la media de cada bloque DMA completo.
 */

#define PWM_GPIO_CW                         26  /* EN/IN1 */
#define PWM_GPIO_CCW                        27  /* PH/IN2 */
#define DRV_NFAULT_GPIO                     5   /* nFAULT, activo en bajo */

#define PWM_FREQUENCY_HZ                    10000U
#define PWM_PERIOD_TICKS                    SENSE_PWM_PERIOD_TICKS
#define PWM_MAX_DUTY                        PWM_PERIOD_TICKS
#define MCPWM_TIMER_RESOLUTION_HZ           SENSE_PWM_TIMER_HZ
#define MCPWM_GROUP_ID                      0

#define SNAPSHOT_RATE_HZ                  1000U
#define PROCESSING_RATE_HZ                  100U
#define PWM_CYCLES_PER_ADC_SAMPLE           (PWM_FREQUENCY_HZ / SNAPSHOT_RATE_HZ)
#define ADC_SAMPLING_COMPARE_TICKS          20U /* Wake-up at 1 us; NOT conversion phase. */
#define PROCESSING_PERIOD_MS                (1000U / PROCESSING_RATE_HZ)
#define COMMAND_BUFFER_SIZE                 512

/*
 * Entorno de control de corriente.
 *
 * El controlador trabaja con CS media escalada, pendiente de calibracion fisica:
 * - Entrada: error en A equivalentes, e = I_ref - I_medida.
 * - Salida: esfuerzo de control en porcentaje PWM, u = 0..100 %.
 * - Conversion final: duty_ticks = u * 2000 / 100.
 *
 * PI MATLAB libre: Kp=8.832, Ki=80090, Kd=0. Realimentacion: CS media DMA.
 * Arranque OPEN y referencia cero. Activacion explicita mediante MODE/CTRL.
 */
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
    CONTROL_MODE_SPEED,
    CONTROL_MODE_POSITION,
} control_mode_t;

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
} telemetry_t;

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
static bool g_pending_speed_fault;
static bool g_pending_position_fault;
static bool g_pending_drv_fault;
static bool g_position_reference_set;
static uint32_t g_position_reference_id;
static position_pi_t g_position={.kp=POSITION_KP_INITIAL,.ki=POSITION_KI_INITIAL,
    .max_speed_rad_s=2,.accel_rad_s2=5,.tolerance_rad=0.02f,.direction=1};
static speed_pi_t g_speed = {.kp=SPEED_KP_INITIAL, .ki=SPEED_KI_INITIAL,
    .current_limit_a=SPEED_CURRENT_LIMIT_INITIAL_A};

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
    return mode == CONTROL_MODE_POSITION ? "POSITION" : mode == CONTROL_MODE_SPEED ? "SPEED" : mode == CONTROL_MODE_PID ? "PID" : "OPEN";
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

static void control_set_mode(control_mode_t mode)
{
    portENTER_CRITICAL(&g_state_lock);
    g_sequence.active = false;
    g_sequence.index = -1;
    g_control_mode = mode;
    position_pi_reset(&g_position);
    speed_pi_reset(&g_speed);
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

/* Caller owns g_motor_lock. Same-direction manual output is tracked on entry.
 * Opposite-direction activation requires STOP first. Zero reference always coasts. */
static bool control_start_pid(motor_direction_t direction, float reference_a)
{
    if ((direction != MOTOR_CW && direction != MOTOR_CCW) || !pi_reference_valid(reference_a)) {
        printf("ERR,PI,expected_CW_or_CCW_and_reference_0_to_1A_eq\n");
        return false;
    }
    if (g_direction != MOTOR_STOP && g_direction != direction) {
        printf("ERR,PI,STOP_before_reversing\n");
        return false;
    }
    current_sample_t current = current_sense_read(g_duty_cw+g_duty_ccw,
                                                 0, 0, g_output_changed_us);
    if (!current_sample_usable(current.status) || current.dma_block_samples != SENSE_DMA_FRAME_SAMPLES ||
        !isfinite(current.current_mean_a)) {
        control_stop();
        printf("ERR,PI,valid_DMA_feedback_required\n");
        return false;
    }
    float tracking = fabsf(g_duty_percent);
    bool was_running = g_direction != MOTOR_STOP;
    portENTER_CRITICAL(&g_state_lock);
    g_sequence.active = false;
    g_sequence.index = -1;
    g_control_mode = CONTROL_MODE_PID;
    position_pi_reset(&g_position);
    speed_pi_reset(&g_speed);
    g_control_direction = direction;
    pid_reset(&g_pid);
    g_pid.current_ref_a = reference_a;
    g_pid.last_sample_us = current.sample_time_us;
    g_pid.error_a = reference_a-current.current_mean_a;
    if (was_running && reference_a > 0) {
        g_pid.output_percent = clamp_float(tracking, g_pid.output_min_percent, g_pid.output_max_percent);
        g_pid.integrator = g_pid.output_percent-g_pid.kp*g_pid.error_a;
    }
    portEXIT_CRITICAL(&g_state_lock);
    if (reference_a == 0) motor_set_output(MOTOR_STOP, 0);
    return true;
}

/* Enter only after both feedback paths and a coast-compatible direction check. */
static bool control_start_speed(motor_direction_t direction)
{
    encoder_sample_t enc=encoder_sample();
    if (!enc.speed_valid || !isfinite(enc.output_speed_rad_s)) {
        control_stop();
        printf("ERR,SPEED,valid_encoder_required\n");
        return false;
    }
    float sign=direction==MOTOR_CCW ? -1.0f : 1.0f;
    if (sign*enc.output_speed_rad_s < -SPEED_REVERSAL_MAX_RAD_S) {
        printf("ERR,SPEED,wait_for_coast_before_reversing\n");
        return false;
    }
    if (!control_start_pid(direction,0)) return false;
    portENTER_CRITICAL(&g_state_lock);
    g_control_mode=CONTROL_MODE_SPEED;
    speed_pi_reset(&g_speed);
    g_speed.last_update_us=enc.time_us;
    portEXIT_CRITICAL(&g_state_lock);
    return true;
}

/* Position starts only from a stopped, explicitly targeted axis. */
static bool control_start_position(void)
{
    encoder_sample_t enc=encoder_sample();
    if(g_control_mode!=CONTROL_MODE_OPEN || g_direction!=MOTOR_STOP) {
        printf("ERR,POSITION,STOP_before_activation\n");return false;
    }
    if(!g_position_reference_set || enc.reference_id!=g_position_reference_id) {
        printf("ERR,POSITION,set_PREF_after_encoder_zero\n");return false;
    }
    if(!enc.speed_valid || !isfinite(enc.output_position_rad) || !isfinite(enc.output_speed_rad_s) ||
       fabs(enc.output_speed_rad_s)>POSITION_STOP_RAD_S) {
        printf("ERR,POSITION,valid_stopped_encoder_required\n");return false;
    }
    motor_direction_t dir=g_position.reference_rad>=enc.output_position_rad ? MOTOR_CW : MOTOR_CCW;
    if(!control_start_pid(dir,0))return false;
    portENTER_CRITICAL(&g_state_lock);
    g_control_mode=CONTROL_MODE_POSITION;
    g_position.direction=dir==MOTOR_CW ? 1 : -1;
    position_pi_reset(&g_position);g_position.last_update_us=enc.time_us;
    g_speed.reference_rad_s=0;g_speed.last_update_us=enc.time_us;
    portEXIT_CRITICAL(&g_state_lock);
    return true;
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
    int64_t rate_time_us = 0;
    float observed_pwm_hz = 0, observed_attempt_hz = 0, observed_adc_hz = 0;

    (void)arg;

    while (true) {
        uint32_t notifications = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
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
        /* Encoder timing belongs to the PCNT read, not the ADC DMA block.
         * g_motor_lock also serializes reference changes from UART commands. */
        encoder_sample_t encoder = encoder_sample();
        const bool valid = current_sample_usable(current.status) &&
            current.dma_block_samples == SENSE_DMA_FRAME_SAMPLES && isfinite(current.current_mean_a);
        adc_sample_count=current.total_samples;

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
        bool drv_fault = gpio_get_level(DRV_NFAULT_GPIO) == 0;
        if (drv_fault) {
            stop_for_sensing = true;
            g_pending_drv_fault = true;
        }
        motor_direction_t pid_direction = MOTOR_STOP;
        float pid_output_percent = 0.0f;

        portENTER_CRITICAL(&g_state_lock);
        if(g_control_mode==CONTROL_MODE_POSITION) {
            if(!encoder.speed_valid || !isfinite(encoder.output_position_rad) ||
               !isfinite(encoder.output_speed_rad_s) || encoder.reference_id!=g_position_reference_id ||
               encoder.time_us<=0 || now_us-encoder.time_us>=SPEED_MAX_GAP_US ||
               (g_position.last_update_us && encoder.time_us-g_position.last_update_us>=POSITION_MAX_GAP_US)) {
                stop_for_sensing=true;g_pending_position_fault=true;
            } else if(!stop_for_sensing && valid && encoder.time_us-g_position.last_update_us>=POSITION_PERIOD_US) {
                bool downstream=g_speed.limited ||
                    (g_pid.output_percent>=g_pid.output_max_percent-0.01f && g_pid.error_a>0);
                int previous_direction=g_position.direction;
                if(!position_pi_update(&g_position,encoder.output_position_rad,encoder.output_speed_rad_s,
                                       encoder.time_us,downstream)) {
                    stop_for_sensing=true;g_pending_position_fault=true;
                } else {
                    if(previous_direction!=g_position.direction || g_position.output_rad_s==0) {
                        speed_pi_reset(&g_speed);pid_reset(&g_pid);g_pid.current_ref_a=0;
                        g_speed.last_update_us=encoder.time_us;
                    }
                    g_control_direction=g_position.direction>0 ? MOTOR_CW : MOTOR_CCW;
                    g_speed.reference_rad_s=fabsf(g_position.output_rad_s);
                }
            }
        }
        if (g_control_mode == CONTROL_MODE_SPEED || g_control_mode==CONTROL_MODE_POSITION) {
            if (!encoder.speed_valid || !isfinite(encoder.output_speed_rad_s) ||
                encoder.time_us <= 0 || now_us-encoder.time_us >= SPEED_MAX_GAP_US ||
                (g_speed.last_update_us && encoder.time_us-g_speed.last_update_us >= SPEED_MAX_GAP_US)) {
                stop_for_sensing=true;
                g_pending_speed_fault=true;
            } else if (!stop_for_sensing && valid) {
                if (g_speed.reference_rad_s == 0) {
                    speed_pi_reset(&g_speed);
                    g_speed.last_update_us=encoder.time_us;
                    g_pid.current_ref_a=0;
                } else if (encoder.time_us-g_speed.last_update_us >= SPEED_PERIOD_US) {
                    float projected=(g_control_direction==MOTOR_CCW ? -1 : 1)*encoder.output_speed_rad_s;
                    bool inner_high=g_pid.output_percent >= g_pid.output_max_percent-0.01f && g_pid.error_a>0;
                    if (speed_pi_update(&g_speed,projected,encoder.time_us,inner_high))
                        g_pid.current_ref_a=g_speed.output_a;
                    else { stop_for_sensing=true; g_pending_speed_fault=true; }
                }
            }
        }
        if (g_control_mode != CONTROL_MODE_OPEN) {
            if (!valid || g_control_direction == MOTOR_STOP ||
                (g_direction != MOTOR_STOP && g_control_direction != g_direction) ||
                (g_pid.last_sample_us && now_us-g_pid.last_sample_us >= SENSE_PID_LOSS_TIMEOUT_US))
                stop_for_sensing = true;
            if (!stop_for_sensing && g_pid.current_ref_a == 0) {
                pid_reset(&g_pid);
                g_pid.last_sample_us = sample_time_us;
                pid_direction = MOTOR_STOP;
                apply_pid_output = true;
            } else if (!stop_for_sensing && sample_time_us > g_pid.last_sample_us) {
                if (pi_update_sample(&g_pid, current.current_mean_a, sample_time_us)) {
                    pid_output_percent = g_pid.output_percent;
                    pid_direction = g_control_direction;
                    apply_pid_output = true;
                } else stop_for_sensing = true;
            }
        } else {
            g_pid.error_a = valid ? g_pid.current_ref_a - current.current_mean_a : NAN;
        }

        g_telemetry.time_ms = (uint32_t)(sample_time_us / 1000ULL);
        g_telemetry.sample_time_us = sample_time_us;
        g_telemetry.encoder_count = encoder.count_x4;
        g_telemetry.encoder = encoder;
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
        g_telemetry.speed = g_speed;
        g_telemetry.position = g_position;
        g_telemetry.control_direction_sign = g_control_direction==MOTOR_CCW ? -1 : 1;
        g_telemetry.direction = g_direction;
        g_telemetry.duty_cw = g_duty_cw;
        g_telemetry.duty_ccw = g_duty_ccw;
        g_telemetry.duty_percent = g_duty_percent;
        portEXIT_CRITICAL(&g_state_lock);

        if (stop_for_sensing) {
            /* Emit ERR from the UART task, never block this task on printf. */
            if (!drv_fault && !g_pending_speed_fault && !g_pending_position_fault && (duty_ticks > 0 || g_control_mode != CONTROL_MODE_OPEN))
                g_pending_current_fault = valid ? CURRENT_OUTPUT_SETTLING : current.status;
            control_stop();
        } else if (apply_pid_output) {
            motor_set_output_percent(pid_direction, pid_output_percent);
        }
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
    if (strcmp(text,"POSITION")==0) { *mode=CONTROL_MODE_POSITION; return true; }
    if (strcmp(text, "SPEED") == 0) { *mode=CONTROL_MODE_SPEED; return true; }
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
        /* Each experiment starts with a firmware-owned position reference.
         * Re-zeroing does not reset raw counts or the velocity history. */
        encoder_zero_reference();
        sequence_start(&g_sequence, esp_timer_get_time());
        printf("ACK,SEQ,%lu,%d\n", (unsigned long)g_sequence.id, g_sequence.count);
        return;
    }

    if (strcmp(command, "ENCODER") == 0) {
        char *action = strtok(NULL, ",");
        if (action == NULL || strcmp(action, "ZERO") != 0 || strtok(NULL, ",") != NULL) {
            printf("ERR,ENCODER,expected_ZERO\n");
        } else if (g_sequence.active || g_control_mode!=CONTROL_MODE_OPEN || g_direction!=MOTOR_STOP) {
            printf("ERR,ENCODER,STOP_before_zero\n");
        } else {
            encoder_zero_reference();
            g_position_reference_set=false;
            printf("ACK,ENCODER,ZERO\n");
        }
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

        if(g_control_mode==CONTROL_MODE_POSITION && mode!=CONTROL_MODE_OPEN) {
            printf("ERR,MODE,STOP_before_leaving_or_restarting_POSITION\n");return;
        }
        if(mode==CONTROL_MODE_POSITION) {
            if(!control_start_position())return;
        } else if (mode == CONTROL_MODE_SPEED) {
            if (!control_start_speed(requested_direction)) return;
        } else if (mode == CONTROL_MODE_PID) {
            if (!control_start_pid(requested_direction, g_control_mode==CONTROL_MODE_SPEED ? 0 : g_pid.current_ref_a)) return;
        } else {
            if(g_control_mode==CONTROL_MODE_POSITION)control_stop();
            else control_set_mode(mode);
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

        if (g_control_mode != CONTROL_MODE_OPEN && direction != g_control_direction) {
            printf("ERR,DIR,STOP_before_changing_PI_direction\n");
            return;
        }
        portENTER_CRITICAL(&g_state_lock);
        g_control_direction = direction;
        portEXIT_CRITICAL(&g_state_lock);
        printf("ACK,DIR,%s\n", motor_direction_to_text(direction));
        return;
    }

    if (strcmp(command, "IREF") == 0) {
        if (g_control_mode==CONTROL_MODE_SPEED || g_control_mode==CONTROL_MODE_POSITION) { printf("ERR,IREF,reference_owned_by_outer_loop\n"); return; }
        char *current_text = strtok(NULL, ",");
        if (current_text == NULL) {
            return;
        }

        float current_ref_a;
        if (!control_parse_float(current_text, &current_ref_a) || !pi_reference_valid(current_ref_a)) {
            printf("ERR,IREF,expected_0_to_1A_eq\n");
            return;
        }

        portENTER_CRITICAL(&g_state_lock);
        g_pid.current_ref_a = current_ref_a;
        if (current_ref_a == 0) pid_reset(&g_pid);
        portEXIT_CRITICAL(&g_state_lock);
        if (current_ref_a == 0 && g_control_mode == CONTROL_MODE_PID) motor_set_output(MOTOR_STOP, 0);
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

        if (!control_parse_float(kp_text, &kp) || !control_parse_float(ki_text, &ki) ||
            !control_parse_float(kd_text, &kd) || kp < 0 || ki < 0 || kd != 0 || strtok(NULL, ",")) {
            printf("ERR,PID,PI_requires_finite_Kp_Ki_nonnegative_Kd_zero\n");
            return;
        }

        portENTER_CRITICAL(&g_state_lock);
        g_pid.kp = kp;
        g_pid.ki = ki;
        g_pid.kd = kd;
        if (g_control_mode != CONTROL_MODE_OPEN) {
            g_pid.integrator = g_pid.output_percent-kp*g_pid.error_a;
        } else pid_reset(&g_pid);
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

        float min_percent, max_percent;
        if (!control_parse_float(min_text, &min_percent) || !control_parse_float(max_text, &max_percent) ||
            min_percent < 0 || max_percent > 100 || max_percent < min_percent || strtok(NULL, ",")) {
            printf("ERR,LIMIT,expected_0_le_min_le_max_le_100\n");
            return;
        }
        portENTER_CRITICAL(&g_state_lock);
        g_pid.output_min_percent = min_percent;
        g_pid.output_max_percent = max_percent;
        g_pid.output_percent = clamp_float(g_pid.output_percent, min_percent, max_percent);
        if (g_control_mode != CONTROL_MODE_OPEN)
            g_pid.integrator = g_pid.output_percent-g_pid.kp*g_pid.error_a;
        portEXIT_CRITICAL(&g_state_lock);
        printf("ACK,LIMIT,%.3f,%.3f\n", min_percent, max_percent);
        return;
    }

    if(strcmp(command,"PREF")==0) {
        char *value=strtok(NULL,",");char *end=NULL;
        double ref=value ? strtod(value,&end) : NAN;
        if(!value || end==value || *end || !position_reference_valid(ref) || strtok(NULL,",")) {
            printf("ERR,PREF,expected_rad_within_10000\n");return;
        }
        encoder_sample_t enc=encoder_sample();
        portENTER_CRITICAL(&g_state_lock);
        g_position.reference_rad=ref;g_position_reference_id=enc.reference_id;g_position_reference_set=true;
        position_pi_reset(&g_position);g_position.last_update_us=enc.time_us;
        if(g_control_mode==CONTROL_MODE_POSITION) {
            speed_pi_reset(&g_speed);g_speed.reference_rad_s=0;g_speed.last_update_us=enc.time_us;
            pid_reset(&g_pid);g_pid.current_ref_a=0;
        }
        portEXIT_CRITICAL(&g_state_lock);
        if(g_control_mode==CONTROL_MODE_POSITION)motor_set_output(MOTOR_STOP,0);
        printf("ACK,PREF,%.6f\n",ref);return;
    }
    if(strcmp(command,"PPI")==0 || strcmp(command,"PLIMIT")==0) {
        if(g_control_mode==CONTROL_MODE_POSITION) {printf("ERR,POSITION,STOP_before_parameter_change\n");return;}
        bool gains=strcmp(command,"PPI")==0;
        float a,b,c=0;char *ta=strtok(NULL,","),*tb=strtok(NULL,","),*tc=gains ? NULL : strtok(NULL,",");
        bool ok=control_parse_float(ta,&a) && control_parse_float(tb,&b);
        if(!gains)ok=ok && control_parse_float(tc,&c);
        ok=ok && !strtok(NULL,",");
        if(!ok || (gains ? (a<0 || b<0) : (a<=0 || a>32 || b<=0 || b>100 || c<0.0023f || c>1))) {
            printf("ERR,POSITION,invalid_gains_or_limits\n");return;
        }
        portENTER_CRITICAL(&g_state_lock);
        if(gains){g_position.kp=a;g_position.ki=b;}
        else {g_position.max_speed_rad_s=a;g_position.accel_rad_s2=b;g_position.tolerance_rad=c;}
        position_pi_reset(&g_position);
        portEXIT_CRITICAL(&g_state_lock);
        printf("ACK,%s,%.6f,%.6f,%.6f\n",command,a,b,c);return;
    }

    if (strcmp(command,"WREF")==0) {
        if(g_control_mode==CONTROL_MODE_POSITION) {printf("ERR,WREF,use_PREF_in_POSITION\n");return;}
        float ref; char *value=strtok(NULL,",");
        if (!control_parse_float(value,&ref) || !speed_reference_valid(ref) || strtok(NULL,",")) {
            printf("ERR,WREF,expected_0_to_32_rad_s\n"); return;
        }
        portENTER_CRITICAL(&g_state_lock);
        g_speed.reference_rad_s=ref;
        if (ref==0) {
            speed_pi_reset(&g_speed);
            if (g_control_mode==CONTROL_MODE_SPEED) { g_pid.current_ref_a=0; pid_reset(&g_pid); }
        }
        portEXIT_CRITICAL(&g_state_lock);
        if (ref==0 && g_control_mode==CONTROL_MODE_SPEED) motor_set_output(MOTOR_STOP,0);
        printf("ACK,WREF,%.4f\n",ref); return;
    }
    if (strcmp(command,"SPI")==0) {
        float kp,ki; char *a=strtok(NULL,","), *b=strtok(NULL,",");
        if (!control_parse_float(a,&kp) || !control_parse_float(b,&ki) || kp<0 || ki<0 || strtok(NULL,",")) {
            printf("ERR,SPI,expected_finite_nonnegative_Kp_Ki\n"); return;
        }
        portENTER_CRITICAL(&g_state_lock);
        g_speed.kp=kp; g_speed.ki=ki;
        if (g_control_mode==CONTROL_MODE_SPEED || g_control_mode==CONTROL_MODE_POSITION) g_speed.integral_a=g_speed.output_a-kp*g_speed.error_rad_s;
        else speed_pi_reset(&g_speed);
        portEXIT_CRITICAL(&g_state_lock);
        printf("ACK,SPI,%.6f,%.6f\n",kp,ki); return;
    }
    if (strcmp(command,"ILIMIT")==0) {
        float cap; char *value=strtok(NULL,",");
        if (!control_parse_float(value,&cap) || cap<=0 || cap>PI_REFERENCE_MAX_A || strtok(NULL,",")) {
            printf("ERR,ILIMIT,expected_0_lt_A_eq_le_1\n"); return;
        }
        portENTER_CRITICAL(&g_state_lock);
        bool clamped=g_speed.output_a>cap;
        g_speed.current_limit_a=cap;
        g_speed.output_a=fminf(cap,g_speed.output_a);
        if (g_control_mode==CONTROL_MODE_SPEED || g_control_mode==CONTROL_MODE_POSITION) {
            g_pid.current_ref_a=fminf(cap,g_pid.current_ref_a);
            if (clamped) g_speed.integral_a=g_speed.output_a-g_speed.kp*g_speed.error_rad_s;
        }
        portEXIT_CRITICAL(&g_state_lock);
        printf("ACK,ILIMIT,%.4f\n",cap); return;
    }

    if (strcmp(command, "CTRL") == 0) {
        if(g_control_mode==CONTROL_MODE_POSITION) {printf("ERR,CTRL,STOP_before_leaving_POSITION\n");return;}
        char *direction_text = strtok(NULL, ",");
        char *current_text = strtok(NULL, ",");
        motor_direction_t direction = MOTOR_STOP;

        if (direction_text == NULL || current_text == NULL || !parse_direction(direction_text, &direction)) {
            return;
        }

        float current_ref_a;
        if (!control_parse_float(current_text, &current_ref_a) || !pi_reference_valid(current_ref_a)) {
            printf("ERR,IREF,expected_0_to_1A_eq\n");
            return;
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
           (unsigned long)SENSE_CONTINUOUS_RATE_HZ,
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
    fflush(stdout);
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

    gpio_config_t nfault_config = {
        .pin_bit_mask = 1ULL << DRV_NFAULT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&nfault_config));

    mcpwm_init();
    motor_set_output(MOTOR_STOP, 0);

    processing_print_header();

    TickType_t telemetry_wake = xTaskGetTickCount();
    unsigned speed_telemetry_divider=0;
    while (true) {
        serial_command_poll();
        xSemaphoreTake(g_motor_lock, portMAX_DELAY);
        bool drv_fault=g_pending_drv_fault;g_pending_drv_fault=false;
        bool position_fault=g_pending_position_fault;g_pending_position_fault=false;
        bool speed_fault=g_pending_speed_fault;
        g_pending_speed_fault=false;
        current_status_t fault = g_pending_current_fault;
        g_pending_current_fault = CURRENT_UNINITIALIZED;
        xSemaphoreGive(g_motor_lock);
        if(drv_fault) printf("ERR,DRV_NFAULT,STOP\n");
        if(position_fault) printf("ERR,POSITION_FEEDBACK,STOP\n");
        if (speed_fault) printf("ERR,SPEED_FEEDBACK,STOP\n");
        if (fault != CURRENT_UNINITIALIZED) printf("ERR,CURRENT_SENSE,%d,STOP\n", (int)fault);
        telemetry_t telemetry = telemetry_snapshot();
        processing_print_telemetry(&telemetry);
        if (++speed_telemetry_divider>=10) { processing_print_speed(&telemetry); processing_print_position(&telemetry); speed_telemetry_divider=0; }
        vTaskDelayUntil(&telemetry_wake, pdMS_TO_TICKS(PROCESSING_PERIOD_MS));
    }
}
