#include "encoder.h"
#include "encoder_processing.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"

static const int k_pin_a[ENCODER_COUNT] = MOTOR_PIN_ENC_A;
static const int k_pin_b[ENCODER_COUNT] = MOTOR_PIN_ENC_B;
static const int k_sign_initial[ENCODER_COUNT] = MOTOR_ENC_SIGN;

typedef struct {
    pcnt_unit_handle_t unit;
    encoder_processing_t processing;
    int sign;
} encoder_t;

static encoder_t encoders[ENCODER_COUNT];

static void encoder_init_one(encoder_t *e, int pin_a, int pin_b)
{
    pcnt_unit_config_t config = {
        .low_limit = -30000, .high_limit = 30000,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&config, &e->unit));
    pcnt_glitch_filter_config_t filter = {.max_glitch_ns = 1000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(e->unit, &filter));
    pcnt_channel_handle_t a, b;
    pcnt_chan_config_t channel = {.edge_gpio_num = pin_a, .level_gpio_num = pin_b};
    ESP_ERROR_CHECK(pcnt_new_channel(e->unit, &channel, &a));
    channel.edge_gpio_num = pin_b;
    channel.level_gpio_num = pin_a;
    ESP_ERROR_CHECK(pcnt_new_channel(e->unit, &channel, &b));
#if ENCODER_USE_INTERNAL_PULLUP
    // Igual que INPUT_PULLUP del sketch validado del companero.
    ESP_ERROR_CHECK(gpio_pullup_en(pin_a));
    ESP_ERROR_CHECK(gpio_pullup_en(pin_b));
#endif
    // Decode both edges of both channels: four counts per quadrature cycle.
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(e->unit, -30000));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(e->unit, 30000));
    ESP_ERROR_CHECK(pcnt_unit_enable(e->unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(e->unit));
    e->processing = (encoder_processing_t){0};
    ESP_ERROR_CHECK(pcnt_unit_start(e->unit));
}

void encoder_init(void)
{
    for (int m = 0; m < ENCODER_COUNT; ++m) {
        encoders[m].sign = k_sign_initial[m] < 0 ? -1 : 1;
        encoder_init_one(&encoders[m], k_pin_a[m], k_pin_b[m]);
    }
}

// Read the PCNT accumulated counter with the motor's sign applied. Negating
// modulo 2^32 keeps the wrap-extension in encoder_processing valid.
static uint32_t encoder_raw_count(const encoder_t *e)
{
    int count;
    ESP_ERROR_CHECK(pcnt_unit_get_count(e->unit, &count));
    uint32_t raw = (uint32_t)count;
    return e->sign < 0 ? 0U - raw : raw;
}

static encoder_t *encoder_at(int motor)
{
    return (motor >= 0 && motor < ENCODER_COUNT) ? &encoders[motor] : &encoders[0];
}

encoder_sample_t encoder_sample(int motor)
{
    encoder_t *e = encoder_at(motor);
    const uint32_t raw = encoder_raw_count(e);
    const int64_t now_us = esp_timer_get_time();
    return encoder_processing_sample(&e->processing, raw, now_us);
}

void encoder_zero_reference(int motor)
{
    encoder_t *e = encoder_at(motor);
    encoder_processing_zero(&e->processing, encoder_raw_count(e));
}

void encoder_set_sign(int motor, int sign)
{
    encoder_t *e = encoder_at(motor);
    const uint32_t next_id = e->processing.reference_id + 1U;
    e->sign = sign < 0 ? -1 : 1;
    e->processing = (encoder_processing_t){0};
    e->processing.previous_raw = encoder_raw_count(e);
    e->processing.reference_id = next_id;
}

int encoder_get_sign(int motor)
{
    return encoder_at(motor)->sign;
}
