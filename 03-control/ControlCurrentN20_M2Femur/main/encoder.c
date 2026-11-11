#include "encoder.h"
#include "encoder_processing.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"

static pcnt_unit_handle_t unit;
static encoder_processing_t processing;

void encoder_init(void)
{
    pcnt_unit_config_t config = {
        .low_limit = -30000, .high_limit = 30000,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&config, &unit));
    pcnt_glitch_filter_config_t filter = {.max_glitch_ns = 1000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter));
    pcnt_channel_handle_t a, b;
    pcnt_chan_config_t channel = {.edge_gpio_num = ENCODER_GPIO_A, .level_gpio_num = ENCODER_GPIO_B};
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &channel, &a));
    channel.edge_gpio_num = ENCODER_GPIO_B;
    channel.level_gpio_num = ENCODER_GPIO_A;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &channel, &b));
#if ENCODER_USE_INTERNAL_PULLUP
    // Igual que INPUT_PULLUP del sketch validado (D13 aislado con pull-up).
    ESP_ERROR_CHECK(gpio_pullup_en(ENCODER_GPIO_A));
    ESP_ERROR_CHECK(gpio_pullup_en(ENCODER_GPIO_B));
#endif
    // Decode both edges of both channels: four counts per quadrature cycle.
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, -30000));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, 30000));
    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    processing = (encoder_processing_t){0};
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
}

// Read the PCNT accumulated counter. All callers hold the shared motor lock,
// including reference-zero commands, to keep extension and history serialized.
static uint32_t encoder_raw_count(void)
{
    int count;
    ESP_ERROR_CHECK(pcnt_unit_get_count(unit, &count));
    return (uint32_t)count;
}

int64_t encoder_read(void)
{
    return encoder_processing_extend(&processing, encoder_raw_count());
}

encoder_sample_t encoder_sample(void)
{
    const uint32_t raw = encoder_raw_count();
    const int64_t now_us = esp_timer_get_time();
    return encoder_processing_sample(&processing, raw, now_us);
}

void encoder_zero_reference(void)
{
    encoder_processing_zero(&processing, encoder_raw_count());
}
