#include "encoder.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"

static pcnt_unit_handle_t unit;
static uint32_t previous;
static int64_t position;

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
    // Decode both edges of both channels: four counts per quadrature cycle.
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, -30000));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, 30000));
    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
}

// One caller only: the sampling task. Extend the driver's accumulated count.
int64_t encoder_read(void)
{
    int count;
    ESP_ERROR_CHECK(pcnt_unit_get_count(unit, &count));
    uint32_t next = (uint32_t)count;
    uint32_t wrapped_delta = next - previous;
    int64_t delta = wrapped_delta <= INT32_MAX ? (int64_t)wrapped_delta : (int64_t)wrapped_delta - 4294967296LL;
    position += ENCODER_SIGN * delta;
    previous = next;
    return position;
}
