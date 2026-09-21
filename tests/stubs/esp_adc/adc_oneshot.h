#pragma once
#include "esp_err.h"
typedef void *adc_oneshot_unit_handle_t;
#define ADC_UNIT_1 1
#define ADC_RTC_CLK_SRC_DEFAULT 0
#define ADC_ULP_MODE_DISABLE 0
#define ADC_CHANNEL_4 4
#define ADC_ATTEN_DB_12 12
#define ADC_BITWIDTH_12 12
typedef struct { int unit_id; int clk_src; int ulp_mode; } adc_oneshot_unit_init_cfg_t;
typedef struct { int atten; int bitwidth; } adc_oneshot_chan_cfg_t;
esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t *config,
                             adc_oneshot_unit_handle_t *handle);
esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t handle, int channel,
                                    const adc_oneshot_chan_cfg_t *config);
esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t handle, int channel, int *raw);
