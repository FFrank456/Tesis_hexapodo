#pragma once
#include "esp_adc/adc_cali.h"
typedef struct { int unit_id; int atten; int bitwidth; int default_vref; } adc_cali_line_fitting_config_t;
esp_err_t adc_cali_create_scheme_line_fitting(const adc_cali_line_fitting_config_t *config,
                                             adc_cali_handle_t *handle);
