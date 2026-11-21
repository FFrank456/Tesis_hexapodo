idf_component_register(SRCS "main.c" "current_sense.c" "encoder.c" "pwm_sequence.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_adc esp_driver_gpio esp_driver_mcpwm esp_driver_pcnt esp_timer)
