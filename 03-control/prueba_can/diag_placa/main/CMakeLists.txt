idf_component_register(SRCS "diag_placa.c"
                       REQUIRES driver esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag
                                esp_timer nvs_flash)
