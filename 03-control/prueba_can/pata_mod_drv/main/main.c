/*
 * Pata MOD_DRV (ESP32-WROOM-32E) para la prueba CAN.
 * El modulo WCMCU-230 va en el zocalo J4: IO21 -> CTX, IO22 <- CRX.
 * El bus sale por P1/P2: pin 1 CANH, pin 2 CANL, pin 3 GND.
 * Consola por el USB-C (CP2102N) a 115200.
 */
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_err.h"
#include "prueba_can.h"

void app_main(void)
{
    /* Las 6 entradas de los DRV8876 a 0: los motores no se mueven en esta prueba. */
    static const gpio_num_t motores[] = {26, 27, 18, 19, 16, 2};
    for (unsigned i = 0; i < sizeof(motores) / sizeof(motores[0]); i++) {
        ESP_ERROR_CHECK(gpio_set_level(motores[i], 0));
        ESP_ERROR_CHECK(gpio_set_direction(motores[i], GPIO_MODE_OUTPUT));
    }

    /* Con el driver instalado la consola tambien puede leer teclas. */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    uart_vfs_dev_use_driver(UART_NUM_0);

    static const prueba_can_cfg_t cfg = {
        .nombre = "MOD_DRV pata 1",
        .pin_tx = GPIO_NUM_21,
        .pin_rx = GPIO_NUM_22,
        .es_maestra = false,
        .pata_id = 1,   /* igual que CAN_LEG_ID en pata_MOD_DRV/main/pin_map.h */
    };
    prueba_can_ejecutar(&cfg);
}
