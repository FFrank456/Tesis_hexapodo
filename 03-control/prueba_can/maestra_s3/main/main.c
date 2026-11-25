/*
 * Maestra ESP32-S3 DevKitC-1 para la prueba CAN.
 * Cableado al modulo CAN (SN65HVD230 / WCMCU-230):
 *   GPIO4 -> CTX/TXD    GPIO5 <- CRX/RXD    3V3 -> 3V3/VCC    GND -> GND
 * La consola va por el conector "USB" (USB nativo del S3), no por "UART".
 */
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_err.h"
#include "prueba_can.h"

void app_main(void)
{
    /* Con el driver instalado la consola tambien puede leer teclas. */
    usb_serial_jtag_driver_config_t usb = {.rx_buffer_size = 256, .tx_buffer_size = 4096};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    usb_serial_jtag_vfs_use_driver();

    static const prueba_can_cfg_t cfg = {
        .nombre = "ESP32-S3 maestra",
        .pin_tx = GPIO_NUM_4,
        .pin_rx = GPIO_NUM_5,
        .es_maestra = true,
        .pata_id = 1,   /* le habla a la pata 1: 0x101 / 0x201 */
    };
    prueba_can_ejecutar(&cfg);
}
