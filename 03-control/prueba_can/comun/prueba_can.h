#pragma once
/*
 * Prueba CAN paso a paso entre la maestra ESP32-S3 y una MOD_DRV.
 * El mismo codigo corre en las dos placas; solo cambia esta configuracion.
 */
#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"

typedef struct {
    const char *nombre;   /* texto que se imprime al arrancar */
    gpio_num_t pin_tx;    /* GPIO del ESP que va a CTX/TXD del transceptor */
    gpio_num_t pin_rx;    /* GPIO del ESP que viene de CRX/RXD del transceptor */
    bool es_maestra;      /* true en el ESP32-S3 */
    uint8_t pata_id;      /* 1..6: la pata responde a 0x100+id; la maestra le habla a ese id */
} prueba_can_cfg_t;

/* Arranca en modo ESCUCHA y atiende los comandos del monitor serie. No retorna. */
void prueba_can_ejecutar(const prueba_can_cfg_t *cfg);
