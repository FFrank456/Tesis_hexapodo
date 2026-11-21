#pragma once
/*
 * Protocolo CAN maestra <-> pata (v2). 500 kbit/s, ID estandar de 11 bits.
 * Lo comparten pata_MOD_DRV y prueba_can/comun (la maestra): cambiar en los dos.
 *
 *   0x0FF            maestra -> todas   PARO (dlc 0). Responde DETENIDA.
 *   0x100 + pata     maestra -> pata    orden (byte 0 = codigo de orden).
 *   0x200 + pata     pata -> maestra    respuesta (byte 0 = codigo de respuesta).
 *
 * Ordenes (0x100 + pata):
 *   dlc 0                       PING / latido.             -> PONG
 *   [01]                        ARMAR: la pata acepta IR_A. Exige las tres
 *                               articulaciones paradas.     -> ACEPTADA | RECHAZADA
 *   [02 mask c c f f t t]       IR_A: angulo ABSOLUTO (mrad, int16 little endian)
 *                               para las articulaciones de mask (bit0 coxa, bit1
 *                               femur, bit2 tibia). La pata hace la trayectoria
 *                               con su PLIMIT, gravedad y friccion. Se puede
 *                               cambiar el objetivo en pleno movimiento.
 *                                                           -> ACEPTADA | RECHAZADA | DESARMADA
 *   [03]                        ESTADO (tambien sirve de latido) -> ESTADO
 *   [04 mask]                   CERO: el encoder de esas articulaciones vale 0
 *                               en la pose actual. Solo desarmada y parada.
 *                                                           -> ACEPTADA | RECHAZADA
 *   [05]                        DESARMAR: suelta las articulaciones (como PARO,
 *                               pero solo esta pata).        -> DETENIDA
 *
 * Respuestas (0x200 + pata):
 *   [00] PONG   [01] ACEPTADA   [02] DESARMADA   [04] DETENIDA
 *   [03 motivo] RECHAZADA (motivo: LEG_CAN_REASON_*)
 *   [05 flags c c f f t t] ESTADO: posicion actual (mrad, int16 LE) y flags:
 *       bit0-2  la articulacion llego (POSITION en HOLD)
 *       bit3-5  nFAULT del driver
 *       bit6    pata armada
 *
 * Seguridad: armada, la pata se desarma (y suelta las articulaciones) si pasan
 * LEG_CAN_TIMEOUT_US sin ninguna orden de la maestra. La maestra manda ESTADO
 * cada ~100 ms mientras la pata este armada. Las ordenes serie de motor se
 * rechazan mientras este armada; STOP por serie la desarma.
 */

#define LEG_CAN_STOP_ID 0x0FF
#define LEG_CAN_COMMAND_ID(leg) (0x100 + (leg))
#define LEG_CAN_REPLY_ID(leg) (0x200 + (leg))
#define LEG_CAN_AXIS_MASK 0x07
#define LEG_CAN_TIMEOUT_US 500000

/* Codigos de orden (byte 0). */
enum {
    LEG_CAN_CMD_ARM = 1,
    LEG_CAN_CMD_GOTO = 2,
    LEG_CAN_CMD_STATUS = 3,
    LEG_CAN_CMD_ZERO = 4,
    LEG_CAN_CMD_DISARM = 5,
};
#define LEG_CAN_GOTO_DLC 8
#define LEG_CAN_STATUS_DLC 8

/* Codigos de respuesta (byte 0). */
enum {
    LEG_CAN_PONG = 0,
    LEG_CAN_ACCEPTED,
    LEG_CAN_DISARMED,
    LEG_CAN_REJECTED,
    LEG_CAN_STOPPED,
    LEG_CAN_STATUS,
};

/* Motivo de RECHAZADA (byte 1). */
enum {
    LEG_CAN_REASON_FORMAT = 1,     /* orden desconocida o largo incorrecto */
    LEG_CAN_REASON_RANGE = 2,      /* angulo fuera del rango de la articulacion */
    LEG_CAN_REASON_NFAULT = 3,     /* el driver de esa articulacion esta en falla */
    LEG_CAN_REASON_NOT_STOPPED = 4,/* hay que estar parada (ARMAR, CERO, arrancar) */
    LEG_CAN_REASON_ARMED = 5,      /* CERO o ARMAR con la pata ya armada */
    LEG_CAN_REASON_CONTROL = 6,    /* el control no arranco o perdio el encoder */
};

#define LEG_CAN_STATUS_ARRIVED_SHIFT 0
#define LEG_CAN_STATUS_NFAULT_SHIFT 3
#define LEG_CAN_STATUS_ARMED_BIT 0x40

/* Rango permitido por articulacion (mrad, respecto del cero del encoder).
 * Son los rangos por defecto del dashboard: coxa -0.5..1.5, femur -1..1,
 * tibia -1..0.7 rad. */
#define LEG_CAN_MIN_MRAD {-500, -1000, -1000}
#define LEG_CAN_MAX_MRAD {1500, 1000, 700}
