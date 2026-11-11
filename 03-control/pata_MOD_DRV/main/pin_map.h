#pragma once
/*
 * Mapa de pines - pata_MOD_DRV (placa propia MOD_DRV / Moduloa_DRV, Altium)
 * Pata completa: Coxa, Femur, Tibia simultaneos.
 *
 * Fuente: 02-electronica/notas/"Modulo de control de extremidad - Registro del
 * proyecto.md" sec. 4.4, extraida de la netlist compilada de Moduloa_DRV.PrjPcb.
 *
 * | Senal            | DRVA = M1 Coxa | DRVB = M2 Femur | DRVC = M3 Tibia |
 * |------------------|----------------|-----------------|-----------------|
 * | IN1 (PWM)        | IO26           | IO18            | IO16            |
 * | IN2 (PWM)        | IO27           | IO19            | IO2  (arranque) |
 * | IPROPI (R1 3 k)  | IO32 ADC1_CH4  | IO33 ADC1_CH5   | IO34 ADC1_CH6   |
 * | nFAULT (R4 10 k) | IO5            | IO14            | IO15 (arranque) |
 * | Encoder J3 pin 4 | IO35 (sin PU)  | IO23            | IO4             |
 * | Encoder J3 pin 6 | IO25           | IO17            | IO13            |
 * | Conector         | J3A            | J3B             | J3C             |
 *
 * PMODE, nSLEEP y VREF a 3V3 en la placa (modo PWM, siempre activo).
 * IO35 es solo entrada y no tiene pull-up interno: el encoder de J3A debe
 * tener salida push-pull o una resistencia de pull-up externa.
 * IO21/IO22 quedan para el modulo CAN (J4); este firmware no los usa.
 *
 * ASUNCION: DRVA = Coxa, DRVB = Femur, DRVC = Tibia. Si en la pata se cablea
 * distinto, basta con reordenar las columnas de estos arreglos.
 * Los arreglos se indexan 0 = Coxa, 1 = Femur, 2 = Tibia. Por serial los
 * motores se nombran M1/M2/M3 o COXA/FEMUR/TIBIA.
 */

#define BOARD_NAME "pata_MOD_DRV"
#define MOTOR_COUNT 3

/* Motor que reciben los comandos sin prefijo y la telemetria LEGACY al
 * arrancar (0 = Coxa, 1 = Femur, 2 = Tibia). Se cambia en caliente con SEL,1|2|3,
 * pero abrir el puerto suele reiniciar el ESP32 y volver a este valor. */
#define DEFAULT_SELECTED_MOTOR 1

#define MOTOR_NAMES             {"COXA", "FEMUR", "TIBIA"}
#define MOTOR_PIN_IN1           {26, 18, 16}   /* EN/IN1 -> sentido "CW"  */
#define MOTOR_PIN_IN2           {27, 19, 2}    /* PH/IN2 -> sentido "CCW" */
#define MOTOR_PIN_CS            {32, 33, 34}
#define MOTOR_ADC_CHANNELS      {ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6}
#define MOTOR_PIN_ENC_A         {35, 23, 4}    /* J3 pin 4 */
#define MOTOR_PIN_ENC_B         {25, 17, 13}   /* J3 pin 6 */
/* Signo inicial del encoder por motor. Verificar: con PWM positivo (IN1) la
 * posicion debe crecer. Se puede cambiar en caliente con Mx,ESIGN,-1 (en STOP). */
#define MOTOR_ENC_SIGN          {1, 1, 1}

/* Pull-up interno en los pines de encoder que lo tienen (IO35 no: se omite). */
#define ENCODER_USE_INTERNAL_PULLUP 1

/* nFAULT de cada DRV8876 (activo en bajo, pull-up R4 de 10 k en la placa).
 * Un nivel bajo sostenido detiene SOLO ese motor y se reporta Mx,ERR,DRV_NFAULT. */
#define DRV_NFAULT_ENABLED       1
#define MOTOR_PIN_NFAULT        {5, 14, 15}
/* Muestras consecutivas de 1 ms en bajo antes de detener (filtra glitches). */
#define DRV_NFAULT_FILTER_MS     3

/*
 * IPROPI de la MOD_DRV: R1 = 3 kOhm, A_IPROPI = 1000 uA/A -> 3.0 V/A.
 * I_TRIP = VREF / (R * A) = 3.3 / 3 = 1.1 A (corte del driver).
 * El ADC con 12 dB llega a ~3.1 V -> ~1.03 A medibles.
 */
#define CS_R_IPROPI_OHMS         3000.0f
#define CS_A_IPROPI_A_PER_A      1.0e-3f
