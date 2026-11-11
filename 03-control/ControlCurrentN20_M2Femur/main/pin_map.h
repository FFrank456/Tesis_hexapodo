#pragma once
/*
 * Mapa de pines - ControlCurrentN20_M2Femur
 *
 * Fuente: repo flavioe-alarcon/hexapodo-secundario-firmware, AGENTS.md y
 * CONTEXTO_TESIS.md sec. 8.1 (pata individual, ESP32 DevKit V1 + DRV8876 MJKDZ),
 * sketch validado pruebas/motores_encoders/sketch_test_motor2_final.ino.
 *
 * Motor 2 (Femur):
 *   IN1 (EN/IN1, PWM sentido A) -> GPIO18 (D18)
 *   IN2 (PH/IN2, PWM sentido B) -> GPIO17 (D17)
 *   CS / IPROPI                 -> GPIO39 (VN) = ADC1_CH3 (solo entrada)
 *   Encoder C1 (A)              -> GPIO25 (D25)
 *   Encoder C2 (B)              -> GPIO13 (D13) (migrado de D26 por diafonia; pull-up interno)
 *   nFAULT                      -> sin conectar (como en las pruebas del companero)
 *   SLEEP y PMODE               -> puenteados a 3.3 V en el modulo (modo PWM/PWM)
 *
 * Proyecto original (ControlCurrentN20): IN1=26, IN2=27, CS=32 (ADC1_CH4),
 * nFAULT=5, encoder A/B=35/25.
 */

/* Puente H (MCPWM) */
#define PIN_MOTOR_IN1            18   /* EN/IN1 -> giro "CW" del firmware */
#define PIN_MOTOR_IN2            17   /* PH/IN2 -> giro "CCW" del firmware */

/* Sensado de corriente CS/IPROPI (ADC1, compatible con Wi-Fi) */
#define PIN_MOTOR_CS             39   /* VN */
#define SENSE_ADC_CHANNEL        ADC_CHANNEL_3   /* GPIO39 = ADC1_CH3 */

/* Encoder de cuadratura (PCNT) */
#define PIN_ENCODER_A            25
#define PIN_ENCODER_B            13
#define ENCODER_USE_INTERNAL_PULLUP 1  /* GPIO25 y GPIO13 si tienen pull-up interno */

/* nFAULT del DRV8876: deshabilitado (no cableado en el modulo de pruebas).
 * Para habilitarlo: poner 1 y definir un pin libre (p. ej. 19 como entrada).
 * GPIO5 NO: en el mapa del companero es IN2 del Motor 1. */
#define DRV_NFAULT_ENABLED       0
#define PIN_DRV_NFAULT           (-1)

/*
 * Resistencia IPROPI del modulo DRV8876 (clon MJKDZ del carrier Pololu).
 * Pololu #4036/#4037: CS a GND con 2.49 kOhm -> ~2.49 V/A (A_IPROPI = 1000 uA/A).
 * VERIFICAR midiendo CS-GND con el modulo desenergizado. Si mide 3 kOhm, usar 3000.
 * Con VREF atado a SLEEP (3.3 V): I_TRIP = 3.3 / (R * 1e-3) = ~1.33 A con 2.49 kOhm.
 * El ADC con 12 dB satura cerca de 3.1 V -> ~1.25 A medibles.
 */
#define CS_R_IPROPI_OHMS         2490.0f
#define CS_A_IPROPI_A_PER_A      1.0e-3f
