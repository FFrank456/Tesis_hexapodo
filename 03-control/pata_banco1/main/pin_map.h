#pragma once
/*
 * Mapa de pines - pata_banco1 (pata completa: Coxa, Femur, Tibia simultaneos)
 *
 * Fuente: repo flavioe-alarcon/hexapodo-secundario-firmware, AGENTS.md y
 * CONTEXTO_TESIS.md sec. 8.1 (ESP32 DevKit V1 + 3x modulo DRV8876 MJKDZ).
 *
 * | Senal        | M1 Coxa  | M2 Femur | M3 Tibia |
 * |--------------|----------|----------|----------|
 * | EN/IN1 (PWM) | GPIO21   | GPIO18   | GPIO16   |
 * | PH/IN2 (PWM) | GPIO5    | GPIO17   | GPIO4    |
 * | CS / IPROPI  | GPIO36 VP| GPIO39 VN| GPIO34   |
 * | ADC1 canal   | CH0      | CH3      | CH6      |
 * | Encoder A    | GPIO32   | GPIO25   | GPIO27   |
 * | Encoder B    | GPIO33   | GPIO13   | GPIO14   |
 *
 * Notas del companero: IN2 de M1 migrado de D19 a D5 (D19 no daba nivel alto);
 * encoder B de M2 migrado de D26 a D13 (diafonia entre jumpers paralelos).
 * SLEEP y PMODE de cada modulo puenteados a 3.3 V (modo PWM/PWM).
 * nFAULT sin conectar (en la PCB final ira a IO4, pero GPIO4 es IN2 de M3).
 *
 * GPIO5 es pin de arranque: durante el boot puede quedar en alto unos ms
 * antes de que el firmware fuerce las salidas a 0 (posible tiron breve en M1).
 *
 * Los arreglos se indexan 0 = Coxa, 1 = Femur, 2 = Tibia. Por serial los
 * motores se nombran M1/M2/M3 o COXA/FEMUR/TIBIA.
 */

#define MOTOR_COUNT 3

/* Motor que reciben los comandos sin prefijo y la telemetria LEGACY al
 * arrancar (0 = Coxa, 1 = Femur, 2 = Tibia). Se cambia en caliente con SEL,1|2|3,
 * pero abrir el puerto suele reiniciar el ESP32 y volver a este valor. */
#define DEFAULT_SELECTED_MOTOR 1

#define MOTOR_NAMES             {"COXA", "FEMUR", "TIBIA"}
#define MOTOR_PIN_IN1           {21, 18, 16}   /* EN/IN1 -> sentido "CW"  */
#define MOTOR_PIN_IN2           {5, 17, 4}     /* PH/IN2 -> sentido "CCW" */
#define MOTOR_PIN_CS            {36, 39, 34}
#define MOTOR_ADC_CHANNELS      {ADC_CHANNEL_0, ADC_CHANNEL_3, ADC_CHANNEL_6}
#define MOTOR_PIN_ENC_A         {32, 25, 27}
#define MOTOR_PIN_ENC_B         {33, 13, 14}
/* Signo inicial del encoder por motor. Verificar: con PWM positivo (IN1) la
 * posicion debe crecer. Se puede cambiar en caliente con Mx,ESIGN,-1 (en STOP). */
#define MOTOR_ENC_SIGN          {1, 1, 1}

#define ENCODER_USE_INTERNAL_PULLUP 1  /* Todos los pines de encoder tienen pull-up interno */

/* nFAULT deshabilitado (no cableado en los modulos de prueba). */
#define DRV_NFAULT_ENABLED       0

/*
 * Resistencia IPROPI del modulo DRV8876 (clon MJKDZ del carrier Pololu).
 * Pololu #4036/#4037: CS a GND con 2.49 kOhm -> ~2.49 V/A (A_IPROPI = 1000 uA/A).
 * VERIFICAR midiendo CS-GND en cada modulo desenergizado.
 * Con VREF atado a SLEEP (3.3 V): I_TRIP = 3.3 / (R * 1e-3) = ~1.33 A con 2.49 kOhm.
 */
#define CS_R_IPROPI_OHMS         2490.0f
#define CS_A_IPROPI_A_PER_A      1.0e-3f
