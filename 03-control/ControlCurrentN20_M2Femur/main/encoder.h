#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "pin_map.h"

// Encoder on motor shaft, before the 100:1 gearbox. Inputs must be 3.3 V compatible.
// Datasheet values supplied by the user: 7 pulses/rev/channel, quadrature x4.
// The two channels are already included in x4: 7 * 4 * 100 = 2800 counts/output rev.
// All position and speed conversion runs in the ESP32. Do not multiply the
// returned counts by four again. Callers serialize this API with g_motor_lock.
// Mapa M2 Femur (pin_map.h): A = GPIO25, B = GPIO13, con pull-up interno.
// Verificar ENCODER_SIGN: con PWM en IN1 la posicion debe crecer.
#define ENCODER_GPIO_A PIN_ENCODER_A
#define ENCODER_GPIO_B PIN_ENCODER_B
#define ENCODER_SIGN 1
#define ENCODER_DATA_VERSION 1U
#define ENCODER_PULSES_PER_MOTOR_REV 7U
#define ENCODER_QUADRATURE_FACTOR 4U
#define ENCODER_GEAR_RATIO 100U
#define ENCODER_COUNTS_PER_OUTPUT_REV \
    (ENCODER_PULSES_PER_MOTOR_REV * ENCODER_QUADRATURE_FACTOR * ENCODER_GEAR_RATIO)
#define ENCODER_SPEED_MIN_WINDOW_US 50000U
#define ENCODER_SPEED_MAX_GAP_US 100000U

typedef struct {
    int64_t count_x4;                 // Continuous signed count since boot.
    int64_t relative_count_x4;        // Count relative to the ESP32 reference.
    int64_t time_us;                  // Timestamp of this encoder read.
    int64_t reference_count_x4;
    double output_position_rad;      // Relative output-shaft position in rad.
    double output_speed_rad_s;       // Signed speed, NAN while invalid.
    uint32_t speed_window_us;         // Actual differencing interval, 0 if invalid.
    uint32_t reference_id;            // Increments on each reference-zero command.
    bool speed_valid;
} encoder_sample_t;

void encoder_init(void);
// Read and extend the hardware count without changing speed history.
int64_t encoder_read(void);
// Capture position and speed together, independently of ADC/UART timestamps.
encoder_sample_t encoder_sample(void);
// Capture a fresh count as the position origin without resetting PCNT or speed.
void encoder_zero_reference(void);
