#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "pin_map.h"

// Encoders on motor shaft, before the 100:1 gearbox (los tres N20 1:100).
// 7 pulses/rev/channel, quadrature x4: 7 * 4 * 100 = 2800 counts/output rev.
// One PCNT unit per motor; pins and initial sign per motor in pin_map.h.
// All position and speed conversion runs in the ESP32. Callers serialize this
// API with g_motor_lock.
#define ENCODER_COUNT MOTOR_COUNT
// Sign applied inside encoder_processing (kept at +1; the per-motor sign is
// applied to the raw PCNT count in encoder.c so the shared math stays tested).
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

// Initialize all ENCODER_COUNT PCNT units.
void encoder_init(void);
// Capture position and speed of one motor (0..ENCODER_COUNT-1).
encoder_sample_t encoder_sample(int motor);
// Capture a fresh count as the position origin without resetting PCNT or speed.
void encoder_zero_reference(int motor);
// Change the counting sign (+1/-1). Restarts that motor's count, velocity
// history and position reference (new reference_id). Call only while stopped.
void encoder_set_sign(int motor, int sign);
int encoder_get_sign(int motor);
