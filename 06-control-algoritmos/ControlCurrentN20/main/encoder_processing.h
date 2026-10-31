#pragma once

#include "encoder.h"
#include <limits.h>
#include <math.h>
#include <stddef.h>

// At a nominal 1 kHz, 128 entries cover the 50 ms velocity window with margin.
// This bounded state is shared by the hardware wrapper and deterministic tests.
#define ENCODER_HISTORY_CAPACITY 128U
#define ENCODER_TWO_PI 6.283185307179586476925286766559

typedef struct {
    int64_t count_x4;
    int64_t time_us;
} encoder_history_entry_t;

typedef struct {
    uint32_t previous_raw;
    int64_t count_x4;
    int64_t reference_count_x4;
    uint32_t reference_id;
    encoder_history_entry_t history[ENCODER_HISTORY_CAPACITY];
    size_t history_next;
    size_t history_count;
} encoder_processing_t;

// Extend the PCNT driver's accumulated 32-bit count through wraps in either
// direction. Successive hardware reads must be less than 2^31 counts apart.
static inline int64_t encoder_processing_extend(encoder_processing_t *state,
                                                 uint32_t raw)
{
    const uint32_t wrapped_delta = raw - state->previous_raw;
    const int64_t delta = wrapped_delta <= INT32_MAX
        ? (int64_t)wrapped_delta : (int64_t)wrapped_delta - 4294967296LL;
    state->count_x4 += ENCODER_SIGN * delta;
    state->previous_raw = raw;
    return state->count_x4;
}

// A reference change affects only position. Velocity always differences the
// continuous signed count, so zeroing while moving cannot create a speed spike.
static inline void encoder_processing_zero(encoder_processing_t *state,
                                           uint32_t raw)
{
    state->reference_count_x4 = encoder_processing_extend(state, raw);
    ++state->reference_id;
}

// Compute a sample using the nearest recorded predecessor at least 50 ms old.
// Differences are formed as integers before conversion to retain precision
// after long running times. Nonpositive/repeated/backwards times are invalid
// and do not enter the history. A gap above 100 ms restarts velocity history;
// no estimate is published until a new complete window has accumulated.
static inline encoder_sample_t encoder_processing_sample(
    encoder_processing_t *state, uint32_t raw, int64_t time_us)
{
    const int64_t count = encoder_processing_extend(state, raw);
    const int64_t relative = count - state->reference_count_x4;
    const double radians_per_count = ENCODER_TWO_PI / ENCODER_COUNTS_PER_OUTPUT_REV;
    encoder_sample_t result = {
        .count_x4 = count,
        .relative_count_x4 = relative,
        .time_us = time_us,
        .reference_count_x4 = state->reference_count_x4,
        .output_position_rad = (double)relative * radians_per_count,
        .output_speed_rad_s = NAN,
        .speed_window_us = 0,
        .reference_id = state->reference_id,
        .speed_valid = false,
    };

    if (time_us <= 0) return result;
    if (state->history_count > 0) {
        const size_t latest_index = (state->history_next + ENCODER_HISTORY_CAPACITY - 1U)
            % ENCODER_HISTORY_CAPACITY;
        const int64_t previous_time = state->history[latest_index].time_us;
        if (time_us <= previous_time) return result;
        if (time_us - previous_time > ENCODER_SPEED_MAX_GAP_US) {
            state->history_count = 0;
            state->history_next = 0;
        }
    }

    for (size_t age = 0; age < state->history_count; ++age) {
        const size_t index = (state->history_next + ENCODER_HISTORY_CAPACITY - 1U - age)
            % ENCODER_HISTORY_CAPACITY;
        const encoder_history_entry_t previous = state->history[index];
        const int64_t elapsed_us = time_us - previous.time_us;
        if (elapsed_us < ENCODER_SPEED_MIN_WINDOW_US) continue;
        if (elapsed_us <= ENCODER_SPEED_MAX_GAP_US) {
            const int64_t delta_count = count - previous.count_x4;
            result.output_speed_rad_s = (double)delta_count * radians_per_count
                * 1000000.0 / (double)elapsed_us;
            result.speed_window_us = (uint32_t)elapsed_us;
            result.speed_valid = true;
        }
        break;
    }

    state->history[state->history_next] = (encoder_history_entry_t){count, time_us};
    state->history_next = (state->history_next + 1U) % ENCODER_HISTORY_CAPACITY;
    if (state->history_count < ENCODER_HISTORY_CAPACITY) ++state->history_count;
    return result;
}
