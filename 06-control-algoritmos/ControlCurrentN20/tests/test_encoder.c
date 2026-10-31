#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "encoder_processing.h"

static void near(double actual, double expected)
{
    assert(isfinite(actual));
    assert(fabs(actual - expected) < 1e-10);
}

static void invalid(encoder_sample_t sample)
{
    assert(!sample.speed_valid);
    assert(isnan(sample.output_speed_rad_s));
    assert(sample.speed_window_us == 0);
}

static void test_position_and_speed(void)
{
    encoder_processing_t state = {0};
    assert(ENCODER_COUNTS_PER_OUTPUT_REV == 2800U);
    encoder_sample_t sample = encoder_processing_sample(&state, 2800U, 1000);
    near(sample.output_position_rad, ENCODER_TWO_PI);
    invalid(sample);
    sample = encoder_processing_sample(&state, (uint32_t)-2800, 51000);
    near(sample.output_position_rad, -ENCODER_TWO_PI);
    near(sample.output_speed_rad_s, -40.0 * ENCODER_TWO_PI);

    state = (encoder_processing_t){0};
    invalid(encoder_processing_sample(&state, 0, 1000));
    invalid(encoder_processing_sample(&state, 70, 26000));
    sample = encoder_processing_sample(&state, 140, 51000);
    assert(sample.speed_valid && sample.speed_window_us == 50000U);
    near(sample.output_speed_rad_s, ENCODER_TWO_PI);

    state = (encoder_processing_t){0};
    invalid(encoder_processing_sample(&state, 0, 1000));
    sample = encoder_processing_sample(&state, (uint32_t)-140, 51000);
    near(sample.output_speed_rad_s, -ENCODER_TWO_PI);
}

static void test_wraps_and_large_counts(void)
{
    encoder_processing_t state = {0};
    state.previous_raw = UINT32_MAX - 9U;
    state.count_x4 = 100;
    assert(encoder_processing_extend(&state, 10) == 120);
    assert(encoder_processing_extend(&state, UINT32_MAX - 9U) == 100);
    state.previous_raw = INT32_MAX - 9U;
    assert(encoder_processing_extend(&state, (uint32_t)INT32_MAX + 11U) == 120);
    assert(encoder_processing_extend(&state, INT32_MAX - 9U) == 100);

    state = (encoder_processing_t){0};
    state.count_x4 = (INT64_C(1) << 54) + 3;
    state.reference_count_x4 = state.count_x4 - 2800;
    encoder_sample_t sample = encoder_processing_sample(&state, 0, 1000);
    near(sample.output_position_rad, ENCODER_TWO_PI);
    sample = encoder_processing_sample(&state, 1, 51000);
    near(sample.output_speed_rad_s, ENCODER_TWO_PI / 140.0);
    assert(sample.relative_count_x4 == 2801);
}

static void test_irregular_windows_and_invalid_times(void)
{
    encoder_processing_t state = {0};
    invalid(encoder_processing_sample(&state, 0, 0));
    invalid(encoder_processing_sample(&state, 0, -1));
    assert(state.history_count == 0);
    invalid(encoder_processing_sample(&state, 0, 1000));
    invalid(encoder_processing_sample(&state, 20, 11000));
    invalid(encoder_processing_sample(&state, 60, 31000));
    encoder_sample_t sample = encoder_processing_sample(&state, 110, 56000);
    assert(sample.speed_window_us == 55000U);
    near(sample.output_speed_rad_s, ENCODER_TWO_PI * 2000.0 / 2800.0);
    sample = encoder_processing_sample(&state, 160, 81000);
    assert(sample.speed_window_us == 50000U); // Nearest qualifying predecessor.
    near(sample.output_speed_rad_s, ENCODER_TWO_PI * 2000.0 / 2800.0);
    const size_t history_count = state.history_count;
    invalid(encoder_processing_sample(&state, 160, 81000));
    invalid(encoder_processing_sample(&state, 160, 79000));
    assert(state.history_count == history_count);
}

static void test_zero_reference_preserves_speed(void)
{
    encoder_processing_t state = {0};
    encoder_processing_sample(&state, 0, 1000);
    encoder_processing_sample(&state, 140, 51000);
    const size_t history_count = state.history_count;
    encoder_processing_zero(&state, 168);
    assert(state.reference_count_x4 == 168);
    assert(state.reference_id == 1);
    assert(state.history_count == history_count);
    encoder_sample_t sample = encoder_processing_sample(&state, 168, 61000);
    assert(sample.count_x4 == 168 && sample.relative_count_x4 == 0);
    near(sample.output_position_rad, 0);
    near(sample.output_speed_rad_s, ENCODER_TWO_PI);
    sample = encoder_processing_sample(&state, 280, 101000);
    assert(sample.relative_count_x4 == 112);
    near(sample.output_speed_rad_s, ENCODER_TWO_PI);
}

static void test_history_ring_stop_and_gap(void)
{
    encoder_processing_t state = {0};
    encoder_sample_t sample;
    for (uint32_t i = 0; i < 300; ++i) {
        sample = encoder_processing_sample(&state, i * 2U, 1000 + i * 1000);
        if (i < 50) invalid(sample);
        else {
            assert(sample.speed_window_us == 50000U);
            near(sample.output_speed_rad_s, ENCODER_TWO_PI * 2000.0 / 2800.0);
        }
    }
    assert(state.history_count == ENCODER_HISTORY_CAPACITY);
    for (int i = 1; i <= 50; ++i)
        sample = encoder_processing_sample(&state, 598, 300000 + i * 1000);
    assert(sample.speed_valid);
    near(sample.output_speed_rad_s, 0);
    invalid(encoder_processing_sample(&state, 700, 450001));
    assert(state.history_count == 1);
    invalid(encoder_processing_sample(&state, 700, 500000));
    sample = encoder_processing_sample(&state, 700, 500001);
    assert(sample.speed_valid && sample.speed_window_us == 50000U);
    near(sample.output_speed_rad_s, 0);
}

int main(void)
{
    test_position_and_speed();
    test_wraps_and_large_counts();
    test_irregular_windows_and_invalid_times();
    test_zero_reference_preserves_speed();
    test_history_ring_stop_and_gap();
    puts("PASS: encoder x4, radians, rolling rad/s, wrap, reference zero, timing, stop and stale gap");
    return 0;
}
