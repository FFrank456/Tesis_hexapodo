#pragma once
#include <stdbool.h>
#include <stdint.h>

#define SEQUENCE_MAX_STAGES 48
#define SEQUENCE_MAX_DURATION_MS 120000

typedef struct {
    float pwm[SEQUENCE_MAX_STAGES];
    uint32_t duration_ms[SEQUENCE_MAX_STAGES];
    int count;
    int index; // -1 when stopped; zero-based while running
    uint32_t id;
    int64_t start_us;
    int64_t stage_start_us;
    bool active;
} pwm_sequence_t;

// Format: percent:duration_ms;percent:duration_ms (no empty stages).
bool sequence_parse(const char *text, pwm_sequence_t *out);
void sequence_start(pwm_sequence_t *sequence, int64_t now_us);
// True when output must change. A late call skips expired stages.
bool sequence_tick(pwm_sequence_t *sequence, int64_t now_us, float *pwm);
