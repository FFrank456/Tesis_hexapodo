#include "pwm_sequence.h"
#include <math.h>
#include <stdlib.h>

bool sequence_parse(const char *text, pwm_sequence_t *out)
{
    pwm_sequence_t candidate = {.index = -1};
    uint32_t total = 0;
    if (!text || !*text) return false;
    for (;;) {
        if (candidate.count == SEQUENCE_MAX_STAGES) return false;
        char *end;
        float pwm = strtof(text, &end);
        if (end == text || *end != ':' || !isfinite(pwm) || pwm < -100 || pwm > 100) return false;
        text = end + 1;
        if (*text < '0' || *text > '9') return false;
        unsigned long duration = strtoul(text, &end, 10);
        if (end == text || duration < 100 || duration > 30000 || (*end && *end != ';')) return false;
        total += (uint32_t)duration;
        if (total > SEQUENCE_MAX_DURATION_MS) return false;
        candidate.pwm[candidate.count] = pwm;
        candidate.duration_ms[candidate.count++] = (uint32_t)duration;
        if (!*end) break;
        text = end + 1;
        if (!*text) return false;
    }
    *out = candidate;
    return true;
}

void sequence_start(pwm_sequence_t *sequence, int64_t now_us)
{
    sequence->index = -1;
    sequence->start_us = sequence->stage_start_us = now_us;
    sequence->active = true;
}

bool sequence_tick(pwm_sequence_t *sequence, int64_t now_us, float *pwm)
{
    if (!sequence->active) return false;
    int next = 0;
    int64_t boundary = sequence->start_us;
    while (next < sequence->count && now_us - boundary >= (int64_t)sequence->duration_ms[next] * 1000) {
        boundary += (int64_t)sequence->duration_ms[next++] * 1000;
    }
    if (next == sequence->count) {
        sequence->active = false;
        sequence->index = -1;
        sequence->stage_start_us = now_us;
        *pwm = 0;
        return true;
    }
    if (next == sequence->index) return false;
    sequence->index = next;
    sequence->stage_start_us = now_us;
    *pwm = sequence->pwm[next];
    return true;
}
