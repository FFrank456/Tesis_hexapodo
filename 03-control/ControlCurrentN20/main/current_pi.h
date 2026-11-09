#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>

/* Gains use percent PWM / A and percent PWM / (A*s). Wire names stay PID. */
#define PID_KP_INITIAL 8.832f
#define PID_KI_INITIAL 80090.0f
#define PID_KD_INITIAL 0.0f
#define PI_REFERENCE_MAX_A 1.0f /* Command range, NOT a hardware current limit. */

typedef struct {
    float kp, ki, kd, integrator, previous_error;
    float output_min_percent, output_max_percent, current_ref_a;
    float output_percent, error_a;
    bool previous_error_valid;
    int64_t last_sample_us;
} pid_controller_t;

static inline bool control_parse_float(const char *text, float *value)
{
    if (!text || !*text) return false;
    char *end;
    float x = strtof(text, &end);
    if (end == text || *end || !isfinite(x)) return false;
    *value = x;
    return true;
}

static inline bool pi_reference_valid(float value)
{
    return isfinite(value) && value >= 0 && value <= PI_REFERENCE_MAX_A;
}

static inline void pid_reset(pid_controller_t *p)
{
    p->integrator = p->previous_error = p->output_percent = p->error_a = 0;
    p->previous_error_valid = false;
    p->last_sample_us = 0;
}

/* Integrate once per NEW block, using its elapsed time. A missed snapshot is
 * approximated by the current error over the interval; no fixed 1 ms fallback.
 * Caller stops on invalid/stale feedback and gaps >= 20 ms before this call. */
static inline bool pi_update_sample(pid_controller_t *p, float measured,
                                     int64_t sample_us)
{
    if (!isfinite(measured) || !pi_reference_valid(p->current_ref_a) ||
        !isfinite(p->kp) || !isfinite(p->ki) || p->kp < 0 || p->ki < 0 ||
        !isfinite(p->integrator) || p->kd != 0 ||
        !isfinite(p->output_min_percent) || !isfinite(p->output_max_percent) ||
        p->output_min_percent < 0 || p->output_max_percent > 100 ||
        p->output_min_percent > p->output_max_percent) return false;
    if (sample_us <= p->last_sample_us) return false;
    if (p->last_sample_us && sample_us-p->last_sample_us >= 20000) return false;
    float dt = p->last_sample_us ? (sample_us-p->last_sample_us)*1e-6f : 0;
    float error = p->current_ref_a-measured;
    float proportional = p->kp*error;
    float candidate = p->integrator+p->ki*error*dt;
    float effort = proportional+candidate;
    if (!isfinite(effort)) return false;
    bool high = effort > p->output_max_percent;
    bool low = effort < p->output_min_percent;
    if (!((high && error > 0) || (low && error < 0))) p->integrator = candidate;
    p->output_percent = fminf(p->output_max_percent,fmaxf(p->output_min_percent,effort));
    p->error_a = error;
    p->last_sample_us = sample_us;
    return true;
}
