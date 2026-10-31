#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define SPEED_KP_INITIAL 0.0284f
#define SPEED_KI_INITIAL 0.01392f
#define SPEED_PERIOD_US 10000LL
#define SPEED_MAX_GAP_US 50000LL
#define SPEED_REFERENCE_MAX_RAD_S 32.0f
#define SPEED_CURRENT_LIMIT_INITIAL_A 0.1f
#define SPEED_REVERSAL_MAX_RAD_S 0.5f

/* Reference and projected feedback are magnitudes in the selected direction.
 * Output is nonnegative CS-equivalent amperes, not signed winding torque. */
typedef struct {
    float kp, ki, reference_rad_s, error_rad_s;
    float integral_a, output_a, current_limit_a;
    int64_t last_update_us;
    bool limited;
} speed_pi_t;

static inline bool speed_reference_valid(float x)
{
    return isfinite(x) && x >= 0 && x <= SPEED_REFERENCE_MAX_RAD_S;
}

static inline void speed_pi_reset(speed_pi_t *p)
{
    p->integral_a = p->output_a = p->error_rad_s = 0;
    p->last_update_us = 0;
    p->limited = false;
}

static inline bool speed_pi_update(speed_pi_t *p, float projected_rad_s,
                                    int64_t time_us, bool inner_high_limited)
{
    if (!isfinite(projected_rad_s) || !speed_reference_valid(p->reference_rad_s) ||
        !isfinite(p->kp) || !isfinite(p->ki) || p->kp < 0 || p->ki < 0 ||
        !isfinite(p->integral_a) || !isfinite(p->current_limit_a) ||
        p->current_limit_a <= 0 || p->current_limit_a > 1 ||
        time_us <= p->last_update_us) return false;
    int64_t elapsed = time_us-p->last_update_us;
    if (p->last_update_us && (elapsed < SPEED_PERIOD_US || elapsed >= SPEED_MAX_GAP_US)) return false;
    float dt = p->last_update_us ? elapsed*1e-6f : 0;
    float error = p->reference_rad_s-projected_rad_s;
    float candidate = p->integral_a+p->ki*error*dt;
    float requested = p->kp*error+candidate;
    if (!isfinite(requested)) return false;
    bool high = requested > p->current_limit_a;
    bool low = requested < 0;
    bool block = (high && error>0) || (low && error<0) || (inner_high_limited && error>0);
    if (!block) p->integral_a=candidate;
    /* Freeze the extra integral contribution when the inner PWM hits its ceiling. */
    if (inner_high_limited && error>0) requested=p->kp*error+p->integral_a;
    p->output_a=fminf(p->current_limit_a,fmaxf(0,requested));
    p->error_rad_s=error;
    p->last_update_us=time_us;
    p->limited=high || low || (inner_high_limited && error>0);
    return true;
}
