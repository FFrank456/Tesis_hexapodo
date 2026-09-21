#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "current_sense.h"

/* Signed arithmetic avoids the former unsigned underflow at 0% PWM.
 * This classifier is usable only with an independently validated phase. */
static inline bool dma_window_accept(uint32_t duty, int phase_us)
{
    if (!duty || duty > SENSE_PWM_PERIOD_TICKS) return false;
    if (duty == SENSE_PWM_PERIOD_TICKS) return true;
    int on_us = (int)((uint64_t)duty * SENSE_PWM_PERIOD_US / SENSE_PWM_PERIOD_TICKS);
    return phase_us >= (int)SENSE_SETTLE_US + SENSE_PHASE_UNCERTAINTY_US &&
           phase_us < on_us - (int)SENSE_END_GUARD_US - SENSE_PHASE_UNCERTAINTY_US;
}
