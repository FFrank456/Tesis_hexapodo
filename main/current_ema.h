#pragma once
#include <stdbool.h>
typedef struct { float value; bool initialized; } current_ema_t;
static inline float current_ema_push(current_ema_t *filter, float value, float alpha)
{
    if (!filter->initialized) { filter->value=value; filter->initialized=true; }
    else filter->value+=alpha*(value-filter->value);
    return filter->value;
}
