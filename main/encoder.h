#pragma once
#include <stdint.h>

// Encoder on motor shaft, before the 100:1 gearbox. Inputs must be 3.3 V compatible.
#define ENCODER_GPIO_A 18
#define ENCODER_GPIO_B 19
#define ENCODER_SIGN 1

void encoder_init(void);
int64_t encoder_read(void);
