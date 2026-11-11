#pragma once
#include <stdint.h>
typedef int BaseType_t; typedef uint32_t TickType_t; typedef int portMUX_TYPE;
typedef void *TaskHandle_t; typedef void *SemaphoreHandle_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY 0xffffffffu
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define portENTER_CRITICAL_ISR(x) ((void)(x))
#define portEXIT_CRITICAL_ISR(x) ((void)(x))
#define pdMS_TO_TICKS(x) (x)
#define configASSERT(x) do { if (!(x)) abort(); } while (0)
#include <stdlib.h>
