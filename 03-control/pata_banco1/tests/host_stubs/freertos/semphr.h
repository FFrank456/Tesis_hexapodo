#pragma once
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (void*)1; }
static inline int xSemaphoreTake(SemaphoreHandle_t a, uint32_t b) { (void)a; (void)b; return 1; }
static inline int xSemaphoreGive(SemaphoreHandle_t a) { (void)a; return 1; }
