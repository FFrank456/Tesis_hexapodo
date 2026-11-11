#pragma once
static inline uint32_t ulTaskNotifyTake(int a, int b) { (void)a; (void)b; return 1; }
static inline void vTaskNotifyGiveFromISR(TaskHandle_t t, BaseType_t *w) { (void)t; (void)w; }
static inline TickType_t xTaskGetTickCount(void) { return 0; }
static inline void vTaskDelayUntil(TickType_t *a, int b) { (void)a; (void)b; }
BaseType_t xTaskCreatePinnedToCore(void (*f)(void*), const char *n, int s, void *a, int p, TaskHandle_t *h, int c);
