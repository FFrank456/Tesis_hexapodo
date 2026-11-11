#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERROR_CHECK(x) do { if ((x)!=0) abort(); } while (0)
#include <stdlib.h>
