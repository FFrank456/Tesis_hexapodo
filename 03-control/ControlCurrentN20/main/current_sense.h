#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Divisors of the ESP32 PLL160M clock are exact for these values. */
#define SENSE_PWM_TIMER_HZ          20000000U
#define SENSE_PWM_PERIOD_TICKS      2000U
#define SENSE_PWM_PERIOD_US         100U
#define SENSE_NOMINAL_INTERVAL_US   1000U
#define SENSE_PID_LOSS_TIMEOUT_US  20000U
#define SENSE_CONTINUOUS_RATE_HZ  200000U
#define SENSE_DMA_FRAME_SAMPLES 200U
#define SENSE_DMA_STALE_US 10000
#define SENSE_DATA_VERSION 6U
/* MOD_DRV: R_IPROPI = 3 kOhm, A_IPROPI = 1000 uA/A -> 3.0 V/A (antes 2.49). */
#define SENSE_CS_VOLTS_PER_AMP 3.0f

typedef enum {
    CURRENT_UNINITIALIZED = 0,
    CURRENT_VALID_ON = 1,
    CURRENT_PWM_OFF = 2,
    CURRENT_NO_WINDOW = 3,
    CURRENT_ADC_FLOOR = 4,
    CURRENT_ADC_SATURATED = 5,
    CURRENT_ADC_ERROR = 6,
    CURRENT_CALIBRATION_ERROR = 7,
    CURRENT_OUTPUT_SETTLING = 8,
    CURRENT_CONTINUOUS = 9,
    CURRENT_DMA_OVERFLOW = 10,
    CURRENT_BELOW_RANGE = 11,      /* Valid ON timing, value censored to 0 A. */
    CURRENT_PHASE_UNVERIFIED = 12,
    CURRENT_DMA_ON = 13,
    CURRENT_DMA_BELOW_RANGE = 14,
    CURRENT_CONTINUOUS_ZERO_ASSIGNED = 15, /* raw==0; SDK voltage retained. */
} current_status_t;

static inline bool current_sample_usable(current_status_t status)
{
    return status == CURRENT_VALID_ON || status == CURRENT_CONTINUOUS || status == CURRENT_BELOW_RANGE ||
           status == CURRENT_DMA_ON || status == CURRENT_DMA_BELOW_RANGE ||
           status == CURRENT_CONTINUOUS_ZERO_ASSIGNED;
}

typedef struct {
    int raw;                       /* -1: no conversion attempted/succeeded */
    float voltage;
    float current_a;               /* Calibrated CS / SENSE_CS_VOLTS_PER_AMP; not winding mean current. */
    bool calibrated;               /* This individual voltage conversion succeeded. */
    bool converted;
    current_status_t status;
    int64_t sample_time_us;         /* DMA EOF ISR timestamp, not exact aperture. */
    uint32_t read_us;               /* Nominal block span (legacy field name). */
    int32_t phase_us;               /* Estimated phase; -1 when unavailable/DC. */
    /* Arithmetic means of a complete DMA block. Calibrate each sample first;
     * assign raw==0 to 0 V before averaging. No PWM phase selection. */
    float adc_raw_mean;
    float adc_voltage_mean;
    float current_mean_a;          /* Mean CS / SENSE_CS_VOLTS_PER_AMP; not winding mean current. */
    uint32_t dma_block_samples;     /* 200 for a valid mean, 0 with NaN on error. */
    uint32_t total_samples;
    uint32_t overflows;
    uint32_t accepted_samples;
    uint32_t rejected_window;
    uint32_t rejected_quality;
    uint32_t generation;
} current_sample_t;

/* Start continuous acquisition and publish independent 200-sample block means. */
void current_sense_init(void);
/* Read the latest complete block; the retained PWM arguments do not gate DMA.
 * Stale or invalid blocks expose NaN means and a zero block sample count. */
current_sample_t current_sense_read(uint32_t duty_ticks, int active_gpio,
                                    int inactive_gpio, int64_t output_changed_us);
