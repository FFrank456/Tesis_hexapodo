#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "pin_map.h"

/* Divisors of the ESP32 PLL160M clock are exact for these values. */
#define SENSE_PWM_TIMER_HZ          20000000U
#define SENSE_PWM_PERIOD_TICKS      2000U
#define SENSE_PWM_PERIOD_US         100U
#define SENSE_NOMINAL_INTERVAL_US   1000U
#define SENSE_PID_LOSS_TIMEOUT_US  20000U
/* Three channels share one ADC1 DMA stream (pattern CH0, CH3, CH6).
 * The ESP32 SAR in DMA mode converts at ~200 kS/s at most (SAR clock 5 MHz);
 * asking the I2S for more returns duplicated/misaligned results. The total is
 * therefore kept at 192 kS/s: 64 kS/s and 64 samples per 1 ms block per
 * channel. 6.4 samples per 10 kHz PWM period, so the 1 ms mean covers 32
 * distinct PWM phases instead of a few fixed ones. The frame (192 results x
 * 2 bytes) must be a multiple of SOC_ADC_DIGI_DATA_BYTES_PER_CONV (4 B). */
#define SENSE_CHANNEL_COUNT         MOTOR_COUNT
#define SENSE_PER_CHANNEL_RATE_HZ   64000U
#define SENSE_CONTINUOUS_RATE_HZ    (SENSE_PER_CHANNEL_RATE_HZ * SENSE_CHANNEL_COUNT)
#define SENSE_DMA_FRAME_SAMPLES     64U    /* per channel, per 1 ms block */
#define SENSE_FRAME_RESULTS         (SENSE_DMA_FRAME_SAMPLES * SENSE_CHANNEL_COUNT)
/* A block is valid with at least this many samples of its channel (tolerates
 * a pattern that does not split the frame exactly 64/64/64). */
#define SENSE_MIN_BLOCK_SAMPLES     58U
/* Results with an unexpected channel field tolerated per frame before the
 * whole frame is rejected. */
#define SENSE_MAX_MALFORMED_PER_FRAME 6U
#define SENSE_DMA_STALE_US 10000
#define SENSE_DATA_VERSION 6U
/* Modulo DRV8876 MJKDZ: V/A = R_IPROPI * A_IPROPI (ver pin_map.h; 2.49 V/A por defecto). */
#define SENSE_CS_VOLTS_PER_AMP (CS_R_IPROPI_OHMS * CS_A_IPROPI_A_PER_A)

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
    /* Arithmetic means of this channel's samples in a complete DMA block. */
    float adc_raw_mean;
    float adc_voltage_mean;
    float current_mean_a;          /* Mean CS / SENSE_CS_VOLTS_PER_AMP; not winding mean current. */
    uint32_t dma_block_samples;     /* samples in a valid mean (~64), 0 with NaN on error. */
    uint32_t total_samples;
    uint32_t overflows;             /* Shared by all channels. */
    uint32_t accepted_samples;
    uint32_t rejected_window;
    uint32_t rejected_quality;
    uint32_t generation;
} current_sample_t;

typedef struct {
    uint32_t frames;               /* frames received since boot */
    uint32_t bad_frames;           /* short reads / overflows */
    uint32_t malformed_last;       /* unknown-channel results in the last frame */
    uint32_t malformed_total;
    uint16_t channel_mask_last;    /* bit n set: channel n seen in the last frame */
    uint32_t overflows;
    uint32_t bytes_last;
    bool calibrated;
    struct {
        uint32_t count, accepted, saturated;
        int raw_min, raw_max;
        float raw_mean;
        current_status_t status;
    } ch[SENSE_CHANNEL_COUNT];
} current_sense_diag_t;

/* Snapshot of the last frame's per-channel statistics (DIAG command). */
current_sense_diag_t current_sense_diag(void);

/* Start continuous acquisition of all channels and publish per-channel block means. */
void current_sense_init(void);
/* Latest complete block of one motor (0..MOTOR_COUNT-1). Stale or invalid
 * blocks expose NaN means and a zero block sample count. */
current_sample_t current_sense_read(int motor);
