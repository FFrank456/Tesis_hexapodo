#include "current_sense.h"
#include "current_ema.h"
#include <math.h>
#include <string.h>
#ifndef CURRENT_SENSE_HOST_TEST
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#endif

#define FRAME_BYTES (SENSE_DMA_FRAME_SAMPLES * SOC_ADC_DIGI_RESULT_BYTES)
#define FRAME_US (SENSE_DMA_FRAME_SAMPLES * 1000000U / SENSE_CONTINUOUS_RATE_HZ)
#define FRAME_QUEUE_DEPTH 8

_Static_assert(SENSE_DMA_FRAME_SAMPLES * 1000000U % SENSE_CONTINUOUS_RATE_HZ == 0,
               "DMA block duration must be a whole number of microseconds");

typedef struct { int64_t eof_us; uint32_t bytes; } frame_meta_t;

static adc_continuous_handle_t adc;
static adc_cali_handle_t cal;
static QueueHandle_t frames;
static int millivolts[4096]; /* SDK conversion once at startup, not 200k calls/s. */
static bool calibrated;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static current_sample_t latest = {.raw=-1, .voltage=NAN, .current_a=NAN,
    .ema_a=NAN, .adc_raw_mean=NAN, .adc_voltage_mean=NAN, .current_mean_a=NAN,
    .phase_us=-1, .status=CURRENT_UNINITIALIZED};
static current_ema_t filter;
static uint32_t overflows;

/* Metadata is paired FIFO with the driver's DMA frames. A lost entry makes the
 * pairing unsafe: overflow forces stop/flush/restart of both queues. */
static bool IRAM_ATTR frame_done(adc_continuous_handle_t h,
                                 const adc_continuous_evt_data_t *event, void *ctx)
{
    (void)h; (void)ctx;
    frame_meta_t m = {.eof_us=esp_timer_get_time(), .bytes=event->size};
    BaseType_t wake=pdFALSE;
    if (xQueueSendFromISR(frames, &m, &wake) != pdTRUE) {
        portENTER_CRITICAL_ISR(&lock); overflows++; portEXIT_CRITICAL_ISR(&lock);
    }
    return wake == pdTRUE;
}

static bool IRAM_ATTR overflow_cb(adc_continuous_handle_t h,
                                  const adc_continuous_evt_data_t *e, void *ctx)
{
    (void)h; (void)e; (void)ctx;
    portENTER_CRITICAL_ISR(&lock); overflows++; portEXIT_CRITICAL_ISR(&lock);
    return false;
}

static current_sample_t empty_sample(current_status_t status)
{
    return (current_sample_t){.raw=-1, .voltage=NAN, .current_a=NAN, .ema_a=NAN,
        .adc_raw_mean=NAN, .adc_voltage_mean=NAN, .current_mean_a=NAN,
        .phase_us=-1, .status=status, .sample_time_us=esp_timer_get_time(),
        .calibrated=calibrated};
}

/* Invalidate published measurements without erasing the diagnostic SDK voltage
 * or the internal EMA memory. A partial/stale block is never a valid mean. */
static void invalidate_measurement(current_sample_t *s)
{
    s->current_a=NAN;
    s->ema_a=NAN;
    s->adc_raw_mean=NAN;
    s->adc_voltage_mean=NAN;
    s->current_mean_a=NAN;
    s->dma_block_samples=0;
}

/* This task owns calibration lookup, counters, block means and EMA updates.
 * PWM does not gate acquisition or filtering. EOF is a block receipt timestamp. */
static void adc_task(void *arg)
{
    (void)arg;
    uint8_t buf[FRAME_BYTES] __attribute__((aligned(4)));
    uint32_t total=0, accepted=0, window=0, quality=0, seen_overflows=0;
    while (true) {
        frame_meta_t m;
        current_sample_t s=empty_sample(CURRENT_ADC_ERROR);
        portENTER_CRITICAL(&lock);
        uint32_t lost=overflows;
        portEXIT_CRITICAL(&lock);
        if (lost != seen_overflows) {
            ESP_ERROR_CHECK(adc_continuous_stop(adc));
            ESP_ERROR_CHECK(adc_continuous_flush_pool(adc));
            xQueueReset(frames);
            portENTER_CRITICAL(&lock); seen_overflows=overflows; portEXIT_CRITICAL(&lock);
            s.status=CURRENT_DMA_OVERFLOW;
            /* Keep EMA memory, but never display that old value as fresh data. */
            ESP_ERROR_CHECK(adc_continuous_start(adc));
        } else if (xQueueReceive(frames, &m, pdMS_TO_TICKS(20)) == pdTRUE) {
            uint32_t bytes=0;
            esp_err_t err=adc_continuous_read(adc, buf, sizeof(buf), &bytes, 0);
            if (err != ESP_OK || bytes != FRAME_BYTES || m.bytes != FRAME_BYTES) {
                portENTER_CRITICAL(&lock); overflows++; portEXIT_CRITICAL(&lock);
                s.status=CURRENT_DMA_OVERFLOW;
            } else {
                bool fresh=esp_timer_get_time()-m.eof_us <= SENSE_DMA_STALE_US;
                current_ema_t candidate;
                portENTER_CRITICAL(&lock);
                candidate=filter;
                portEXIT_CRITICAL(&lock);
                uint32_t block_accepted=0, block_window=0, block_quality=0;
                uint32_t raw_sum=0, processed_mv_sum=0;
                s.status=CURRENT_CONTINUOUS;
                bool eligible=fresh;
                s.sample_time_us=m.eof_us; /* ISR receipt, not hardware aperture. */
                s.read_us=FRAME_US;       /* Block span, not oneshot call time. */
                bool malformed=false;
                for (uint32_t i=0; i<SENSE_DMA_FRAME_SAMPLES; i++) {
                    adc_digi_output_data_t v={0};
                    memcpy(&v, buf+i*SOC_ADC_DIGI_RESULT_BYTES, SOC_ADC_DIGI_RESULT_BYTES);
                    total++;
                    if (v.type1.channel!=ADC_CHANNEL_4) {
                        malformed=true; block_quality++; continue;
                    }
                    int raw=v.type1.data;
                    int mv=millivolts[raw];
                    /* Preserve diagnostic raw/voltage even without an ON estimate. */
                    if (!block_accepted) { s.raw=raw; s.voltage=calibrated ? mv/1000.0f : NAN; }
                    s.converted=true;
                    if (!eligible) { block_quality++; continue; }
                    if (!calibrated || raw==4095) {
                        block_quality++;
                        continue;
                    }
                    /* Censor only code zero, before EMA; preserve SDK voltage
                     * in diagnostics. This is an assignment, not calibration. */
                    float amps=raw==0 ? 0.0f : mv/2490.0f;
                    raw_sum+=(uint32_t)raw;
                    processed_mv_sum+=raw==0 ? 0U : (uint32_t)mv;
                    current_ema_push(&candidate, amps, SENSE_CONTINUOUS_ALPHA);
                    block_accepted++;
                    s.raw=raw; s.voltage=mv/1000.0f; s.current_a=amps;
                    s.ema_a=candidate.value; s.phase_us=-1;
                    s.status=raw==0 ? CURRENT_CONTINUOUS_ZERO_ASSIGNED : CURRENT_CONTINUOUS;
                }
                if (!calibrated) s.status=CURRENT_CALIBRATION_ERROR;
                else if (malformed) s.status=CURRENT_ADC_ERROR;
                else if (eligible && block_quality) s.status=CURRENT_ADC_SATURATED;
                portENTER_CRITICAL(&lock);
                if (overflows!=seen_overflows || malformed || !fresh || block_quality) {
                    block_quality+=block_accepted; block_accepted=0;
                    if (overflows!=seen_overflows) s.status=CURRENT_DMA_OVERFLOW;
                    else if (malformed || !fresh) s.status=CURRENT_ADC_ERROR;
                } else if (block_accepted==SENSE_DMA_FRAME_SAMPLES) {
                    filter=candidate;
                    s.adc_raw_mean=(float)raw_sum/SENSE_DMA_FRAME_SAMPLES;
                    s.adc_voltage_mean=(float)processed_mv_sum/(1000.0f*SENSE_DMA_FRAME_SAMPLES);
                    s.current_mean_a=s.adc_voltage_mean/2.49f;
                    s.dma_block_samples=block_accepted;
                }
                portEXIT_CRITICAL(&lock);
                accepted+=block_accepted; window+=block_window; quality+=block_quality;
                if (!current_sample_usable(s.status)) invalidate_measurement(&s);
            }
        }
        s.total_samples=total; s.accepted_samples=accepted;
        s.rejected_window=window; s.rejected_quality=quality;
        portENTER_CRITICAL(&lock); s.overflows=overflows; latest=s; portEXIT_CRITICAL(&lock);
    }
}

void current_sense_init(void)
{
    adc_cali_line_fitting_config_t c={.unit_id=ADC_UNIT_1, .atten=ADC_ATTEN_DB_12,
        .bitwidth=ADC_BITWIDTH_12, .default_vref=1100};
    calibrated=adc_cali_create_scheme_line_fitting(&c, &cal)==ESP_OK;
    for (int raw=0; raw<4096 && calibrated; raw++)
        calibrated=adc_cali_raw_to_voltage(cal, raw, &millivolts[raw])==ESP_OK;
    frames=xQueueCreate(FRAME_QUEUE_DEPTH, sizeof(frame_meta_t));
    configASSERT(frames);
    adc_continuous_handle_cfg_t h={.max_store_buf_size=FRAME_BYTES*FRAME_QUEUE_DEPTH,
        .conv_frame_size=FRAME_BYTES};
    ESP_ERROR_CHECK(adc_continuous_new_handle(&h, &adc));
    adc_digi_pattern_config_t p={.atten=ADC_ATTEN_DB_12, .channel=ADC_CHANNEL_4,
        .unit=ADC_UNIT_1, .bit_width=12};
    adc_continuous_config_t cfg={.sample_freq_hz=SENSE_CONTINUOUS_RATE_HZ,
        .conv_mode=ADC_CONV_SINGLE_UNIT_1, .format=ADC_DIGI_OUTPUT_FORMAT_TYPE1,
        .pattern_num=1, .adc_pattern=&p};
    ESP_ERROR_CHECK(adc_continuous_config(adc, &cfg));
    adc_continuous_evt_cbs_t cb={.on_pool_ovf=overflow_cb, .on_conv_done=frame_done};
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc, &cb, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(adc));
    BaseType_t ok=xTaskCreatePinnedToCore(adc_task, "adc_dma", 4096, NULL, 12, NULL, 1);
    configASSERT(ok==pdPASS);
}

current_sample_t current_sense_read(uint32_t duty_ticks, int active_gpio,
                                    int inactive_gpio, int64_t changed)
{
    (void)duty_ticks; (void)active_gpio; (void)inactive_gpio; (void)changed;
    portENTER_CRITICAL(&lock);
    current_sample_t s=latest;
    portEXIT_CRITICAL(&lock);
    int64_t now=esp_timer_get_time();
    if (now-s.sample_time_us>SENSE_DMA_STALE_US) s.status=CURRENT_ADC_ERROR;
    if (!current_sample_usable(s.status)) invalidate_measurement(&s);
    return s;
}
