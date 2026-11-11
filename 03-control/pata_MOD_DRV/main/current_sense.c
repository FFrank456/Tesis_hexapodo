#include "current_sense.h"
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

/*
 * Multi-channel continuous ADC (pata_banco1 / pata_MOD_DRV).
 *
 * One ADC1 DMA stream converts the pattern CH0, CH3, CH6 (Coxa, Femur, Tibia)
 * at 192 kS/s total. Each 1 ms frame holds 64 results per channel. Results are
 * routed by their channel field, never by position, so the published means do
 * not depend on the pattern start. A frame-level fault (overflow, short read,
 * unknown channel, stale EOF) invalidates every channel; saturation or a
 * missing sample invalidates only that channel.
 */

#define FRAME_BYTES (SENSE_FRAME_RESULTS * SOC_ADC_DIGI_RESULT_BYTES)
#define FRAME_US (SENSE_FRAME_RESULTS * 1000000U / SENSE_CONTINUOUS_RATE_HZ)
#define FRAME_QUEUE_DEPTH 8

_Static_assert(SENSE_FRAME_RESULTS * 1000000U % SENSE_CONTINUOUS_RATE_HZ == 0,
               "DMA block duration must be a whole number of microseconds");
_Static_assert(FRAME_BYTES % 4 == 0,
               "ESP32: conv_frame_size must be a multiple of SOC_ADC_DIGI_DATA_BYTES_PER_CONV (4)");

typedef struct { int64_t eof_us; uint32_t bytes; } frame_meta_t;

static const int k_channels[SENSE_CHANNEL_COUNT] = MOTOR_ADC_CHANNELS;
static int8_t channel_to_index[16];

static adc_continuous_handle_t adc;
static adc_cali_handle_t cal;
static QueueHandle_t frames;
static int millivolts[4096]; /* SDK conversion once at startup, not per sample. */
static bool calibrated;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static current_sample_t latest[SENSE_CHANNEL_COUNT];
static uint32_t overflows;
static current_sense_diag_t diag;

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
    return (current_sample_t){.raw=-1, .voltage=NAN, .current_a=NAN,
        .adc_raw_mean=NAN, .adc_voltage_mean=NAN, .current_mean_a=NAN,
        .phase_us=-1, .status=status, .sample_time_us=esp_timer_get_time(),
        .calibrated=calibrated};
}

/* Invalidate published measurements while preserving the diagnostic SDK voltage.
 * A partial/stale block is never a valid mean. */
static void invalidate_measurement(current_sample_t *s)
{
    s->current_a=NAN;
    s->adc_raw_mean=NAN;
    s->adc_voltage_mean=NAN;
    s->current_mean_a=NAN;
    s->dma_block_samples=0;
}

typedef struct {
    uint32_t total, accepted, window, quality;
} channel_counters_t;

/* This task owns calibration lookup, counters and complete-block arithmetic means.
 * PWM does not gate acquisition. EOF is a block receipt timestamp. */
static void adc_task(void *arg)
{
    (void)arg;
    static uint8_t buf[FRAME_BYTES] __attribute__((aligned(4)));
    channel_counters_t counters[SENSE_CHANNEL_COUNT] = {0};
    uint32_t seen_overflows=0;
    while (true) {
        frame_meta_t m;
        current_sample_t s[SENSE_CHANNEL_COUNT];
        for (int c=0; c<SENSE_CHANNEL_COUNT; c++) s[c]=empty_sample(CURRENT_ADC_ERROR);
        portENTER_CRITICAL(&lock);
        uint32_t lost=overflows;
        portEXIT_CRITICAL(&lock);
        if (lost != seen_overflows) {
            ESP_ERROR_CHECK(adc_continuous_stop(adc));
            ESP_ERROR_CHECK(adc_continuous_flush_pool(adc));
            xQueueReset(frames);
            portENTER_CRITICAL(&lock); seen_overflows=overflows; portEXIT_CRITICAL(&lock);
            for (int c=0; c<SENSE_CHANNEL_COUNT; c++) s[c].status=CURRENT_DMA_OVERFLOW;
            ESP_ERROR_CHECK(adc_continuous_start(adc));
        } else if (xQueueReceive(frames, &m, pdMS_TO_TICKS(20)) == pdTRUE) {
            uint32_t bytes=0;
            esp_err_t err=adc_continuous_read(adc, buf, sizeof(buf), &bytes, 0);
            diag.frames++; diag.bytes_last=bytes;
            if (err != ESP_OK || bytes != FRAME_BYTES || m.bytes != FRAME_BYTES) {
                diag.bad_frames++;
                portENTER_CRITICAL(&lock); overflows++; portEXIT_CRITICAL(&lock);
                for (int c=0; c<SENSE_CHANNEL_COUNT; c++) s[c].status=CURRENT_DMA_OVERFLOW;
            } else {
                bool fresh=esp_timer_get_time()-m.eof_us <= SENSE_DMA_STALE_US;
                uint32_t count[SENSE_CHANNEL_COUNT]={0}, accepted_n[SENSE_CHANNEL_COUNT]={0};
                uint32_t quality_n[SENSE_CHANNEL_COUNT]={0};
                uint32_t raw_sum[SENSE_CHANNEL_COUNT]={0}, mv_sum[SENSE_CHANNEL_COUNT]={0};
                uint32_t malformed=0;
                uint16_t mask=0;
                int raw_min[SENSE_CHANNEL_COUNT], raw_max[SENSE_CHANNEL_COUNT];
                uint32_t all_sum[SENSE_CHANNEL_COUNT]={0};
                for (int c=0; c<SENSE_CHANNEL_COUNT; c++) { raw_min[c]=4096; raw_max[c]=-1; }
                for (int c=0; c<SENSE_CHANNEL_COUNT; c++) {
                    s[c].status=CURRENT_CONTINUOUS;
                    s[c].sample_time_us=m.eof_us; /* ISR receipt, not hardware aperture. */
                    s[c].read_us=FRAME_US;       /* Block span, not oneshot call time. */
                }
                for (uint32_t i=0; i<SENSE_FRAME_RESULTS; i++) {
                    adc_digi_output_data_t v={0};
                    memcpy(&v, buf+i*SOC_ADC_DIGI_RESULT_BYTES, SOC_ADC_DIGI_RESULT_BYTES);
                    mask|=(uint16_t)(1U<<(v.type1.channel & 0x0F));
                    int c=channel_to_index[v.type1.channel & 0x0F];
                    if (c < 0) { malformed++; continue; }
                    current_sample_t *p=&s[c];
                    count[c]++;
                    int raw=v.type1.data;
                    if (raw<raw_min[c]) raw_min[c]=raw;
                    if (raw>raw_max[c]) raw_max[c]=raw;
                    all_sum[c]+=(uint32_t)raw;
                    int mv=millivolts[raw];
                    /* Preserve diagnostic raw/voltage even without an estimate. */
                    if (!accepted_n[c]) { p->raw=raw; p->voltage=calibrated ? mv/1000.0f : NAN; }
                    p->converted=true;
                    if (!fresh || !calibrated || raw==4095) { quality_n[c]++; continue; }
                    /* Assign only code zero to 0 V before averaging; preserve SDK voltage
                     * in diagnostics. This is an assignment, not calibration. */
                    float amps=raw==0 ? 0.0f : mv/(1000.0f*SENSE_CS_VOLTS_PER_AMP);
                    raw_sum[c]+=(uint32_t)raw;
                    mv_sum[c]+=raw==0 ? 0U : (uint32_t)mv;
                    accepted_n[c]++;
                    p->raw=raw; p->voltage=mv/1000.0f; p->current_a=amps;
                    p->phase_us=-1;
                    p->status=raw==0 ? CURRENT_CONTINUOUS_ZERO_ASSIGNED : CURRENT_CONTINUOUS;
                }
                portENTER_CRITICAL(&lock);
                bool lost_now=overflows!=seen_overflows;
                portEXIT_CRITICAL(&lock);
                bool bad_frame=malformed>SENSE_MAX_MALFORMED_PER_FRAME;
                diag.malformed_last=malformed; diag.malformed_total+=malformed;
                diag.channel_mask_last=mask;
                for (int c=0; c<SENSE_CHANNEL_COUNT; c++) {
                    current_sample_t *p=&s[c];
                    uint32_t q=quality_n[c], a=accepted_n[c];
                    bool short_block=count[c]<SENSE_MIN_BLOCK_SAMPLES;
                    if (!calibrated) p->status=CURRENT_CALIBRATION_ERROR;
                    else if (bad_frame || short_block) p->status=CURRENT_ADC_ERROR;
                    else if (fresh && q) p->status=CURRENT_ADC_SATURATED;
                    diag.ch[c].count=count[c]; diag.ch[c].saturated=q;
                    diag.ch[c].raw_min=raw_min[c]; diag.ch[c].raw_max=raw_max[c];
                    diag.ch[c].raw_mean=count[c] ? (float)all_sum[c]/count[c] : NAN;
                    if (lost_now || bad_frame || !fresh || q || short_block) {
                        q+=a; a=0;
                        if (lost_now) p->status=CURRENT_DMA_OVERFLOW;
                        else if (bad_frame || !fresh) p->status=CURRENT_ADC_ERROR;
                    } else {
                        /* Means over this channel's actual sample count. */
                        p->adc_raw_mean=(float)raw_sum[c]/a;
                        p->adc_voltage_mean=(float)mv_sum[c]/(1000.0f*a);
                        p->current_mean_a=p->adc_voltage_mean/SENSE_CS_VOLTS_PER_AMP;
                        p->dma_block_samples=a;
                    }
                    counters[c].total+=count[c];
                    counters[c].accepted+=a; counters[c].quality+=q;
                    if (!current_sample_usable(p->status)) invalidate_measurement(p);
                    diag.ch[c].accepted=a; diag.ch[c].status=p->status;
                }
            }
        }
        portENTER_CRITICAL(&lock);
        for (int c=0; c<SENSE_CHANNEL_COUNT; c++) {
            s[c].total_samples=counters[c].total; s[c].accepted_samples=counters[c].accepted;
            s[c].rejected_window=counters[c].window; s[c].rejected_quality=counters[c].quality;
            /* Every received result counts once: unknown-channel results are not
             * attributed, so keep total == accepted + rejected per channel. */
            s[c].total_samples=s[c].accepted_samples+s[c].rejected_window+s[c].rejected_quality;
            s[c].overflows=overflows;
            s[c].generation=latest[c].generation+1;
            latest[c]=s[c];
        }
        portEXIT_CRITICAL(&lock);
    }
}

void current_sense_init(void)
{
    memset(channel_to_index, -1, sizeof(channel_to_index));
    for (int c=0; c<SENSE_CHANNEL_COUNT; c++) {
        channel_to_index[k_channels[c]] = (int8_t)c;
        latest[c]=(current_sample_t){.raw=-1, .voltage=NAN, .current_a=NAN,
            .adc_raw_mean=NAN, .adc_voltage_mean=NAN, .current_mean_a=NAN,
            .phase_us=-1, .status=CURRENT_UNINITIALIZED};
    }
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
    adc_digi_pattern_config_t p[SENSE_CHANNEL_COUNT];
    for (int i=0; i<SENSE_CHANNEL_COUNT; i++)
        p[i]=(adc_digi_pattern_config_t){.atten=ADC_ATTEN_DB_12, .channel=k_channels[i],
            .unit=ADC_UNIT_1, .bit_width=12};
    adc_continuous_config_t cfg={.sample_freq_hz=SENSE_CONTINUOUS_RATE_HZ,
        .conv_mode=ADC_CONV_SINGLE_UNIT_1, .format=ADC_DIGI_OUTPUT_FORMAT_TYPE1,
        .pattern_num=SENSE_CHANNEL_COUNT, .adc_pattern=p};
    ESP_ERROR_CHECK(adc_continuous_config(adc, &cfg));
    adc_continuous_evt_cbs_t cb={.on_pool_ovf=overflow_cb, .on_conv_done=frame_done};
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc, &cb, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(adc));
    BaseType_t ok=xTaskCreatePinnedToCore(adc_task, "adc_dma", 4096, NULL, 12, NULL, 1);
    configASSERT(ok==pdPASS);
}

current_sample_t current_sense_read(int motor)
{
    if (motor < 0 || motor >= SENSE_CHANNEL_COUNT) return empty_sample(CURRENT_ADC_ERROR);
    portENTER_CRITICAL(&lock);
    current_sample_t s=latest[motor];
    portEXIT_CRITICAL(&lock);
    int64_t now=esp_timer_get_time();
    if (now-s.sample_time_us>SENSE_DMA_STALE_US) s.status=CURRENT_ADC_ERROR;
    if (!current_sample_usable(s.status)) invalidate_measurement(&s);
    return s;
}

current_sense_diag_t current_sense_diag(void)
{
    portENTER_CRITICAL(&lock);
    current_sense_diag_t d=diag;
    d.overflows=overflows;
    portEXIT_CRITICAL(&lock);
    d.calibrated=calibrated;
    return d;
}
