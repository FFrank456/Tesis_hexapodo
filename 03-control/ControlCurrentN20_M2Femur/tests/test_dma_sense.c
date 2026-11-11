/* Deterministic host execution of the production DMA task. The fake SDK models
 * frames, changes, calibration and queue loss, not physical aperture accuracy. */
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <setjmp.h>
#define CURRENT_SENSE_HOST_TEST
#define IRAM_ATTR
#define SOC_ADC_DIGI_RESULT_BYTES 2
#define ADC_CHANNEL_4 4
#define ADC_CHANNEL_3 3
#define ADC_ATTEN_DB_12 12
#define ADC_BITWIDTH_12 12
#define ADC_UNIT_1 0
#define ADC_CONV_SINGLE_UNIT_1 0
#define ADC_DIGI_OUTPUT_FORMAT_TYPE1 0
#define ESP_OK 0
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define portENTER_CRITICAL_ISR(x) ((void)(x))
#define portEXIT_CRITICAL_ISR(x) ((void)(x))
#define pdMS_TO_TICKS(x) (x)
#define CHECK(c) do { if (!(c)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); exit(1); } } while (0)
#define configASSERT(c) CHECK(c)
#define ESP_ERROR_CHECK(e) CHECK((e)==0)
typedef int esp_err_t, BaseType_t, portMUX_TYPE;
typedef void *adc_continuous_handle_t, *adc_cali_handle_t, *QueueHandle_t;
typedef union { struct { uint16_t data:12, channel:4; } type1; } adc_digi_output_data_t;
typedef struct { uint8_t *conv_frame_buffer; uint32_t size; } adc_continuous_evt_data_t;
typedef struct { int unit_id, atten, bitwidth, default_vref; } adc_cali_line_fitting_config_t;
typedef struct { int max_store_buf_size, conv_frame_size; } adc_continuous_handle_cfg_t;
typedef struct { int atten, channel, unit, bit_width; } adc_digi_pattern_config_t;
typedef struct { int sample_freq_hz, conv_mode, format, pattern_num; adc_digi_pattern_config_t *adc_pattern; } adc_continuous_config_t;
typedef struct { bool (*on_pool_ovf)(adc_continuous_handle_t,const adc_continuous_evt_data_t*,void*);
    bool (*on_conv_done)(adc_continuous_handle_t,const adc_continuous_evt_data_t*,void*); } adc_continuous_evt_cbs_t;
static int64_t now=10000;
static int frames_left, raw_value=249, mode, stops, starts, flushes;
static jmp_buf done;
int64_t esp_timer_get_time(void) { return now; }
BaseType_t xQueueReceive(QueueHandle_t, void *, unsigned);
esp_err_t adc_continuous_read(adc_continuous_handle_t,uint8_t *,uint32_t,uint32_t *,unsigned);
static inline BaseType_t xQueueSendFromISR(QueueHandle_t q,const void *m,BaseType_t *w) { (void)q;(void)m;(void)w;return 1; }
static inline void xQueueReset(QueueHandle_t q) { (void)q; }
static inline QueueHandle_t xQueueCreate(unsigned n,unsigned s) { (void)n;(void)s;return (void*)1; }
static inline int adc_continuous_stop(adc_continuous_handle_t h) { (void)h;stops++;return 0; }
static inline int adc_continuous_start(adc_continuous_handle_t h) { (void)h;starts++;return 0; }
static inline int adc_continuous_flush_pool(adc_continuous_handle_t h) { (void)h;flushes++;return 0; }
static inline int adc_cali_create_scheme_line_fitting(const adc_cali_line_fitting_config_t *c,adc_cali_handle_t *h) { (void)c;*h=(void*)1;return 0; }
static inline int adc_cali_raw_to_voltage(adc_cali_handle_t h,int raw,int *mv) { (void)h;*mv=raw;return 0; }
static inline int adc_continuous_new_handle(const adc_continuous_handle_cfg_t *c,adc_continuous_handle_t *h) { (void)c;*h=(void*)1;return 0; }
static inline int adc_continuous_config(adc_continuous_handle_t h,const adc_continuous_config_t *c) { (void)h;CHECK(c->sample_freq_hz==200000);return 0; }
static inline int adc_continuous_register_event_callbacks(adc_continuous_handle_t h,const adc_continuous_evt_cbs_t *c,void *x) { (void)h;(void)c;(void)x;return 0; }
static inline int xTaskCreatePinnedToCore(void (*f)(void*),const char *n,int stack,void *a,int p,void *h,int core) { (void)f;(void)n;(void)stack;(void)a;(void)p;(void)h;(void)core;return 1; }
#include "../main/current_sense.c"

BaseType_t xQueueReceive(QueueHandle_t q,void *out,unsigned wait)
{
    (void)q;(void)wait;
    if (!frames_left--) longjmp(done,1);
    now+=1000;
    *(frame_meta_t*)out=(frame_meta_t){.eof_us=now,.bytes=FRAME_BYTES};
    return 1;
}
esp_err_t adc_continuous_read(adc_continuous_handle_t h,uint8_t *buf,uint32_t max,uint32_t *bytes,unsigned timeout)
{
    (void)h;(void)timeout; CHECK(max==FRAME_BYTES);*bytes=FRAME_BYTES;
    for (unsigned i=0;i<SENSE_DMA_FRAME_SAMPLES;i++) {
        adc_digi_output_data_t v={0};
        v.type1.data=(mode==1 && i==0) ? 1000 : raw_value;
        if (mode==5) v.type1.data=i<100 ? raw_value : 0;
        if (mode==6) v.type1.data=i%2 ? raw_value : 0;
        if (mode==7 && i==0) v.type1.data=4095;
        v.type1.channel=(mode==2 && i==199) ? 4 : SENSE_ADC_CHANNEL;
        memcpy(buf+2*i,&v,2);
    }
    if (mode==4) *bytes=2;
    if (mode==8) now+=SENSE_DMA_STALE_US+1;
    return 0;
}
static void run(unsigned duty,int raw,int test_mode,int count)
{
    (void)duty;raw_value=raw;mode=test_mode;frames_left=count;
    if (!setjmp(done)) adc_task(NULL);
    CHECK(latest.total_samples==latest.accepted_samples+latest.rejected_window+latest.rejected_quality);
}
int main(void)
{
    current_sense_init();
    run(2000,249,1,1);
    CHECK(latest.raw==249 && latest.accepted_samples==200);
    CHECK(latest.dma_block_samples==200);
    CHECK(fabsf(latest.adc_raw_mean-(1000+199*249)/200.0f)<1e-5f);
    CHECK(fabsf(latest.adc_voltage_mean-(1000+199*249)/200000.0f)<1e-6f);
    CHECK(fabsf(latest.current_mean_a-latest.adc_voltage_mean/SENSE_CS_VOLTS_PER_AMP)<1e-6f);
    CHECK(latest.current_mean_a>latest.current_a); /* Includes the first sample. */
    run(0,149,0,1); CHECK(latest.status==CURRENT_CONTINUOUS && latest.accepted_samples==200);
    CHECK(fabsf(latest.current_mean_a-.149f/SENSE_CS_VOLTS_PER_AMP)<1e-6f); /* No ON/OFF or floor censor. */
    run(1000,150,0,1); CHECK(latest.status==CURRENT_CONTINUOUS && latest.accepted_samples==200);
    millivolts[0]=142; /* Reproduce real SDK intercept even at raw zero. */
    run(2000,0,0,1);
    CHECK(latest.status==CURRENT_CONTINUOUS_ZERO_ASSIGNED && latest.current_a==0);
    CHECK(fabsf(latest.voltage-.142f)<1e-6f && latest.raw==0);
    CHECK(latest.adc_raw_mean==0 && latest.adc_voltage_mean==0 && latest.current_mean_a==0);
    millivolts[1]=143;
    run(0,1,0,1); CHECK(latest.current_a>0 && latest.status==CURRENT_CONTINUOUS);
    CHECK(fabsf(latest.adc_voltage_mean-.143f)<1e-6f); /* Positive codes retain SDK calibration. */
    /* Same 100 nonzero + 100 zero samples, different order and final sample.
     * Means must agree despite a different last-value diagnostic. */
    millivolts[249]=500;
    run(600,249,5,1);
    CHECK(latest.status==CURRENT_CONTINUOUS_ZERO_ASSIGNED && latest.raw==0);
    CHECK(latest.dma_block_samples==200 && fabsf(latest.adc_raw_mean-124.5f)<1e-6f);
    CHECK(fabsf(latest.adc_voltage_mean-.250f)<1e-6f);
    CHECK(fabsf(latest.current_mean_a-.250f/SENSE_CS_VOLTS_PER_AMP)<1e-6f);
    float pulse_mean=latest.current_mean_a;
    run(600,249,6,1);
    CHECK(fabsf(latest.current_mean_a-pulse_mean)<1e-6f);
    CHECK(latest.status==CURRENT_CONTINUOUS && latest.raw==249);
    millivolts[249]=249;
    run(2000,249,0,1);
    CHECK(fabsf(latest.current_mean_a-.249f/SENSE_CS_VOLTS_PER_AMP)<1e-6f); /* No previous-block samples in mean. */
    run(600,249,7,1); /* One saturated sample invalidates the whole block. */
    CHECK(latest.status==CURRENT_ADC_SATURATED && latest.accepted_samples==0);
    CHECK(latest.rejected_quality==200 && latest.dma_block_samples==0);
    CHECK(isnan(latest.adc_raw_mean) && isnan(latest.adc_voltage_mean) && isnan(latest.current_mean_a));
    run(2000,4095,0,1); CHECK(latest.status==CURRENT_ADC_SATURATED && isnan(latest.current_a));
    run(2000,249,2,1); CHECK(latest.status==CURRENT_ADC_ERROR);
    CHECK(isnan(latest.current_mean_a) && latest.dma_block_samples==0);
    run(600,249,8,1);
    CHECK(latest.status==CURRENT_ADC_ERROR && isnan(latest.current_mean_a));
    run(2000,249,4,1); CHECK(flushes>0 && stops>0 && starts>1);
    CHECK(isnan(latest.current_mean_a) && latest.dma_block_samples==0);
    run(600,249,0,1);
    now+=SENSE_DMA_STALE_US+1;
    current_sample_t stale=current_sense_read(600,27,26,0);
    CHECK(stale.status==CURRENT_ADC_ERROR && isnan(stale.current_mean_a) && stale.dma_block_samples==0);
    puts("PASS: DMA complete-block mean, zero assignment, sample-order independence, saturation, loss, stale data");
    return 0;
}
