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
#define ADC_CHANNEL_0 0
#define ADC_CHANNEL_3 3
#define ADC_CHANNEL_4 4
#define ADC_CHANNEL_5 5
#define ADC_CHANNEL_6 6
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
static int other_raw=100; /* channels 1 and 2 */
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
static inline int adc_continuous_new_handle(const adc_continuous_handle_cfg_t *c,adc_continuous_handle_t *h) { CHECK(c->conv_frame_size%4==0); *h=(void*)1;return 0; } /* ESP32 rule */
static inline int adc_continuous_config(adc_continuous_handle_t h,const adc_continuous_config_t *c) { (void)h;CHECK(c->sample_freq_hz==192000 && c->pattern_num==3);return 0; }
static inline int adc_continuous_register_event_callbacks(adc_continuous_handle_t h,const adc_continuous_evt_cbs_t *c,void *x) { (void)h;(void)c;(void)x;return 0; }
static inline int xTaskCreatePinnedToCore(void (*f)(void*),const char *n,int stack,void *a,int p,void *h,int core) { (void)f;(void)n;(void)stack;(void)a;(void)p;(void)h;(void)core;return 1; }
#include "../main/current_sense.c"
#define N SENSE_DMA_FRAME_SAMPLES
#define UNKNOWN_CH 7           /* not used by either board */
#define HALF (N/2)            /* mode 5 nonzero count; equals odd-index count of mode 6 */

BaseType_t xQueueReceive(QueueHandle_t q,void *out,unsigned wait)
{
    (void)q;(void)wait;
    if (!frames_left--) longjmp(done,1);
    now+=1000;
    *(frame_meta_t*)out=(frame_meta_t){.eof_us=now,.bytes=FRAME_BYTES};
    return 1;
}
/* Channel 0 reproduces every single-motor scenario through its sample index i
 * (0..199). Channels 1/2 carry other_raw. Mode 9 rotates the pattern start,
 * mode 10 saturates only channel 1, mode 11 drops one channel-2 result. */
esp_err_t adc_continuous_read(adc_continuous_handle_t h,uint8_t *buf,uint32_t max,uint32_t *bytes,unsigned timeout)
{
    (void)h;(void)timeout; CHECK(max==FRAME_BYTES);*bytes=FRAME_BYTES;
    const int *ch=k_channels; /* board pattern from pin_map.h */
    unsigned index[3]={0,0,0};
    for (unsigned k=0;k<SENSE_FRAME_RESULTS;k++) {
        unsigned c=(mode==9) ? (k+1)%3 : k%3;
        unsigned i=index[c]++;
        adc_digi_output_data_t v={0};
        if (c==0) {
            v.type1.data=(mode==1 && i==0) ? 1000 : raw_value;
            if (mode==5) v.type1.data=i<HALF ? raw_value : 0;
            if (mode==6) v.type1.data=i%2 ? raw_value : 0;
            if (mode==7 && i==0) v.type1.data=4095;
        } else {
            v.type1.data=other_raw;
            if (mode==10 && c==1 && i==5) v.type1.data=4095;
        }
        v.type1.channel=ch[c];
        if (mode==2 && c==0 && i==N-1) v.type1.channel=UNKNOWN_CH;  /* one unknown result: tolerated */
        if (mode==12 && c==0 && i<10) v.type1.channel=UNKNOWN_CH;   /* ten unknown results: frame rejected */
        if (mode==13 && c==1 && i<30) v.type1.channel=ch[0];   /* ch1 far too short */
        if (mode==11 && c==2 && i==0) v.type1.channel=ch[0];   /* ch2 short, ch0 long */
        memcpy(buf+2*k,&v,2);
    }
    if (mode==4) *bytes=2;
    if (mode==8) now+=SENSE_DMA_STALE_US+1;
    return 0;
}
static void run(unsigned duty,int raw,int test_mode,int count)
{
    (void)duty;raw_value=raw;mode=test_mode;frames_left=count;
    if (!setjmp(done)) adc_task(NULL);
    for (int c=0;c<3;c++)
        CHECK(latest[c].total_samples==latest[c].accepted_samples+latest[c].rejected_window+latest[c].rejected_quality);
}
#define L latest[0]
int main(void)
{
    current_sense_init();
    CHECK(channel_to_index[k_channels[0]]==0 && channel_to_index[k_channels[1]]==1 && channel_to_index[k_channels[2]]==2 && channel_to_index[UNKNOWN_CH]==-1);
    millivolts[100]=100;
    run(2000,249,1,1);
    CHECK(L.raw==249 && L.accepted_samples==N);
    CHECK(L.dma_block_samples==N);
    CHECK(fabsf(L.adc_raw_mean-(1000+(N-1)*249)/(float)N)<1e-4f);
    CHECK(fabsf(L.adc_voltage_mean-(1000+(N-1)*249)/(1000.0f*N))<1e-6f);
    CHECK(fabsf(L.current_mean_a-L.adc_voltage_mean/SENSE_CS_VOLTS_PER_AMP)<1e-6f);
    CHECK(L.current_mean_a>L.current_a); /* Includes the first sample. */
    for (int c=1;c<3;c++) { /* Other channels are independent. */
        CHECK(latest[c].dma_block_samples==N && fabsf(latest[c].adc_raw_mean-100.0f)<1e-5f);
        CHECK(fabsf(latest[c].current_mean_a-.100f/SENSE_CS_VOLTS_PER_AMP)<1e-6f);
    }
    run(0,149,0,1); CHECK(L.status==CURRENT_CONTINUOUS && L.accepted_samples==N);
    CHECK(fabsf(L.current_mean_a-.149f/SENSE_CS_VOLTS_PER_AMP)<1e-6f); /* No ON/OFF or floor censor. */
    run(1000,150,0,1); CHECK(L.status==CURRENT_CONTINUOUS);
    millivolts[0]=142; /* Reproduce real SDK intercept even at raw zero. */
    run(2000,0,0,1);
    CHECK(L.status==CURRENT_CONTINUOUS_ZERO_ASSIGNED && L.current_a==0);
    CHECK(fabsf(L.voltage-.142f)<1e-6f && L.raw==0);
    CHECK(L.adc_raw_mean==0 && L.adc_voltage_mean==0 && L.current_mean_a==0);
    millivolts[1]=143;
    run(0,1,0,1); CHECK(L.current_a>0 && L.status==CURRENT_CONTINUOUS);
    CHECK(fabsf(L.adc_voltage_mean-.143f)<1e-6f); /* Positive codes retain SDK calibration. */
    millivolts[249]=500;
    run(600,249,5,1);
    CHECK(L.status==CURRENT_CONTINUOUS_ZERO_ASSIGNED && L.raw==0);
    CHECK(L.dma_block_samples==N && fabsf(L.adc_raw_mean-HALF*249.0f/N)<1e-4f);
    CHECK(fabsf(L.adc_voltage_mean-HALF*0.5f/N)<1e-6f);
    CHECK(fabsf(L.current_mean_a-HALF*0.5f/N/SENSE_CS_VOLTS_PER_AMP)<1e-6f);
    float pulse_mean=L.current_mean_a;
    run(600,249,6,1);
    CHECK(fabsf(L.current_mean_a-pulse_mean)<1e-6f);
    CHECK((N-1)%2 ? (L.status==CURRENT_CONTINUOUS && L.raw==249) : (L.status==CURRENT_CONTINUOUS_ZERO_ASSIGNED && L.raw==0));
    millivolts[249]=249;
    run(2000,249,0,1);
    CHECK(fabsf(L.current_mean_a-.249f/SENSE_CS_VOLTS_PER_AMP)<1e-6f); /* No previous-block samples in mean. */
    run(2000,249,9,1); /* Pattern starting at another channel gives identical means. */
    CHECK(fabsf(L.current_mean_a-.249f/SENSE_CS_VOLTS_PER_AMP)<1e-6f && L.dma_block_samples==N);
    CHECK(latest[1].dma_block_samples==N && latest[2].dma_block_samples==N);
    uint32_t rq=L.rejected_quality;
    run(600,249,7,1); /* One saturated sample invalidates the whole block of that channel. */
    CHECK(L.status==CURRENT_ADC_SATURATED && L.rejected_quality-rq==N && L.dma_block_samples==0);
    CHECK(isnan(L.adc_raw_mean) && isnan(L.adc_voltage_mean) && isnan(L.current_mean_a));
    CHECK(latest[1].dma_block_samples==N && current_sample_usable(latest[1].status));
    run(600,249,10,1); /* Saturation on channel 1 only. */
    CHECK(latest[1].status==CURRENT_ADC_SATURATED && isnan(latest[1].current_mean_a));
    CHECK(L.dma_block_samples==N && latest[2].dma_block_samples==N);
    run(600,249,11,1); /* N+1/N/N-1 split is tolerated; means use actual counts. */
    CHECK(L.dma_block_samples==N+1 && latest[2].dma_block_samples==N-1 && latest[1].dma_block_samples==N);
    CHECK(fabsf(latest[2].adc_raw_mean-100.0f)<1e-6f);
    run(600,249,13,1); /* ch1 30 short: below the minimum block. */
    CHECK(latest[1].status==CURRENT_ADC_ERROR && isnan(latest[1].current_mean_a));
    CHECK(current_sample_usable(L.status) && latest[2].dma_block_samples==N);
    CHECK(current_sense_diag().ch[1].count==N-30 && current_sense_diag().ch[0].count==N+30);
    run(2000,4095,0,1); CHECK(L.status==CURRENT_ADC_SATURATED && isnan(L.current_a));
    run(2000,249,2,1); /* One unknown-channel result is ignored. */
    CHECK(L.dma_block_samples==N-1 && current_sample_usable(L.status) && current_sense_diag().malformed_last==1);
    { unsigned want=(1u<<k_channels[0])|(1u<<k_channels[1])|(1u<<k_channels[2])|(1u<<UNKNOWN_CH);
      CHECK((current_sense_diag().channel_mask_last & want)==want); }
    run(2000,249,12,1); /* Ten unknown results: whole frame unusable. */
    for (int c=0;c<3;c++) CHECK(latest[c].status==CURRENT_ADC_ERROR && isnan(latest[c].current_mean_a) && latest[c].dma_block_samples==0);
    run(600,249,8,1);
    for (int c=0;c<3;c++) CHECK(latest[c].status==CURRENT_ADC_ERROR && isnan(latest[c].current_mean_a));
    run(2000,249,4,1); CHECK(flushes>0 && stops>0 && starts>1);
    for (int c=0;c<3;c++) CHECK(isnan(latest[c].current_mean_a) && latest[c].dma_block_samples==0);
    run(600,249,0,1);
    CHECK(current_sense_read(2).dma_block_samples==N);
    CHECK(current_sense_read(3).status==CURRENT_ADC_ERROR); /* out of range motor */
    now+=SENSE_DMA_STALE_US+1;
    current_sample_t stale=current_sense_read(0);
    CHECK(stale.status==CURRENT_ADC_ERROR && isnan(stale.current_mean_a) && stale.dma_block_samples==0);
    puts("PASS: 3-channel DMA block means, routing by channel, per-channel saturation, loss, stale data");
    return 0;
}
