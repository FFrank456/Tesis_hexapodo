/* Execute the production sensing module against a deterministic PWM clock and
 * ADC API. These tests exercise scheduling/window rejection, not ADC accuracy. */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "current_sense.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"

/* Avoid the Windows CRT assertion dialog: failed checks must end unattended. */
#undef assert
#define assert(condition) do { if (!(condition)) { \
    fprintf(stderr, "Check failed: %s (%s:%d)\n", #condition, __FILE__, __LINE__); \
    exit(1); \
} } while (0)

enum { ACTIVE_GPIO = 27, INACTIVE_GPIO = 26 };

static struct {
    int64_t now;
    uint32_t duty_ticks;
    uint32_t conversion_us;
    int adc_raw;
    int voltage_mv;
    esp_err_t adc_error;
    bool calibration_init_error;
    bool calibration_error;
    bool active_stuck_low;
    bool active_stuck_high;
    bool inactive_high;
    bool saw_low;
    bool preempt_first_edge;
    uint32_t clock_calls;
    uint32_t adc_calls;
    int64_t adc_begin;
    int64_t adc_end;
    bool adc_begin_high;
    bool adc_end_high;
} sim;

static bool active_high(void)
{
    if (sim.active_stuck_low) return false;
    if (sim.active_stuck_high) return true;
    if (sim.duty_ticks >= SENSE_PWM_PERIOD_TICKS) return true;
    const int64_t phase = sim.now % SENSE_PWM_PERIOD_US;
    return phase * SENSE_PWM_PERIOD_TICKS <
           (int64_t)sim.duty_ticks * SENSE_PWM_PERIOD_US;
}

int64_t esp_timer_get_time(void)
{
    /* Each clock read costs one microsecond in this deliberately conservative
     * simulation. Watchdog catches accidental unbounded polling in host tests. */
    assert(++sim.clock_calls < 10000);
    return sim.now++;
}

int gpio_get_level(int gpio)
{
    assert(gpio == ACTIVE_GPIO || gpio == INACTIVE_GPIO);
    if (gpio == INACTIVE_GPIO) return sim.inactive_high;
    const bool high = active_high();
    if (!high) sim.saw_low = true;
    if (high && sim.saw_low && sim.preempt_first_edge) {
        /* The GPIO was HIGH before preemption and HIGH again one period later.
         * An endpoint-only validator would silently accept this delayed edge. */
        sim.now += SENSE_PWM_PERIOD_US;
        sim.preempt_first_edge = false;
    }
    return high;
}

esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t *config,
                             adc_oneshot_unit_handle_t *handle)
{
    assert(config->unit_id == ADC_UNIT_1);
    *handle = &sim;
    return ESP_OK;
}

esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t handle, int channel,
                                    const adc_oneshot_chan_cfg_t *config)
{
    assert(handle == &sim && channel == ADC_CHANNEL_4);
    assert(config->atten == ADC_ATTEN_DB_12 && config->bitwidth == ADC_BITWIDTH_12);
    return ESP_OK;
}

esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t handle, int channel, int *raw)
{
    assert(handle == &sim && channel == ADC_CHANNEL_4);
    sim.adc_calls++;
    sim.adc_begin = sim.now;
    sim.adc_begin_high = active_high();
    sim.now += sim.conversion_us;
    sim.adc_end = sim.now;
    sim.adc_end_high = active_high();
    *raw = sim.adc_raw;
    return sim.adc_error;
}

esp_err_t adc_cali_create_scheme_line_fitting(const adc_cali_line_fitting_config_t *config,
                                             adc_cali_handle_t *handle)
{
    assert(config->unit_id == ADC_UNIT_1 && config->default_vref == 1100);
    *handle = &sim;
    return sim.calibration_init_error ? ESP_FAIL : ESP_OK;
}

esp_err_t adc_cali_raw_to_voltage(adc_cali_handle_t handle, int raw, int *voltage_mv)
{
    assert(handle == &sim && raw == sim.adc_raw);
    *voltage_mv = sim.voltage_mv;
    return sim.calibration_error ? ESP_FAIL : ESP_OK;
}

static void setup(uint32_t duty, int64_t start)
{
    memset(&sim, 0, sizeof(sim));
    sim.duty_ticks = duty;
    sim.now = start;
    sim.conversion_us = 3;
    sim.adc_raw = 500;
    sim.voltage_mv = 249;
    current_sense_init();
}

static current_sample_t read_sample(void)
{
    return current_sense_read(sim.duty_ticks, ACTIVE_GPIO, INACTIVE_GPIO, -1000);
}

static void assert_invalid(current_sample_t s, current_status_t status)
{
    assert(s.status == status);
    assert(isnan(s.current_a));
}

static void test_window_predicate(void)
{
    /* All possible integer-microsecond starts/ends near a bracketed edge; also
     * try onset brackets spanning a missed period and negative/reversed time. */
    uint32_t checked = 0;
    for (int lower = -1; lower <= 2; lower++) {
        for (int upper = -1; upper <= 105; upper++) {
            for (int on = 0; on <= 100; on += 5) {
                for (int begin = -2; begin <= 110; begin++) {
                    for (int duration = -1; duration <= 105; duration += 7) {
                        int end = begin + duration;
                        bool expected = lower >= 0 && upper >= lower && duration >= 0 &&
                                        begin - upper >= 5 && lower + on - end >= 2;
                        assert(current_window_contains(lower, upper, (uint32_t)on,
                                                       begin, end) == expected);
                        checked++;
                    }
                }
            }
        }
    }
    printf("Window bounds: %u cases passed\n", checked);
}

static void test_asynchronous_wake_phases(void)
{
    const uint32_t duties[] = {600, 1200, 2000}; /* 30 %, 60 %, 100 % */
    for (unsigned d = 0; d < sizeof(duties) / sizeof(duties[0]); d++) {
        for (int phase = 0; phase < 100; phase++) {
            setup(duties[d], 1000 + phase);
            current_sample_t s = read_sample();
            assert(s.status == CURRENT_VALID_ON);
            assert(s.converted && s.calibrated && s.raw == 500);
            assert(sim.adc_calls == 1);
            assert(fabsf(s.current_a - 0.1f) < 0.000001f);
            assert(sim.adc_begin_high && sim.adc_end_high);
            assert(s.sample_time_us == sim.adc_end + 1); /* Round-up API bound. */
            if (duties[d] < SENSE_PWM_PERIOD_TICKS) {
                assert(s.phase_us >= (int32_t)SENSE_SETTLE_US);
                assert(sim.adc_begin / 100 == sim.adc_end / 100);
                assert(sim.adc_begin % 100 >= SENSE_SETTLE_US);
                assert(sim.adc_begin % 100 >= (int64_t)duties[d] / 40);
                assert(sim.adc_begin % 100 <= (int64_t)duties[d] / 40 + 8);
                assert(sim.adc_end % 100 <= (int64_t)duties[d] / 20 - SENSE_END_GUARD_US);
            }
        }
    }
    puts("30/60/100 percent: centred API start, all 100 wake phases passed");
}

static void test_invalid_windows(void)
{
    setup(0, 1000);
    current_sample_t s = read_sample();
    assert_invalid(s, CURRENT_PWM_OFF);
    assert(s.converted && s.raw == 500); /* Preserve useful raw diagnostic. */

    setup(100, 1000); /* 5 % cannot contain settle+end guard. */
    assert_invalid(read_sample(), CURRENT_NO_WINDOW);
    assert(sim.adc_calls == 0);

    setup(800, 1000);
    s = current_sense_read(sim.duty_ticks, ACTIVE_GPIO, INACTIVE_GPIO, 1000);
    assert_invalid(s, CURRENT_OUTPUT_SETTLING);
    assert(sim.adc_calls == 0);

    setup(800, 1000);
    sim.conversion_us = 35; /* Begin in ON, but finish after the falling edge. */
    assert_invalid(read_sample(), CURRENT_NO_WINDOW);
    assert(sim.adc_calls == 1 && sim.adc_begin_high && !sim.adc_end_high);

    setup(800, 1000);
    sim.conversion_us = 103; /* Both endpoints HIGH across a complete period. */
    assert_invalid(read_sample(), CURRENT_NO_WINDOW);
    assert(sim.adc_calls == 1 && sim.adc_begin_high && sim.adc_end_high);

    setup(800, 1050);
    sim.preempt_first_edge = true;
    assert_invalid(read_sample(), CURRENT_NO_WINDOW);
    assert(sim.adc_calls == 0);

    setup(800, 1000);
    sim.active_stuck_high = true; /* Never observes LOW: no known rising edge. */
    assert_invalid(read_sample(), CURRENT_NO_WINDOW);
    assert(sim.adc_calls == 0 && sim.now < 1300);

    setup(800, 1000);
    sim.active_stuck_low = true;
    assert_invalid(read_sample(), CURRENT_NO_WINDOW);
    assert(sim.adc_calls == 0 && sim.now < 1300);

    setup(2000, 1000);
    sim.inactive_high = true;
    assert_invalid(read_sample(), CURRENT_NO_WINDOW);
    assert(sim.adc_calls == 0);

    setup(2000, 1000);
    sim.conversion_us = SENSE_WAIT_TIMEOUT_US + 1;
    assert_invalid(read_sample(), CURRENT_NO_WINDOW);
    puts("OFF, settling, short/missed/preempted windows and stuck outputs passed");
}

static void test_adc_classification(void)
{
    setup(2000, 1000);
    sim.adc_raw = 0;
    sim.voltage_mv = 142; /* The former spurious 0.057 A at ADC floor. */
    current_sample_t s = read_sample();
    assert(s.status==CURRENT_BELOW_RANGE && s.current_a==0 && s.ema_a==0);
    assert(s.converted && s.calibrated && fabsf(s.voltage - 0.142f) < 0.000001f);

    setup(2000, 1000);
    sim.adc_raw = 4095;
    assert_invalid(read_sample(), CURRENT_ADC_SATURATED);

    setup(2000, 1000);
    sim.adc_error = ESP_ERR_TIMEOUT;
    s = read_sample();
    assert_invalid(s, CURRENT_ADC_ERROR);
    assert(!s.converted && !s.calibrated && s.raw == -1);

    setup(2000, 1000);
    sim.calibration_error = true;
    s = read_sample();
    assert_invalid(s, CURRENT_CALIBRATION_ERROR);
    assert(s.converted && !s.calibrated && isnan(s.voltage));

    setup(2000, 1000);
    sim.calibration_init_error = true;
    current_sense_init();
    s = read_sample();
    assert_invalid(s, CURRENT_CALIBRATION_ERROR);
    assert(!s.calibrated);

    setup(2000, 1000);
    sim.voltage_mv = 498;
    s = read_sample();
    assert(s.status == CURRENT_VALID_ON);
    assert(fabsf(s.current_a - 0.2f) < 0.000001f); /* No blanket 0.057 A removal. */
    puts("ADC floor/saturation/errors/calibration and current scaling passed");
}

static void test_synchronized_ema(void)
{
    setup(2000,1000);
    current_sample_t first=read_sample();
    assert(fabsf(first.ema_a-0.1f)<1e-6f);
    sim.now+=2000;
    sim.voltage_mv=498;
    current_sample_t second=read_sample();
    float alpha=SENSE_EMA_NOMINAL_ALPHA;
    assert(fabsf(second.ema_a-(0.1f+alpha*0.1f))<1e-6f);
    assert(second.total_samples==2);
    sim.adc_raw=0;
    current_sample_t below=read_sample();
    assert(below.status==CURRENT_BELOW_RANGE && below.current_a==0);
    assert(fabsf(below.ema_a-second.ema_a*0.9f)<1e-6f);
    sim.adc_error=ESP_ERR_TIMEOUT;
    assert_invalid(read_sample(),CURRENT_ADC_ERROR);
    sim.adc_error=ESP_OK;
    sim.now+=100000; /* Long missing interval must not erase EMA memory. */
    sim.adc_raw=500;
    sim.voltage_mv=747;
    current_sample_t recovered=read_sample();
    assert(fabsf(recovered.ema_a-(below.ema_a*0.9f+0.03f))<1e-6f);
    assert(recovered.total_samples==4);
    assert(recovered.overflows==0);
    /* Output change/settling, OFF and a new duty all preserve the EMA. */
    current_sample_t settling=current_sense_read(1000,ACTIVE_GPIO,INACTIVE_GPIO,sim.now);
    assert_invalid(settling,CURRENT_OUTPUT_SETTLING);
    assert_invalid(current_sense_read(0,ACTIVE_GPIO,INACTIVE_GPIO,-1000),CURRENT_PWM_OFF);
    sim.now+=1000;
    sim.duty_ticks=1200;
    sim.voltage_mv=249;
    current_sample_t changed=read_sample();
    assert(changed.status==CURRENT_VALID_ON);
    assert(fabsf(changed.ema_a-(recovered.ema_a*0.9f+0.01f))<1e-6f);
    puts("EMA holds through gaps/OFF/PWM change; below-range zero decays EMA");
}

static void test_thresholds(void)
{
    const int volts[]={124,149,150,151,2117};
    for (unsigned i=0;i<sizeof(volts)/sizeof(volts[0]);i++) {
        setup(2000,1000); sim.voltage_mv=volts[i];
        current_sample_t s=read_sample();
        assert(s.status==(volts[i]<150 ? CURRENT_BELOW_RANGE : CURRENT_VALID_ON));
        assert(current_sample_usable(s.status));
        assert(fabsf(s.current_a-(volts[i]<150 ? 0.0f : s.voltage/2.49f))<1e-6f);
        assert(s.raw==500 && s.calibrated);
    }
    setup(800,1000); sim.conversion_us=44; sim.voltage_mv=124;
    assert_invalid(read_sample(),CURRENT_NO_WINDOW); /* Invalid time is NOT zero. */
    setup(2000,1000); sim.calibration_error=true; sim.voltage_mv=124;
    assert_invalid(read_sample(),CURRENT_CALIBRATION_ERROR);
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    test_window_predicate();
    test_asynchronous_wake_phases();
    test_invalid_windows();
    test_adc_classification();
    test_synchronized_ema();
    test_thresholds();
    puts("All production current_sense.c host tests passed");
    return 0;
}
