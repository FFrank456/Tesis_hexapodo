/* Host test of pata_banco1 main.c: command routing for three motors, motor
 * prefixes, ALL, SEL, global STOP, ESIGN, PWM pin mapping and an independent
 * cascade (POSITION on one motor, fault on another) with fake feedback.
 * Build: gcc -std=gnu11 -Itests/host_stubs -Imain tests/test_main_host.c main/pwm_sequence.c -lm */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <math.h>
#include "encoder.h"
#include "current_sense.h"

static int64_t now_us = 1000000;
int64_t esp_timer_get_time(void) { return now_us; }

/* ---- fake MCPWM ---- */
#include "freertos/FreeRTOS.h"
#include "driver/mcpwm_prelude.h"
static struct fake_gen gens[8]; static int ngen;
static struct fake_cmp cmps[8]; static int ncmp;
static int nops, timer_cb_registered;
int mcpwm_new_timer(const mcpwm_timer_config_t *c, mcpwm_timer_handle_t *t) { (void)c; *t=(void*)1; return 0; }
int mcpwm_new_operator(const mcpwm_operator_config_t *c, mcpwm_oper_handle_t *o) { if (c->group_id!=0) return 1; nops++; *o=(void*)(intptr_t)nops; return nops>3; }
int mcpwm_operator_connect_timer(mcpwm_oper_handle_t o, mcpwm_timer_handle_t t) { (void)o; (void)t; return 0; }
int mcpwm_new_comparator(mcpwm_oper_handle_t o, const mcpwm_comparator_config_t *c, mcpwm_cmpr_handle_t *h) { (void)o; (void)c; *h=&cmps[ncmp++]; return 0; }
int mcpwm_new_generator(mcpwm_oper_handle_t o, const mcpwm_generator_config_t *c, mcpwm_gen_handle_t *h) { (void)o; gens[ngen].gpio=c->gen_gpio_num; *h=&gens[ngen++]; return 0; }
int mcpwm_comparator_set_compare_value(mcpwm_cmpr_handle_t c, uint32_t v) { c->value=v; return 0; }
int mcpwm_generator_set_force_level(mcpwm_gen_handle_t g, int level, bool hold) { (void)hold; g->force=level; return 0; }
int mcpwm_timer_register_event_callbacks(mcpwm_timer_handle_t t, const mcpwm_timer_event_callbacks_t *c, void *u) { (void)t; (void)u; timer_cb_registered = c->on_empty != NULL; return 0; }
int mcpwm_timer_enable(mcpwm_timer_handle_t t) { (void)t; return 0; }
int mcpwm_timer_start_stop(mcpwm_timer_handle_t t, int c) { (void)t; (void)c; return 0; }
BaseType_t xTaskCreatePinnedToCore(void (*f)(void*), const char *n, int s, void *a, int p, TaskHandle_t *h, int c)
{ (void)f; (void)n; (void)s; (void)a; (void)p; (void)c; if (h) *h=(void*)1; return 1; }

/* ---- fake GPIO (nFAULT) ---- */
#include "driver/gpio.h"
static int gpio_level[40]; static uint64_t gpio_configured;
int gpio_config(const gpio_config_t *c) { gpio_configured |= c->pin_bit_mask; return 0; }
int gpio_get_level(int g) { return gpio_level[g]; }

/* ---- fake encoders and current sense ---- */
static double pos[3], spd[3]; static bool spd_valid[3] = {true, true, true};
static uint32_t ref_id[3]; static int zero_calls[3], sign_[3] = {1, 1, 1};
static double ref_pos[3];
encoder_sample_t encoder_sample(int m)
{
    return (encoder_sample_t){ .count_x4=(int64_t)(pos[m]*2800/6.283185307), .time_us=now_us,
        .output_position_rad=pos[m]-ref_pos[m], .output_speed_rad_s=spd_valid[m] ? spd[m] : NAN,
        .speed_valid=spd_valid[m], .reference_id=ref_id[m], .speed_window_us=50000 };
}
void encoder_zero_reference(int m) { ref_pos[m]=pos[m]; ref_id[m]++; zero_calls[m]++; }
void encoder_set_sign(int m, int s) { sign_[m]=s; ref_id[m]++; }
int encoder_get_sign(int m) { return sign_[m]; }
void encoder_init(void) {}
static float amps[3];
current_sample_t current_sense_read(int m)
{
    return (current_sample_t){ .status=CURRENT_CONTINUOUS, .dma_block_samples=SENSE_DMA_FRAME_SAMPLES,
        .current_mean_a=amps[m], .sample_time_us=now_us, .calibrated=true, .raw=100 };
}
void current_sense_init(void) {}
current_sense_diag_t current_sense_diag(void) { current_sense_diag_t d = {0}; return d; }

#define app_main app_main_unused
#include "../main/main.c"
#undef app_main

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); exit(1); } } while (0)

static char out[65536];
static FILE *capture;
static const char *cmd(const char *text)
{
    char line[COMMAND_BUFFER_SIZE];
    snprintf(line, sizeof(line), "%s", text);
    fflush(stdout);
    long start = lseek(STDOUT_FILENO, 0, SEEK_CUR);
    handle_serial_command(line);
    fflush(stdout);
    long end = lseek(STDOUT_FILENO, 0, SEEK_CUR);
    fseek(capture, start, SEEK_SET);
    size_t n = fread(out, 1, (size_t)(end - start), capture);
    out[n] = '\0';
    return out;
}

static double grav_dist[3]; /* gravity as an equivalent speed bias, rad/s * cos(pos) */
static void step(int ms)
{
    control_rates_t rates = {0};
    for (int k = 0; k < ms; ++k) {
        now_us += 1000;
        for (int i = 0; i < MOTOR_COUNT; ++i) motor_control_step(&g_motors[i], now_us, &rates);
        for (int i = 0; i < MOTOR_COUNT; ++i) { /* first-order plant per motor */
            float duty = g_motors[i].duty_percent;
            double g = grav_dist[i] * cos(pos[i] - ref_pos[i]);
            if (g_motors[i].direction == MOTOR_BRAKE) spd[i] = spd[i] * 0.8 - g * 0.02; /* brake: slides under load */
            else spd[i] += (duty * 0.1 - g - spd[i]) * 0.05;
            pos[i] += spd[i] * 0.001;
            amps[i] = fabsf(duty) / 100.0f * 0.3f;
        }
    }
}

int main(void)
{
    int fd = open("/tmp/pata_banco1_host_out.txt", O_RDWR | O_CREAT | O_TRUNC, 0600);
    CHECK(fd >= 0);
    fflush(stdout); dup2(fd, STDOUT_FILENO);
    capture = fopen("/tmp/pata_banco1_host_out.txt", "r"); setvbuf(capture, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    for (int i = 0; i < 40; ++i) gpio_level[i] = 1; /* nFAULT pulled up */
    for (int i = 0; i < MOTOR_COUNT; ++i) motor_init_state(&g_motors[i], i);
    mcpwm_init();
    CHECK(nops == 3 && ngen == 6 && ncmp == 6 && timer_cb_registered);
    CHECK(g_motors[0].generator_cw->gpio == 26 && g_motors[0].generator_ccw->gpio == 27);
    CHECK(g_motors[1].generator_cw->gpio == 18 && g_motors[1].generator_ccw->gpio == 19);
    CHECK(g_motors[2].generator_cw->gpio == 16 && g_motors[2].generator_ccw->gpio == 2);
    CHECK(g_motors[0].nfault_pin == 5 && g_motors[1].nfault_pin == 14 && g_motors[2].nfault_pin == 15);
    for (int i = 0; i < 6; ++i) CHECK(gens[i].force == 0);

    /* Parsing of motor tokens. */
    CHECK(parse_motor_token("M1") == 0 && parse_motor_token("m3") == 2 && parse_motor_token("2") == 1);
    CHECK(parse_motor_token("FEMUR") == 1 && parse_motor_token("ALL") == 3);
    CHECK(parse_motor_token("M4") == -1 && parse_motor_token("SET") == -1 && parse_motor_token("M") == -1);

    /* Unprefixed commands go to the selected motor (Femur by default). */
    CHECK(strcmp(cmd("SET,25"), "ACK,SET,25.000\n") == 0);
    CHECK(g_motors[1].duty_cw == 500 && g_motors[0].duty_cw == 0 && g_motors[2].duty_cw == 0);
    CHECK(g_motors[1].comparator_cw->value == 500 && g_motors[1].generator_cw->force == -1);
    CHECK(strcmp(cmd("M1,SET,-50"), "M1,ACK,SET,-50.000\n") == 0);
    CHECK(g_motors[0].duty_ccw == 1000 && g_motors[0].direction == MOTOR_CCW);
    CHECK(strstr(cmd("TIBIA,SET,100"), "M3,ACK,SET,100.000") != NULL);
    CHECK(g_motors[2].generator_cw->force == 1); /* 100 % forces high */
    CHECK(strcmp(cmd("M2,STOP"), "M2,ACK,STOP,0.000,0\n") == 0);
    CHECK(g_motors[1].direction == MOTOR_STOP && g_motors[0].direction == MOTOR_CCW);
    CHECK(strcmp(cmd("STOP"), "ACK,STOP,0.000,0\n") == 0);
    for (int i = 0; i < 3; ++i) CHECK(g_motors[i].direction == MOTOR_STOP && g_motors[i].duty_cw + g_motors[i].duty_ccw == 0);

    /* ALL fans out with per-motor prefixes. */
    CHECK(strcmp(cmd("ALL,ENCODER,ZERO"), "M1,ACK,ENCODER,ZERO\nM2,ACK,ENCODER,ZERO\nM3,ACK,ENCODER,ZERO\n") == 0);
    CHECK(zero_calls[0] == 1 && zero_calls[1] == 1 && zero_calls[2] == 1);
    CHECK(strstr(cmd("ALL,PID,5,1000,0"), "M3,ACK,PID,5.0000,1000.0000,0.0000") != NULL);
    CHECK(g_motors[0].pid.kp == 5 && g_motors[2].pid.ki == 1000);

    /* SEL and errors. */
    const char *r = cmd("SEL,3");
    CHECK(strncmp(r, "ACK,SEL,M3,TIBIA\nHEADER,", 24) == 0 && g_selected == 2);
    CHECK(strcmp(cmd("SEL,ALL"), "ERR,SEL,expected_M1_M2_M3\n") == 0);
    CHECK(strcmp(cmd("M4,SET,1"), "ERR,UNKNOWN,M4\n") == 0);
    CHECK(strcmp(cmd("M2,FOO"), "M2,ERR,UNKNOWN,FOO\n") == 0);
    CHECK(strcmp(cmd("M2,SET,200"), "M2,ERR,SET,expected_percent_-100_to_100\n") == 0);
    CHECK(strstr(cmd("TEL,MULTI"), "ACK,TEL,MULTI\nMHEADER,time_ms,selected,m1_mode") != NULL);
    CHECK(g_telemetry_mode == TELEMETRY_MULTI);
    r = cmd("INFO");
    CHECK(strstr(r, "INFO,M1,COXA,IN1=26,IN2=27,CS=32,ADC1_CH4,ENC_A=35,ENC_B=25,NFAULT=5") != NULL);
    CHECK(strstr(r, "INFO,M2,FEMUR,IN1=18,IN2=19,CS=33,ADC1_CH5,ENC_A=23,ENC_B=17,NFAULT=14") != NULL);
    CHECK(strstr(r, "INFO,M3,TIBIA,IN1=16,IN2=2,CS=34,ADC1_CH6,ENC_A=4,ENC_B=13,NFAULT=15") != NULL);
    CHECK(strstr(r, "INFO,BOARD,pata_MOD_DRV") != NULL);
    CHECK(fabsf(SENSE_CS_VOLTS_PER_AMP - 3.0f) < 1e-6f);
    r = cmd("DIAG");
    CHECK(strncmp(r, "DIAG,ADC,frames=", 16) == 0 && strstr(r, "DIAG,M3,TIBIA,ADC1_CH6,count=") != NULL);

    /* ESIGN only while stopped; it invalidates PREF. */
    cmd("M1,SET,20");
    CHECK(strcmp(cmd("M1,ESIGN,-1"), "M1,ERR,ESIGN,STOP_before_sign_change\n") == 0);
    cmd("M1,STOP");
    CHECK(strcmp(cmd("M1,ESIGN,-1"), "M1,ACK,ESIGN,-1\n") == 0 && sign_[0] == -1);
    CHECK(strcmp(cmd("M1,ESIGN,2"), "M1,ERR,ESIGN,expected_1_or_-1\n") == 0);

    /* Independent cascades: M3 POSITION to +1 rad while M2 runs SPEED. */
    step(5);
    CHECK(strcmp(cmd("M3,PREF,1.0"), "M3,ACK,PREF,1.000000\n") == 0);
    CHECK(strcmp(cmd("M3,ILIMIT,0.3"), "M3,ACK,ILIMIT,0.3000\n") == 0);
    CHECK(strcmp(cmd("M3,MODE,POSITION"), "M3,ACK,MODE,POSITION,CW\n") == 0);
    CHECK(strcmp(cmd("M2,ILIMIT,0.3"), "M2,ACK,ILIMIT,0.3000\n") == 0);
    CHECK(strcmp(cmd("M2,WREF,2"), "M2,ACK,WREF,2.0000\n") == 0);
    CHECK(strcmp(cmd("M2,MODE,SPEED,CCW"), "M2,ACK,MODE,SPEED,CCW\n") == 0);
    step(1500);
    CHECK(g_motors[2].control_mode == CONTROL_MODE_POSITION && g_motors[1].control_mode == CONTROL_MODE_SPEED);
    CHECK(g_motors[0].control_mode == CONTROL_MODE_OPEN && g_motors[0].duty_cw + g_motors[0].duty_ccw == 0);
    CHECK(pos[2] - ref_pos[2] > 0.05);            /* M3 moved toward +1 rad */
    CHECK(spd[1] < -0.2);                          /* M2 turning CCW */
    CHECK(g_motors[1].duty_ccw > 0 && g_motors[1].duty_cw == 0);
    CHECK(!g_motors[2].pending_position_fault && !g_motors[1].pending_speed_fault);

    /* nFAULT of the Tibia (IO15): a 1-2 ms glitch is filtered, a sustained low stops only M3. */
    gpio_level[15] = 0; step(2); gpio_level[15] = 1; step(1);
    CHECK(g_motors[2].control_mode == CONTROL_MODE_POSITION && !g_motors[2].pending_drv_fault);
    gpio_level[15] = 0; step(DRV_NFAULT_FILTER_MS);
    CHECK(g_motors[2].control_mode == CONTROL_MODE_OPEN && g_motors[2].pending_drv_fault);
    CHECK(g_motors[1].control_mode == CONTROL_MODE_SPEED);
    gpio_level[15] = 1; g_motors[2].pending_drv_fault = false;
    step(5);
    /* Re-enter POSITION on M3 (still targeting +1 rad) once it is stopped. */
    while (fabs(spd[2]) > 0.05) step(10);
    step(100);
    CHECK(strncmp(cmd("M3,MODE,POSITION"), "M3,ACK,MODE,POSITION,", 21) == 0);

    /* Losing M2's encoder stops only M2. */
    spd_valid[1] = false;
    step(3);
    CHECK(g_motors[1].control_mode == CONTROL_MODE_OPEN && g_motors[1].direction == MOTOR_STOP);
    CHECK(g_motors[1].pending_speed_fault);
    CHECK(g_motors[2].control_mode == CONTROL_MODE_POSITION);
    spd_valid[1] = true;

    /* PLIMIT / PPI are accepted live in POSITION without resetting the loop. */
    { position_state_t st = g_motors[2].position.state; float out = g_motors[2].position.output_rad_s;
      CHECK(strcmp(cmd("M3,PLIMIT,3,30,0.02"), "M3,ACK,PLIMIT,3.000000,30.000000,0.020000\n") == 0);
      CHECK(g_motors[2].control_mode == CONTROL_MODE_POSITION && g_motors[2].position.max_speed_rad_s == 3);
      CHECK(g_motors[2].position.state == st && g_motors[2].position.output_rad_s == out);
      CHECK(strncmp(cmd("M3,PPI,6.166,3192"), "M3,ACK,PPI,", 11) == 0 && g_motors[2].control_mode == CONTROL_MODE_POSITION);
      cmd("M3,PLIMIT,2,5,0.02"); }
    /* Position in POSITION mode refuses mode change without STOP. */
    CHECK(strcmp(cmd("M3,MODE,SPEED"), "M3,ERR,MODE,STOP_before_leaving_or_restarting_POSITION\n") == 0);
    step(4000);
    CHECK(fabs(pos[2] - ref_pos[2] - 1.0) < 0.05); /* reached the target band */
    /* Holding: the brake replaces coasting (both inputs forced high). */
    CHECK(g_motors[2].position.state != POSITION_TRACK && g_motors[2].direction == MOTOR_BRAKE);
    CHECK(g_motors[2].generator_cw->force == 1 && g_motors[2].generator_ccw->force == 1);
    { telemetry_t t; telemetry_snapshot(2, &t); CHECK(t.direction == MOTOR_BRAKE && t.hold_brake); }
    /* COAST hold releases the brake on the next step. */
    CHECK(strcmp(cmd("M3,HOLD,COAST"), "M3,ACK,HOLD,COAST\n") == 0);
    step(2);
    CHECK(g_motors[2].direction == MOTOR_STOP && g_motors[2].generator_cw->force == 0);
    CHECK(strcmp(cmd("M3,HOLD,BRAKE"), "M3,ACK,HOLD,BRAKE\n") == 0);
    CHECK(strcmp(cmd("M3,HOLD,X"), "M3,ERR,HOLD,expected_BRAKE_or_COAST\n") == 0);
    /* A new target leaves the brake and drives again. */
    CHECK(strcmp(cmd("M3,PREF,0.3"), "M3,ACK,PREF,0.300000\n") == 0);
    step(1500);
    CHECK(fabs(pos[2] - ref_pos[2] - 0.3) < 0.05 && g_motors[2].direction == MOTOR_BRAKE);
    cmd("STOP");
    for (int i = 0; i < 3; ++i) CHECK(g_motors[i].control_mode == CONTROL_MODE_OPEN);
    CHECK(g_motors[2].direction == MOTOR_STOP && g_motors[2].generator_cw->force == 0); /* STOP coasts */

    /* Friction and gravity feed-forward. */
    CHECK(strcmp(cmd("M1,FRIC,8"), "M1,ACK,FRIC,8.00\n") == 0 && g_motors[0].friction_pwm == 8);
    CHECK(strcmp(cmd("M1,FRIC,80"), "M1,ERR,FRIC,expected_0_to_70_percent\n") == 0);
    CHECK(strcmp(cmd("M1,GRAV,0.2,0.5"), "M1,ACK,GRAV,0.2000,0.5000\n") == 0);
    g_motors[0].speed.current_limit_a = 0.5f;
    { motor_direction_t d; float g;
      g_motors[0].control_direction = MOTOR_CW;   /* lifting: 0.1 + 0.2 */
      CHECK(fabsf(cascade_current(&g_motors[0], 0.1f, -0.5, &d, &g) - 0.3f) < 1e-6f && d == MOTOR_CW && fabsf(g - 0.2f) < 1e-6f);
      g_motors[0].control_direction = MOTOR_CCW;  /* lowering: still lift 0.2 - 0.1 = 0.1 */
      CHECK(fabsf(cascade_current(&g_motors[0], 0.1f, -0.5, &d, &g) - 0.1f) < 1e-6f && d == MOTOR_CW);
      CHECK(fabsf(cascade_current(&g_motors[0], 0.3f, -0.5, &d, &g) - 0.1f) < 1e-6f && d == MOTOR_CCW); /* pull down */
      CHECK(fabsf(cascade_current(&g_motors[0], 0.1f, 1.0707963, &d, &g) - 0.1f) < 1e-5f && d == MOTOR_CCW); /* cos(90)=0 */
      CHECK(cascade_current(&g_motors[0], 0.2f, -0.5, &d, &g) == 0.0f); /* balanced: brake/coast */
      g_motors[0].control_direction = MOTOR_CW; }
    CHECK(strstr(cmd("M1,GRAV,2,0"), "ERR,GRAV") != NULL);
    cmd("M1,GRAV,0,0"); cmd("M1,FRIC,0");
    CHECK(ENCODER_SPEED_MIN_WINDOW_US == 20000U);

    /* Gravity on the Coxa plant (stand-in for a stretched femur): 3 rad/s bias,
     * equivalent to 30 % PWM = 0.09 A at cos()=1. */
    cmd("M1,STOP"); spd[0] = 0; step(300);
    cmd("M1,ESIGN,1"); cmd("M1,ENCODER,ZERO");
    cmd("M1,PID,8.832,80090,0"); /* firmware default (ALL,PID changed it above) */
    cmd("M1,ILIMIT,0.6"); cmd("M1,LIMIT,0,100"); cmd("M1,SPI,0.08,0.5"); cmd("M1,PPI,4,0");
    /* Without GRAV: brake only, the link slides back and the loop keeps restarting.
     * (Gravity switched on right after activation: POSITION needs a still axis.) */
    CHECK(strcmp(cmd("M1,PREF,0.5"), "M1,ACK,PREF,0.500000\n") == 0);
    CHECK(strncmp(cmd("M1,MODE,POSITION"), "M1,ACK,MODE,POSITION", 20) == 0);
    grav_dist[0] = 3.0;
    int track_entries = 0; position_state_t last_state = g_motors[0].position.state;
    double worst_after_arrival = 0; bool arrived = false;
    for (int k = 0; k < 4000; ++k) {
        step(1);
        position_state_t st = g_motors[0].position.state;
        if (st == POSITION_TRACK && last_state != POSITION_TRACK) track_entries++;
        last_state = st;
        double e = fabs(pos[0] - ref_pos[0] - 0.5);
        if (st == POSITION_HOLD) arrived = true;
        if (arrived && e > worst_after_arrival) worst_after_arrival = e;
    }
    int track_entries_brake = track_entries;
    fprintf(stderr, "  sin GRAV: re-entradas a TRACK=%d, llego a HOLD=%s\n", track_entries_brake, arrived ? "si" : "no");
    /* With GRAV: active hold with the gravity current, no restarts. */
    CHECK(strcmp(cmd("M1,GRAV,0.09,0"), "M1,ACK,GRAV,0.0900,0.0000\n") == 0);
    cmd("M1,PREF,0.0"); step(3000);
    CHECK(strcmp(cmd("M1,PREF,0.5"), "M1,ACK,PREF,0.500000\n") == 0);
    track_entries = 0; last_state = g_motors[0].position.state; worst_after_arrival = 0; arrived = false;
    int hold_ticks = 0;
    for (int k = 0; k < 4000; ++k) {
        step(1);
        position_state_t st = g_motors[0].position.state;
        if (st == POSITION_TRACK && last_state != POSITION_TRACK) track_entries++;
        last_state = st;
        if (st == POSITION_HOLD) arrived = true;
        if (g_motors[0].gravity_holding) hold_ticks++;
        if (getenv("TRACE") && k % 50 == 0) fprintf(stderr, "t=%d pos=%.3f spd=%.2f st=%d hold=%d dir=%d duty=%.1f iref=%.3f amps=%.3f out=%.2f\n",
            k, pos[0]-ref_pos[0], spd[0], st, g_motors[0].gravity_holding, g_motors[0].direction, g_motors[0].duty_percent,
            g_motors[0].pid.current_ref_a, amps[0], g_motors[0].position.output_rad_s);
        double e = fabs(pos[0] - ref_pos[0] - 0.5);
        if (arrived && e > worst_after_arrival) worst_after_arrival = e;
    }
    fprintf(stderr, "  con GRAV: re-entradas a TRACK=%d, peor error tras llegar=%.3f rad, ticks sosteniendo=%d\n",
            track_entries, worst_after_arrival, hold_ticks);
    CHECK(g_motors[0].control_mode == CONTROL_MODE_POSITION && arrived);
    CHECK(g_motors[0].gravity_holding && g_motors[0].direction == MOTOR_CW && g_motors[0].duty_cw > 0);
    CHECK(worst_after_arrival < 0.04 && track_entries <= 1 && track_entries < track_entries_brake);
    { telemetry_t t; telemetry_snapshot(0, &t); CHECK(t.gravity_holding); }
    /* Lowering under gravity: the overspeed brake keeps the fall near vmax. */
    CHECK(strcmp(cmd("M1,PREF,-0.5"), "M1,ACK,PREF,-0.500000\n") == 0);
    double max_down = 0;
    for (int k = 0; k < 3000; ++k) { step(1); if (-spd[0] > max_down) max_down = -spd[0]; }
    fprintf(stderr, "  bajando con gravedad: velocidad max=%.2f rad/s (vmax 2)\n", max_down);
    CHECK(max_down < 3.0 && fabs(pos[0] - ref_pos[0] + 0.5) < 0.05);
    cmd("STOP"); grav_dist[0] = 0;

    /* Telemetry line layout: T + time + selected + 11 per motor. */
    fflush(stdout);
    long start = lseek(STDOUT_FILENO, 0, SEEK_CUR);
    printf("T,%lu,%d", 1UL, 1);
    for (int i = 0; i < MOTOR_COUNT; ++i) { telemetry_t t; telemetry_snapshot(i, &t); multi_print_motor(&t); }
    printf("\n"); fflush(stdout);
    long end = lseek(STDOUT_FILENO, 0, SEEK_CUR); fseek(capture, start, SEEK_SET);
    size_t n = fread(out, 1, (size_t)(end - start), capture); out[n] = 0;
    int commas = 0; for (char *p = out; *p; ++p) commas += *p == ',';
    CHECK(commas == 2 + 11 * MOTOR_COUNT);

    fprintf(stderr, "PASS: routing M1/M2/M3/ALL/SEL, STOP all, ESIGN, pins, independent cascades, fault isolation, T layout (final M3 %.3f rad)\n",
            pos[2] - ref_pos[2]);
    return 0;
}
