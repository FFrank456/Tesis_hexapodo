/* Open-loop step cycle for the whole leg (coxa, femur, tibia).
 *
 * Each joint gets a fixed PWM toward the target angle of the current pose and
 * is braked once it reaches it. There is no current, speed or position loop:
 * the motor takes whatever current it needs (only the DRV8876 limits it).
 * A pose is done when the three joints have arrived and stayed for dwell_ms;
 * then the next pose starts. Pure logic, no ESP-IDF: tested on the PC. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define GAIT_JOINTS     3
#define GAIT_MAX_POSES  8
#define GAIT_EVENTS     8
#define GAIT_PROGRESS_RAD 0.01   /* movement toward the target that counts as progress */

typedef struct {
    float pose[GAIT_MAX_POSES][GAIT_JOINTS]; /* target angles, rad */
    int n_poses;
    float pwm_pct[GAIT_JOINTS];  /* starting PWM toward the target, % */
    float pwm_step_pct;          /* no progress for stall_ms -> PWM += step (up to 100 %) */
    uint32_t stall_ms;
    float tol_rad[GAIT_JOINTS];  /* arrival band; re-driven beyond 3*tol */
    float min_rad[GAIT_JOINTS], max_rad[GAIT_JOINTS]; /* safety window: outside -> abort */
    uint32_t timeout_ms;         /* per pose */
    uint32_t dwell_ms;           /* all arrived for this long -> next pose */
    int cycles;                  /* times through the pose list */
} gait_config_t;

typedef enum { GAIT_IDLE = 0, GAIT_RUNNING, GAIT_DONE, GAIT_ABORTED } gait_status_t;
typedef enum { GAIT_EV_POSE = 1, GAIT_EV_DONE, GAIT_EV_TIMEOUT, GAIT_EV_RANGE, GAIT_EV_STOPPED } gait_event_type_t;

typedef struct {
    gait_event_type_t type;
    int cycle, pose, joint;      /* joint: -1 when not about one joint */
    uint32_t ms;                 /* pose time (POSE) or elapsed time */
    int redrives;                /* re-drives after arriving, this pose */
    float pos[GAIT_JOINTS];
    float pwm_max[GAIT_JOINTS];  /* highest PWM each joint needed in this pose */
} gait_event_t;

typedef struct {
    gait_config_t cfg;
    gait_status_t status;
    int pose, cycle;
    int64_t pose_start_us, all_arrived_since_us, run_start_us;
    bool arrived[GAIT_JOINTS];
    int drive[GAIT_JOINTS];      /* +1 toward +angle, -1 toward -angle, 0 braked */
    float pwm_now[GAIT_JOINTS], pwm_max[GAIT_JOINTS];
    double progress_pos[GAIT_JOINTS];  /* where the joint last made progress */
    int64_t progress_us[GAIT_JOINTS];
    int redrives;
    gait_event_t events[GAIT_EVENTS];
    uint32_t event_head, event_tail; /* written by the control task, read by the UART loop */
} gait_t;

void gait_default_config(gait_config_t *c);
/* Checks ranges; returns false (and leaves the gait idle) on a bad config. */
bool gait_start(gait_t *g, int64_t now_us);
void gait_stop(gait_t *g, int64_t now_us);  /* external STOP: idle, all braked */
/* One control step. pos: joint angles (rad, non-finite = unknown -> abort).
 * Out: dir per joint (+1/-1 drive, 0 brake) and PWM %. Returns true while
 * the gait owns the outputs (RUNNING); false when idle/done/aborted. */
bool gait_tick(gait_t *g, const double pos[GAIT_JOINTS], int64_t now_us, int dir[GAIT_JOINTS], float pwm[GAIT_JOINTS]);
bool gait_pop_event(gait_t *g, gait_event_t *ev);
