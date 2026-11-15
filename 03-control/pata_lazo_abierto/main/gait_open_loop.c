#include "gait_open_loop.h"
#include <math.h>
#include <string.h>

void gait_default_config(gait_config_t *c)
{
    memset(c, 0, sizeof(*c));
    /* Step cycle (coxa, femur, tibia). The femur falls toward +angle (14-nov
     * data), so "up" is -angle for the femur. Check the tibia before using it. */
    static const float poses[4][GAIT_JOINTS] = {
        {  0.30f,  0.00f, 0.00f },  /* 1 apoyo adelante */
        { -0.30f,  0.00f, 0.00f },  /* 2 empujar: apoyo atras */
        { -0.30f, -0.40f, 0.20f },  /* 3 levantar */
        {  0.30f, -0.40f, 0.20f },  /* 4 avanzar en el aire (luego bajar = pose 1) */
    };
    memcpy(c->pose, poses, sizeof(poses));
    c->n_poses = 4;
    for (int j = 0; j < GAIT_JOINTS; ++j) { c->pwm_pct[j] = 60.0f; c->tol_rad[j] = 0.03f; }
    c->pwm_step_pct = 10.0f;
    c->stall_ms = 150;
    c->min_rad[0] = -0.6f; c->max_rad[0] = 1.6f;   /* coxa  */
    c->min_rad[1] = -1.1f; c->max_rad[1] = 1.1f;   /* femur */
    c->min_rad[2] = -1.1f; c->max_rad[2] = 0.8f;   /* tibia */
    c->timeout_ms = 5000;
    c->dwell_ms = 200;
    c->cycles = 3;
}

static void push_event(gait_t *g, gait_event_type_t type, int joint, uint32_t ms, const double pos[GAIT_JOINTS])
{
    if (g->event_head - g->event_tail >= GAIT_EVENTS) g->event_tail++;   /* drop the oldest */
    gait_event_t *e = &g->events[g->event_head % GAIT_EVENTS];
    e->type = type; e->cycle = g->cycle + 1; e->pose = g->pose + 1; e->joint = joint;
    e->ms = ms; e->redrives = g->redrives;
    for (int j = 0; j < GAIT_JOINTS; ++j) { e->pos[j] = pos ? (float)pos[j] : NAN; e->pwm_max[j] = g->pwm_max[j]; }
    g->event_head++;
}

bool gait_pop_event(gait_t *g, gait_event_t *ev)
{
    if (g->event_tail == g->event_head) return false;
    *ev = g->events[g->event_tail % GAIT_EVENTS];
    g->event_tail++;
    return true;
}

static void begin_pose(gait_t *g, int64_t now_us)
{
    g->pose_start_us = now_us;
    g->all_arrived_since_us = 0;
    g->redrives = 0;
    for (int j = 0; j < GAIT_JOINTS; ++j) { g->arrived[j] = false; g->drive[j] = 0; g->pwm_max[j] = 0.0f; }
}

/* Start (or restart) driving joint j: base PWM, progress measured from here. */
static void drive_start(gait_t *g, int j, int dir, double pos, int64_t now_us)
{
    g->drive[j] = dir;
    g->pwm_now[j] = g->cfg.pwm_pct[j];
    g->progress_pos[j] = pos; g->progress_us[j] = now_us;
}

bool gait_start(gait_t *g, int64_t now_us)
{
    const gait_config_t *c = &g->cfg;
    if (c->n_poses < 1 || c->n_poses > GAIT_MAX_POSES || c->cycles < 1 || c->timeout_ms < 100 ||
        !(c->pwm_step_pct >= 0.0f && c->pwm_step_pct <= 50.0f) || c->stall_ms < 20) return false;
    for (int j = 0; j < GAIT_JOINTS; ++j) {
        if (!(c->pwm_pct[j] > 0.0f && c->pwm_pct[j] <= 100.0f) || !(c->tol_rad[j] > 0.0f) ||
            !(c->min_rad[j] < c->max_rad[j])) return false;
        for (int k = 0; k < c->n_poses; ++k)
            if (!isfinite(c->pose[k][j]) || c->pose[k][j] < c->min_rad[j] || c->pose[k][j] > c->max_rad[j]) return false;
    }
    g->status = GAIT_RUNNING;
    g->pose = 0; g->cycle = 0;
    g->run_start_us = now_us;
    g->event_head = g->event_tail = 0;
    begin_pose(g, now_us);
    return true;
}

static void finish(gait_t *g, gait_status_t status)
{
    g->status = status;
    for (int j = 0; j < GAIT_JOINTS; ++j) g->drive[j] = 0;
}

void gait_stop(gait_t *g, int64_t now_us)
{
    if (g->status == GAIT_RUNNING)
        push_event(g, GAIT_EV_STOPPED, -1, (uint32_t)((now_us - g->run_start_us) / 1000), NULL);
    finish(g, GAIT_IDLE);
}

bool gait_tick(gait_t *g, const double pos[GAIT_JOINTS], int64_t now_us, int dir[GAIT_JOINTS], float pwm[GAIT_JOINTS])
{
    for (int j = 0; j < GAIT_JOINTS; ++j) { dir[j] = 0; pwm[j] = 0.0f; }
    if (g->status != GAIT_RUNNING) return false;
    const gait_config_t *c = &g->cfg;
    uint32_t pose_ms = (uint32_t)((now_us - g->pose_start_us) / 1000);

    for (int j = 0; j < GAIT_JOINTS; ++j) {
        if (!isfinite(pos[j]) || pos[j] < c->min_rad[j] || pos[j] > c->max_rad[j]) {
            push_event(g, GAIT_EV_RANGE, j, pose_ms, pos);
            finish(g, GAIT_ABORTED);
            return false;
        }
    }
    bool all = true;
    for (int j = 0; j < GAIT_JOINTS; ++j) {
        double err = c->pose[g->pose][j] - pos[j];
        if (!g->arrived[j]) {
            /* Arrived: inside the band, or already past the target in the drive direction. */
            if (fabs(err) <= c->tol_rad[j] || (g->drive[j] != 0 && g->drive[j] * err <= 0.0)) {
                g->arrived[j] = true; g->drive[j] = 0;
            } else if (g->drive[j] == 0) {
                drive_start(g, j, err > 0 ? 1 : -1, pos[j], now_us);
            }
        } else if (fabs(err) > 3.0 * c->tol_rad[j]) {   /* drifted (gravity, rebound) */
            g->arrived[j] = false; drive_start(g, j, err > 0 ? 1 : -1, pos[j], now_us); g->redrives++;
        }
        if (g->drive[j] != 0) {
            /* Not getting there: give it more PWM (the motor takes the current it needs). */
            if (g->drive[j] * (pos[j] - g->progress_pos[j]) >= GAIT_PROGRESS_RAD) {
                g->progress_pos[j] = pos[j]; g->progress_us[j] = now_us;
            } else if (now_us - g->progress_us[j] >= (int64_t)c->stall_ms * 1000) {
                g->pwm_now[j] = fminf(100.0f, g->pwm_now[j] + c->pwm_step_pct);
                g->progress_us[j] = now_us;
            }
            if (g->pwm_now[j] > g->pwm_max[j]) g->pwm_max[j] = g->pwm_now[j];
        }
        if (!g->arrived[j]) all = false;
        dir[j] = g->drive[j];
        pwm[j] = g->drive[j] ? g->pwm_now[j] : 0.0f;
    }

    if (all) {
        if (!g->all_arrived_since_us) g->all_arrived_since_us = now_us;
        if (now_us - g->all_arrived_since_us >= (int64_t)c->dwell_ms * 1000) {
            push_event(g, GAIT_EV_POSE, -1, (uint32_t)((g->all_arrived_since_us - g->pose_start_us) / 1000), pos);
            if (++g->pose >= c->n_poses) {
                g->pose = 0;
                if (++g->cycle >= c->cycles) {
                    g->pose = c->n_poses - 1; g->cycle = c->cycles - 1;
                    push_event(g, GAIT_EV_DONE, -1, (uint32_t)((now_us - g->run_start_us) / 1000), pos);
                    finish(g, GAIT_DONE);
                    for (int j = 0; j < GAIT_JOINTS; ++j) { dir[j] = 0; pwm[j] = 0.0f; }
                    return false;
                }
            }
            begin_pose(g, now_us);
        }
    } else {
        g->all_arrived_since_us = 0;
    }

    if (pose_ms >= c->timeout_ms) {
        int worst = 0; double we = -1.0;
        for (int j = 0; j < GAIT_JOINTS; ++j) {
            double e = g->arrived[j] ? 0.0 : fabs(c->pose[g->pose][j] - pos[j]);
            if (e > we) { we = e; worst = j; }
        }
        push_event(g, GAIT_EV_TIMEOUT, worst, pose_ms, pos);
        finish(g, GAIT_ABORTED);
        for (int j = 0; j < GAIT_JOINTS; ++j) { dir[j] = 0; pwm[j] = 0.0f; }
        return false;
    }
    return true;
}
