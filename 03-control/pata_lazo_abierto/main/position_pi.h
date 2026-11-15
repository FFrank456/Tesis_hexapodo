#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define POSITION_PERIOD_US 20000LL
#define POSITION_MAX_GAP_US 100000LL
#define POSITION_QUIET_US 100000LL
/* Dwell before reversing after an overshoot. With the brake hold the axis
 * stops in a few ms, so 20 ms is enough (was 100 ms). */
#ifndef POSITION_REVERSE_QUIET_US
#define POSITION_REVERSE_QUIET_US 20000LL
#endif
/* "Stopped" threshold. With the 20 ms speed window one encoder count already
 * reads 0.11 rad/s (2800 cpr), so the threshold is 0.3 rad/s (was 0.1). */
#ifndef POSITION_STOP_RAD_S
#define POSITION_STOP_RAD_S 0.3f
#endif
#define POSITION_KP_INITIAL 6.166f
#define POSITION_KI_INITIAL 3192.0f
#define POSITION_MAX_RAD 10000.0

typedef enum { POSITION_TRACK=0, POSITION_REVERSING=1,
               POSITION_HOLD=2, POSITION_ARRIVING=3 } position_state_t;
typedef struct {
    double reference_rad, error_rad;
    float kp,ki,integral_rad_s,output_rad_s,max_speed_rad_s,accel_rad_s2,tolerance_rad;
    int direction;
    int64_t last_update_us,quiet_since_us;
    position_state_t state;
    bool limited;
} position_pi_t;

static inline bool position_reference_valid(double x) {
    return isfinite(x) && fabs(x)<=POSITION_MAX_RAD;
}
static inline void position_pi_reset(position_pi_t *p) {
    p->integral_rad_s=p->output_rad_s=0;p->error_rad=0;
    p->last_update_us=p->quiet_since_us=0;p->state=POSITION_TRACK;p->limited=false;
    if(p->direction!=1 && p->direction!=-1)p->direction=1;
}
static inline bool position_pi_update(position_pi_t *p,double measured_rad,
                                      float speed_rad_s,int64_t now_us,bool downstream_limited) {
    if(!position_reference_valid(p->reference_rad) || !isfinite(measured_rad) ||
       !isfinite(speed_rad_s) || !isfinite(p->kp) || !isfinite(p->ki) || p->kp<0 || p->ki<0 ||
       !isfinite(p->integral_rad_s) || !isfinite(p->output_rad_s) ||
       !isfinite(p->max_speed_rad_s) || p->max_speed_rad_s<=0 || p->max_speed_rad_s>32 ||
       !isfinite(p->accel_rad_s2) || p->accel_rad_s2<=0 || p->accel_rad_s2>100 ||
       !isfinite(p->tolerance_rad) || p->tolerance_rad<0.0023f || p->tolerance_rad>1 ||
       (p->direction!=1 && p->direction!=-1) || now_us<=p->last_update_us) return false;
    int64_t elapsed=now_us-p->last_update_us;
    if(p->last_update_us && (elapsed<POSITION_PERIOD_US || elapsed>=POSITION_MAX_GAP_US))return false;
    float dt=p->last_update_us ? elapsed*1e-6f : 0;
    double error=p->reference_rad-measured_rad;
    if(!isfinite(error))return false;
    p->error_rad=error;p->last_update_us=now_us;

    /* Coast on entering the target band. Reacquire only outside 2*tolerance.
     * HOLD is a monitored coast state, not active braking or holding torque. */
    double band=(p->state==POSITION_HOLD || p->state==POSITION_ARRIVING) ?
        2*p->tolerance_rad : p->tolerance_rad;
    if(fabs(error)<=band) {
        p->output_rad_s=p->integral_rad_s=0;p->limited=false;
        if(fabsf(speed_rad_s)<=POSITION_STOP_RAD_S) {
            if(!p->quiet_since_us)p->quiet_since_us=now_us;
            p->state=now_us-p->quiet_since_us>=POSITION_QUIET_US ? POSITION_HOLD : POSITION_ARRIVING;
        } else {p->quiet_since_us=0;p->state=POSITION_ARRIVING;}
        return true;
    }
    if(p->state==POSITION_HOLD || p->state==POSITION_ARRIVING) {
        p->state=POSITION_TRACK;p->quiet_since_us=0;
    }
    float candidate=p->integral_rad_s+p->ki*(float)error*dt;
    float requested=p->kp*(float)error+candidate;
    if(!isfinite(requested))return false;
    /* A crossed target must shed stored integral before a reversal. */
    if(requested*error<0) {p->integral_rad_s=0;candidate=p->ki*(float)error*dt;requested=p->kp*(float)error+candidate;}
    int wanted=requested>0 ? 1 : requested<0 ? -1 : p->direction;
    if(wanted!=p->direction || wanted*speed_rad_s < -POSITION_STOP_RAD_S || p->state==POSITION_REVERSING) {
        p->state=POSITION_REVERSING;p->output_rad_s=p->integral_rad_s=0;p->limited=true;
        if(fabsf(speed_rad_s)<=POSITION_STOP_RAD_S) {
            if(!p->quiet_since_us)p->quiet_since_us=now_us;
            if(now_us-p->quiet_since_us>=POSITION_REVERSE_QUIET_US) {
                p->direction=wanted;p->state=POSITION_TRACK;p->quiet_since_us=0;
            }
        } else p->quiet_since_us=0;
        return true;
    }
    p->quiet_since_us=0;
    bool blocked=downstream_limited && error*p->direction>0;
    if(blocked)requested=p->kp*(float)error+p->integral_rad_s;
    float bounded=fminf(p->max_speed_rad_s,fmaxf(-p->max_speed_rad_s,requested));
    float delta=p->accel_rad_s2*dt;
    float ramped=fminf(p->output_rad_s+delta,fmaxf(p->output_rad_s-delta,bounded));
    /* Limits may be tightened while running; never exceed the new speed ceiling. */
    ramped=fminf(p->max_speed_rad_s,fmaxf(-p->max_speed_rad_s,ramped));
    bool saturating=(requested-ramped)*error>0;
    if(!saturating && !blocked)p->integral_rad_s=candidate;
    p->output_rad_s=ramped;p->limited=saturating || blocked;p->state=POSITION_TRACK;
    return true;
}
