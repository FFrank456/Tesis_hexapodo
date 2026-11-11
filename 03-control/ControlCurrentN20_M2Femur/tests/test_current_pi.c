#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "../main/current_pi.h"

static pid_controller_t controller(void) {
    pid_controller_t p = {.kp=PID_KP_INITIAL,.ki=PID_KI_INITIAL,
        .kd=PID_KD_INITIAL,.output_max_percent=100,.current_ref_a=0.1f};
    p.last_sample_us=1000000;
    return p;
}
int main(void) {
    pid_controller_t p=controller();
    assert(pi_update_sample(&p,0,1001000));
    assert(fabsf(p.output_percent-8.8922f)<0.001f);
    float integral=p.integrator, out=p.output_percent;
    assert(!pi_update_sample(&p,0,1001000)); /* Same block: no double integration. */
    assert(p.integrator==integral && p.output_percent==out);
    assert(pi_update_sample(&p,0,1003000)); /* 2 ms, not the old 1 ms fallback. */
    assert(fabsf(p.integrator-24.027f)<0.001f);
    assert(!pi_update_sample(&p,0,1002000));
    assert(!pi_update_sample(&p,0,1023000)); /* >=20 ms gap. */
    p=controller(); p.current_ref_a=1;
    for (int k=1;k<=500;k++) assert(pi_update_sample(&p,0,1000000+k*1000));
    assert(fabsf(p.integrator-80.09f)<0.001f && p.output_percent==100); /* No windup on high rail. */
    assert(pi_update_sample(&p,1.1f,1501000));
    assert(p.output_percent<100 && p.integrator<80.09f); /* Recover from saturation. */
    p=controller();
    assert(pi_update_sample(&p,1.1f,1001000));
    assert(p.output_percent==0 && p.integrator==0); /* Low rail. */
    p=controller(); p.output_max_percent=40; p.current_ref_a=1;
    assert(pi_update_sample(&p,0,1001000) && p.output_percent==40);
    p=controller(); p.integrator=30-p.kp*(p.current_ref_a-0.05f);
    assert(pi_update_sample(&p,0.05f,1001000));
    assert(fabsf(p.output_percent-34.0045f)<0.001f); /* Tracking on mode entry. */
    assert(!pi_update_sample(&p,NAN,1002000));
    p.kp=INFINITY; assert(!pi_update_sample(&p,0,1002000));
    p=controller(); p.kd=1; assert(!pi_update_sample(&p,0,1001000));
    float value=0;
    assert(control_parse_float("504.9",&value) && fabsf(value-504.9f)<0.001f);
    assert(!control_parse_float("nan",&value)); assert(!control_parse_float("inf",&value));
    assert(!control_parse_float("3garbage",&value)); assert(!control_parse_float("",&value));
    assert(!pi_reference_valid(-1) && !pi_reference_valid(NAN) && !pi_reference_valid(1.1f));
    pid_reset(&p); assert(p.integrator==0 && p.last_sample_us==0 && p.output_percent==0);
    puts("PASS: PI gains, timing, duplicate blocks, gaps, rails, tracking, reset and invalid inputs");
}
