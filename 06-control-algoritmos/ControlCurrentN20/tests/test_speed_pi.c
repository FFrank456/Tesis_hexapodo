#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "../main/speed_pi.h"
#undef assert
#define assert(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while (0)

static speed_pi_t controller(void) {
    return (speed_pi_t){.kp=SPEED_KP_INITIAL,.ki=SPEED_KI_INITIAL,
        .reference_rad_s=2,.current_limit_a=.1f,.last_update_us=1000000};
}
int main(void) {
    speed_pi_t p=controller();
    assert(speed_pi_update(&p,1,1010000,false));
    assert(fabsf(p.output_a-.0285392f)<1e-7f);
    float integral=p.integral_a,output=p.output_a;
    assert(!speed_pi_update(&p,1,1010000,false));
    assert(!speed_pi_update(&p,1,1015000,false));
    assert(p.integral_a==integral && p.output_a==output);
    assert(speed_pi_update(&p,1,1022000,false)); /* Actual 12 ms interval. */
    assert(fabsf(p.integral_a-.00030624f)<1e-7f);
    assert(!speed_pi_update(&p,1,1072000,false)); /* 50 ms gap rejected. */
    p=controller(); p.reference_rad_s=20;
    for (int n=1;n<=500;n++) assert(speed_pi_update(&p,0,1000000+n*10000,false));
    assert(p.integral_a==0 && fabsf(p.output_a-.1f)<1e-7f && p.limited);
    assert(speed_pi_update(&p,19,6010000,false));
    assert(p.integral_a>0 && p.output_a<.1f && !p.limited);
    p=controller();
    assert(speed_pi_update(&p,3,1010000,false)); /* Overspeed: coast, never negative current. */
    assert(p.output_a==0 && p.integral_a==0 && p.limited);
    p=controller();
    for (int n=1;n<=500;n++) assert(speed_pi_update(&p,1,1000000+n*10000,true));
    assert(p.integral_a==0 && fabsf(p.output_a-.0284f)<1e-7f && p.limited);
    assert(speed_pi_update(&p,1,6010000,false)); /* Unwind/release inner saturation. */
    assert(p.integral_a>0 && !p.limited);
    p=controller();
    assert(speed_pi_update(&p,-1,1010000,false)); /* Projected motion in opposite direction. */
    assert(p.error_rad_s==3);
    assert(!speed_pi_update(&p,NAN,1020000,false));
    p.ki=INFINITY; assert(!speed_pi_update(&p,0,1020000,false));
    p=controller(); p.current_limit_a=0; assert(!speed_pi_update(&p,0,1010000,false));
    assert(!speed_reference_valid(-1) && !speed_reference_valid(NAN) && !speed_reference_valid(32.1f));
    p=controller(); assert(speed_pi_update(&p,0,1010000,false)); speed_pi_reset(&p);
    assert(p.integral_a==0 && p.output_a==0 && p.last_update_us==0 && !p.limited);
    assert(p.reference_rad_s==2 && fabsf(p.kp-SPEED_KP_INITIAL)<1e-7f);
    puts("PASS: speed PI gains, timing, saturation, cascade antiwindup, coast, reset and invalid inputs");
}
