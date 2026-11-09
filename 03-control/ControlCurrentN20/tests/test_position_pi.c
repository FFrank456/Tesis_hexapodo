#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../main/position_pi.h"
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static position_pi_t initial(void){
    return (position_pi_t){.kp=POSITION_KP_INITIAL,.ki=POSITION_KI_INITIAL,
        .reference_rad=1,.max_speed_rad_s=2,.accel_rad_s2=5,.tolerance_rad=.02f,
        .direction=1,.last_update_us=1000000};
}
int main(void){
    position_pi_t p=initial();
    CHECK(position_pi_update(&p,0,0,1020000,false));
    CHECK(fabsf(p.output_rad_s-.1f)<1e-6f && p.integral_rad_s==0 && p.limited);
    CHECK(!position_pi_update(&p,0,0,1020000,false));
    CHECK(!position_pi_update(&p,0,0,1030000,false));
    CHECK(position_pi_update(&p,0,0,1045000,false));
    CHECK(fabsf(p.output_rad_s-.225f)<1e-6f); /* Actual 25 ms ramp. */
    CHECK(!position_pi_update(&p,0,0,1145000,false));
    p=initial();
    for(int n=1;n<=500;n++)CHECK(position_pi_update(&p,0,0,1000000+n*20000LL,false));
    CHECK(fabsf(p.output_rad_s-2)<1e-6f && p.integral_rad_s==0);
    CHECK(position_pi_update(&p,1.2,1,11020000,false));
    CHECK(p.output_rad_s==0 && p.direction==1 && p.state==POSITION_REVERSING);
    CHECK(position_pi_update(&p,1.2,.2f,11040000,false));
    CHECK(p.quiet_since_us==0);
    for(int n=0;n<5;n++){
        CHECK(position_pi_update(&p,1.2,0,11060000+n*20000LL,false));CHECK(p.direction==1 && p.output_rad_s==0);
    }
    CHECK(position_pi_update(&p,1.2,0,11160000,false));
    CHECK(p.direction==-1 && p.output_rad_s==0 && p.state==POSITION_TRACK);
    CHECK(position_pi_update(&p,1.2,0,11180000,false));CHECK(p.output_rad_s<0);
    /* Target band with slow coast, then dwell and hysteresis. */
    CHECK(position_pi_update(&p,1.01,.2f,11200000,false));CHECK(p.state==POSITION_ARRIVING && p.output_rad_s==0);
    for(int n=1;n<=6;n++)CHECK(position_pi_update(&p,1.01,0,11200000+n*20000LL,false));
    CHECK(p.state==POSITION_HOLD && p.integral_rad_s==0);
    CHECK(position_pi_update(&p,1.03,0,11340000,false));CHECK(p.state==POSITION_HOLD);
    CHECK(position_pi_update(&p,1.06,0,11360000,false));CHECK(p.state==POSITION_TRACK && p.output_rad_s<0);
    /* Unsaturated low-gain fixture verifies the actual PI, separately from the ramp. */
    p=initial();p.kp=1;p.ki=2;p.reference_rad=.1;p.accel_rad_s2=100;
    CHECK(position_pi_update(&p,0,0,1020000,false));CHECK(fabsf(p.output_rad_s-.104f)<1e-6f);
    float integrator=p.integral_rad_s;
    CHECK(position_pi_update(&p,0,0,1040000,true));CHECK(p.integral_rad_s==integrator);
    CHECK(fabsf(p.output_rad_s-.104f)<1e-6f);
    /* Opposite measured movement forces coast even if no sign change was requested. */
    p=initial();CHECK(position_pi_update(&p,0,-.5f,1020000,false));CHECK(p.state==POSITION_REVERSING && p.output_rad_s==0);
    p=initial();p.reference_rad=-1;p.direction=-1;
    CHECK(position_pi_update(&p,0,0,1020000,false));CHECK(p.output_rad_s<0);
    p=initial();CHECK(!position_pi_update(&p,NAN,0,1020000,false));
    CHECK(!position_pi_update(&p,0,INFINITY,1020000,false));
    p.tolerance_rad=0;CHECK(!position_pi_update(&p,0,0,1020000,false));
    CHECK(!position_reference_valid(INFINITY) && !position_reference_valid(10001));
    p=initial();CHECK(position_pi_update(&p,0,0,1020000,false));position_pi_reset(&p);
    CHECK(p.output_rad_s==0 && p.integral_rad_s==0 && p.reference_rad==1 && p.last_update_us==0);
    puts("PASS: position PI, ramp, rails, downstream antiwindup, coast reversal dwell, hysteresis, invalid data");
}
