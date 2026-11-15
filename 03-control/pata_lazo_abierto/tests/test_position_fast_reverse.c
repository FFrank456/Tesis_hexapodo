/* Firmware defaults: 20 ms reversal dwell and 0.3 rad/s stop threshold. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../main/position_pi.h"
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
int main(void){
    CHECK(POSITION_REVERSE_QUIET_US==20000LL && POSITION_STOP_RAD_S==0.3f);
    position_pi_t p={.kp=4,.ki=0,.reference_rad=1,.max_speed_rad_s=2,.accel_rad_s2=50,
        .tolerance_rad=.02f,.direction=1,.last_update_us=1000000};
    CHECK(position_pi_update(&p,1.2,1,1020000,false));            /* overshoot while moving */
    CHECK(p.state==POSITION_REVERSING && p.output_rad_s==0);
    CHECK(position_pi_update(&p,1.2,.25f,1040000,false));         /* 0.25 rad/s counts as stopped */
    CHECK(p.direction==1 && p.quiet_since_us==1040000);
    CHECK(position_pi_update(&p,1.2,0,1060000,false));            /* 20 ms later: reverse */
    CHECK(p.direction==-1 && p.state==POSITION_TRACK);
    CHECK(position_pi_update(&p,1.2,0,1080000,false));CHECK(p.output_rad_s<0);
    puts("PASS: position reversal after 20 ms dwell, 0.3 rad/s stop threshold");
    return 0;
}
