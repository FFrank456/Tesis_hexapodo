#include <math.h>
#include <stdio.h>
#include "current_ema.h"
#include "current_sense.h"
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d\n",__LINE__); return 1; } } while(0)
int main(void) {
    current_ema_t f={0};
    CHECK(fabsf(current_ema_push(&f,0.142f,SENSE_CONTINUOUS_ALPHA)-0.142f)<0.000001f);
    for (int i=0;i<200;i++) current_ema_push(&f,1.142f,SENSE_CONTINUOUS_ALPHA);
    CHECK(fabsf(f.value-0.242f)<0.0001f); // 1 ms step: same response as old 1 kHz alpha .1.
    f=(current_ema_t){0};
    double sum=0;
    for (int i=0;i<200000;i++) {
        float y=current_ema_push(&f,(i%20)<6 ? 0.2f : 0.0f,SENSE_CONTINUOUS_ALPHA);
        if(i>=100000) sum+=y;
    }
    CHECK(fabs(sum/100000-0.06)<0.0001); // All phases included: 30% pulse train average.
    f.initialized=false;
    CHECK(current_ema_push(&f,0.5f,SENSE_CONTINUOUS_ALPHA)==0.5f); // reset after lost data
    puts("PASS: per-sample EMA, time constant, ON/OFF average, reset");
    return 0;
}
