#include "wheel_bench.h"
#include <assert.h>
#include <stdio.h>

static int parse(WheelBenchConfig *c,int kind,int dir,int wheel,int value,int step,int hold,int duration)
{
    char json[256];
    snprintf(json,sizeof(json),"{\"type\":\"cmd\",\"cmd\":\"wheel_bench\",\"kind\":%d,\"dir\":%d,\"wheel\":%d,\"value\":%d,\"step\":%d,\"hold_ms\":%d,\"duration_ms\":%d}",kind,dir,wheel,value,step,hold,duration);
    return WheelBench_Parse(json,c);
}
int main(void)
{
    WheelBenchConfig c;
    assert(parse(&c,1,1,-1,70,0,0,3000));
    assert(WheelBench_Level(&c,2999)==70 && WheelBench_Level(&c,3000)==0);
    assert(WheelBench_Sign(&c,0)==1 && WheelBench_Sign(&c,1)==1);
    assert(WheelBench_Sign(&c,2)==-1 && WheelBench_Sign(&c,3)==-1);
    assert(parse(&c,1,-1,-1,70,0,0,3000) && WheelBench_Sign(&c,0)==-1);
    assert(!parse(&c,1,0,-1,70,0,0,3000));
    assert(!parse(&c,1,1,0,70,0,0,3000));
    assert(!parse(&c,1,1,-1,151,0,0,3000));
    assert(!parse(&c,1,1,-1,70,0,0,10001));
    assert(!parse(&c,2,1,-1,400,0,1000,0));
    assert(!parse(&c,2,1,-1,400,30,1000,0));
    assert(!parse(&c,2,1,-1,400,10,1000,0));
    assert(!parse(&c,2,1,4,400,50,1000,0));
    assert(parse(&c,2,1,-1,400,50,1000,0));
    assert(c.duration_ms==16000);
    assert(WheelBench_Level(&c,0)==0 && WheelBench_Level(&c,999)==0);
    assert(WheelBench_Level(&c,1000)==50);
    assert(WheelBench_Level(&c,8000)==400 && WheelBench_Level(&c,8999)==400);
    assert(WheelBench_Level(&c,9000)==350 && WheelBench_Level(&c,15999)==50);
    assert(WheelBench_Level(&c,16000)==0);
    for(int w=0;w<4;++w) {
        assert(parse(&c,2,-1,w,400,50,1000,0));
        for(int i=0;i<4;++i) assert(WheelBench_Sign(&c,i)==(i==w?-1:0));
    }
    assert(!WheelBench_StopReason(&c,1000,0,0,0.021f));
    assert(!strcmp(WheelBench_StopReason(&c,2500,0,0,0.021f),"link_timeout"));
    assert(!strcmp(WheelBench_StopReason(&c,16000,0,15900,0.021f),"completed"));
    assert(!strcmp(WheelBench_StopReason(&c,100,0,0,0.101f),"control_timeout"));
    assert(!WheelBench_StopReason(&c,50,UINT32_MAX-100,UINT32_MAX-100,0.021f));
    assert(!WheelBench_Parse("{\"type\":\"cmd\",\"cmd\":\"wheel_bench\",\"kind\":1}",&c));
    puts("PASS wheel bench: parsing, limits, directions, staircase, deadlines and tick wrap");
    return 0;
}
