#ifndef WHEEL_BENCH_H
#define WHEEL_BENCH_H
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Compact, ordered wire format used by the Qt controls. Pure helpers also
 * run in the host tests. kind: 1=closed-loop RPM, 2=open-loop PWM staircase. */
typedef struct {
    int kind, dir, wheel, value, step, hold_ms, duration_ms;
} WheelBenchConfig;

static inline int WheelBench_Parse(const char *json, WheelBenchConfig *out)
{
    WheelBenchConfig c;
    int end = 0;
    if (sscanf(json, "{\"type\":\"cmd\",\"cmd\":\"wheel_bench\",\"kind\":%2d,\"dir\":%2d,\"wheel\":%2d,\"value\":%4d,\"step\":%4d,\"hold_ms\":%5d,\"duration_ms\":%5d}%n",
        &c.kind,&c.dir,&c.wheel,&c.value,&c.step,&c.hold_ms,&c.duration_ms,&end) != 7 || !end)
        return 0;
    while (json[end]==' ' || json[end]=='\r' || json[end]=='\n' || json[end]=='\t') ++end;
    if (json[end] || (c.dir!=1 && c.dir!=-1) || c.wheel < -1 || c.wheel > 3) return 0;
    if (c.kind==1) {
        if (c.wheel!=-1 || c.value<20 || c.value>150 || c.duration_ms<500 || c.duration_ms>10000) return 0;
        if(c.step!=0 || c.hold_ms!=0) return 0;
    } else if(c.kind==2) {
        if(c.value<50 || c.value>600 || c.step<10 || c.step>100 || c.value%c.step) return 0;
        if(c.hold_ms<500 || c.hold_ms>2000 || c.duration_ms!=0) return 0;
        c.duration_ms=2*(c.value/c.step)*c.hold_ms;
        if(c.duration_ms>30000) return 0;
    } else return 0;
    *out=c;
    return 1;
}

static inline int WheelBench_Level(const WheelBenchConfig *c, uint32_t elapsed)
{
    if(elapsed >= (uint32_t)c->duration_ms) return 0;
    if(c->kind==1) return c->value;
    /* One zero-output dwell, rise to max once, descend to one step, stop. */
    int stage=(int)(elapsed/(uint32_t)c->hold_ms);
    int peak=c->value/c->step;
    return (stage<=peak ? stage : 2*peak-stage)*c->step;
}

static inline int WheelBench_Sign(const WheelBenchConfig *c, int wheel)
{
    if(c->wheel>=0) return c->wheel==wheel ? c->dir : 0;
    return (wheel<2 ? 1 : -1)*c->dir;
}

static inline const char *WheelBench_StopReason(const WheelBenchConfig *c,
    uint32_t now, uint32_t start, uint32_t heartbeat, float sample_seconds)
{
    if(now-start >= (uint32_t)c->duration_ms) return "completed";
    if(now-heartbeat >= 2500U) return "link_timeout";
    if(sample_seconds>0.1f) return "control_timeout";
    return NULL;
}
#endif
