#ifndef PID_PARAM_PARSER_H
#define PID_PARAM_PARSER_H

#include <stdio.h>
#include <math.h>
#include <ctype.h>

/* Protocol emitted by the Qt Apply button. Do not accept partial packets. */
static const char *PID_ParseParams(const char *json, float *kp, float *ki, float *kd)
{
    float p, i, d;
    int end = 0;
    int count = sscanf(json,
        " { \"type\" : \"param_set\" , \"kp\" : %f , \"ki\" : %f , \"kd\" : %f } %n",
        &p, &i, &d, &end);
    if (count != 3 || end == 0 || json[end] != '\0') return "parse_error";
    if (!isfinite(p) || !isfinite(i) || !isfinite(d) ||
        p < 0.0f || p > 100.0f || i < 0.0f || i > 100.0f ||
        d < 0.0f || d > 100.0f) return "invalid";
    *kp = p;
    *ki = i;
    *kd = d;
    return NULL;
}

#endif
