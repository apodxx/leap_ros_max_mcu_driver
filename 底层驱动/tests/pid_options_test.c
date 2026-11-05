#include "pid_controller.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

int main(void)
{
    PID_Controller_t pid;
    PID_Init(&pid, 0, 1, 0, 5, 1000, -1000);
    PID_SetOptions(&pid, 20, 1);
    near(PID_Compute(&pid, 100, 0, 0.1f), 0);
    near(PID_Compute(&pid, 100, 80, 0.1f), 2);
    near(PID_Compute(&pid, 100, 0, 0.1f), 2);
    near(PID_Compute(&pid, 100, 120, 0.1f), 0);
    near(PID_Compute(&pid, 100, 121, 0.1f), 0);
    near(PID_Compute(&pid, 100, 80, 1), 5);
    PID_SetGains(&pid, 0, 0, 2);
    PID_Reset(&pid);
    near(PID_Compute(&pid, 100, 50, 0.1f), 0);
    near(PID_Compute(&pid, 200, 50, 0.1f), 0);
    near(PID_Compute(&pid, 200, 51, 0.1f), -20);
    near(PID_Compute(&pid, 200, 49, 0.2f), 20);
    assert(pid.integral_separation == 20 && pid.derivative_on_measurement);
    PID_Reset(&pid);
    near(PID_Compute(&pid, 200, 90, 0.1f), 0);
    PID_Init(&pid, 0, 0, 2, 5, 1000, -1000);
    near(PID_Compute(&pid, 10, 0, 0.1f), 200);
    puts("PID option tests passed");
    return 0;
}
