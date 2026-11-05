#include "wheel_output_control.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

static void near(float a, float b) { assert(fabsf(a - b) < 0.002f); }

int main(void)
{
    PID_Controller_t pid[4];
    float target[4] = {0}, measured[4] = {0};
    int16_t pwm[4] = {0};
    uint8_t startup = 1U;
    const float sideways[4] = {-200, 200, -200, 200};
    for (int i = 0; i < 4; ++i)
        PID_Init(&pid[i], 7.5f, 0, 0, 500, 1000, -1000);
    for (int n = 0; n < 20; ++n) {
        int16_t before = pwm[1];
        WheelOutput_Update(pid, sideways, measured, target, pwm, 0.02f, &startup);
        near(target[1], (n + 1) * 10.0f);
        near(target[0], -target[1]);
        assert(abs(pwm[1] - before) <= 200);
        assert(pwm[0] == -pwm[1] && pwm[2] == pwm[0] && pwm[3] == pwm[1]);
    }
    assert(startup == 0U);
    const float adjusted[4] = {-300, 300, -300, 300};
    WheelOutput_Update(pid, adjusted, measured, target, pwm, 0.02f, &startup);
    near(target[1], 300); /* No RPM ramp after startup. */
    WheelOutput_Reset(pid, target, pwm);
    startup = 0U;
    const float rotation[4] = {80, 80, -80, -80};
    WheelOutput_Update(pid, rotation, measured, target, pwm, 0.02f, &startup);
    near(target[0], 80); near(target[2], -80);
    assert(pwm[0] == 200); /* Rotation still has PWM slew protection. */
    /* Mixed translation/rotation preserves the requested ratio at startup. */
    WheelOutput_Reset(pid, target, pwm); startup = 1U;
    const float mixed[4] = {200, 100, -50, -200};
    WheelOutput_Update(pid, mixed, measured, target, pwm, 0.02f, &startup);
    near(target[0], 10); near(target[1], 5); near(target[2], -2.5f);
    /* Same ramp duration with a different scheduler period. */
    WheelOutput_Reset(pid, target, pwm); startup = 1U;
    for (int n = 0; n < 40; ++n)
        WheelOutput_Update(pid, sideways, measured, target, pwm, 0.01f, &startup);
    near(target[1], 200);
    /* Load/disturbance and reversal cannot bypass PWM slew limits. */
    const float reversed[4] = {200, -200, 200, -200};
    for (int n = 0; n < 50; ++n) {
        int16_t before[4];
        for (int i = 0; i < 4; ++i) { before[i] = pwm[i]; measured[i] = n % 2 ? 300 : -300; }
        WheelOutput_Update(pid, reversed, measured, target, pwm, 0.02f, &startup);
        for (int i = 0; i < 4; ++i) {
            assert(abs(pwm[i] - before[i]) <= 200);
            assert(abs(pwm[i]) <= 1000);
        }
    }
    /* Directional deadzone boost must not amplify reverse braking. */
    WheelOutput_Reset(pid, target, pwm); startup = 1U;
    const float slow[4] = {5, 5, 5, 5};
    for (int i = 0; i < 4; ++i) measured[i] = 0;
    WheelOutput_Update(pid, slow, measured, target, pwm, 0.02f, &startup);
    assert(pwm[0] == 200);
    for (int i = 0; i < 4; ++i) measured[i] = 6;
    WheelOutput_Update(pid, slow, measured, target, pwm, 0.02f, &startup);
    assert(pwm[0] == 0); /* Slew from +200 before reverse braking. */
    WheelOutput_Update(pid, slow, measured, target, pwm, 0.02f, &startup);
    assert(pwm[0] == -7);
    /* Below 5 RPM, both drive directions retain the full fixed floor. */
    WheelOutput_Reset(pid, target, pwm); startup = 1U;
    const float tiny[4] = {1, -1, 0.1f, -0.1f};
    for (int i = 0; i < 4; ++i) measured[i] = 0;
    WheelOutput_Update(pid, tiny, measured, target, pwm, 0.02f, &startup);
    assert(pwm[0] == 200 && pwm[1] == -200);
    assert(pwm[2] == 200 && pwm[3] == -200);
    /* Stop immediately clears all state; idle remains zero despite encoder motion. */
    WheelOutput_Reset(pid, target, pwm); startup = 1U;
    const float zero[4] = {0};
    WheelOutput_Update(pid, zero, measured, target, pwm, 0.02f, &startup);
    for (int i = 0; i < 4; ++i) { near(target[i], 0); assert(pwm[i] == 0); }
    /* Integral cannot wind up against the downstream PWM slew limit. */
    for (int i = 0; i < 4; ++i) {
        PID_Init(&pid[i], 100, 1, 0, 500, 1000, -1000);
        measured[i] = 0;
    }
    WheelOutput_Update(pid, sideways, measured, target, pwm, 0.02f, &startup);
    near(pid[1].integral, 0);
    assert(pwm[1] == 200);
    /* A normal zero command may still have PWM ramping down. Do not restore
     * the old integral after resetting the stopped wheel's controller. */
    for (int i = 0; i < 4; ++i) {
        target[i] = 0; pwm[i] = 500; measured[i] = 10;
        pid[i].integral = 20;
    }
    WheelOutput_Update(pid, zero, measured, target, pwm, 0.02f, &startup);
    near(pid[0].integral, 0); assert(pwm[0] == 300);
    puts("wheel_output_control tests passed");
    return 0;
}
