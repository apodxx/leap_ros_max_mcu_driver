#ifndef WHEEL_OUTPUT_CONTROL_H
#define WHEEL_OUTPUT_CONTROL_H

#include "pid_controller.h"
#include <math.h>
#include <stdint.h>

/* Shared by translation, rotation, manual, navigation and RPM tests.
 * Rates are per second, not per scheduler invocation. */
#define WHEEL_TARGET_SLEW_RPM_S 500.0f
#define WHEEL_PWM_SLEW_PER_S 10000.0f
#define WHEEL_PWM_DEADZONE 200.0f
#define WHEEL_PWM_MAX 1000.0f

static inline void WheelOutput_Reset(PID_Controller_t pid[4], float target[4],
                                    int16_t pwm[4])
{
    for (int i = 0; i < 4; ++i) {
        PID_Reset(&pid[i]);
        target[i] = 0.0f;
        pwm[i] = 0;
    }
}

static inline void WheelOutput_Update(PID_Controller_t pid[4],
    const float requested[4], const float measured[4], float target[4],
    int16_t pwm[4], float dt, uint8_t *startup_active)
{
    if (!(dt > 0.0f)) return;
    /* Do not let a delayed update bypass the ramps. */
    if (dt > 0.1f) dt = 0.02f;
    float largest = 0.0f;
    for (int i = 0; i < 4; ++i) {
        float delta = fabsf(requested[i] - target[i]);
        if (delta > largest) largest = delta;
    }
    const float step = WHEEL_TARGET_SLEW_RPM_S * dt;
    const float fraction = *startup_active && largest > step ? step / largest : 1.0f;
    /* One-shot ramp: later speed corrections are applied without RPM slew. */
    if (fraction == 1.0f) *startup_active = 0U;
    const float pwm_step = WHEEL_PWM_SLEW_PER_S * dt;
    for (int i = 0; i < 4; ++i) {
        target[i] += (requested[i] - target[i]) * fraction;
        const float old_integral = pid[i].integral;
        float out;
        if (target[i] == 0.0f && requested[i] == 0.0f) {
            PID_Reset(&pid[i]);
            out = 0.0f;
        } else {
            out = PID_Compute(&pid[i], target[i], measured[i], dt);
            /* Fixed drive floor at every nonzero target, including below
             * 5 RPM. Preserve reverse braking and downstream PWM slew. */
            const float floor = WHEEL_PWM_DEADZONE;
            if (target[i] > 0.0f && out > 0.0f && out < floor) out = floor;
            if (target[i] < 0.0f && out < 0.0f && out > -floor) out = -floor;
        }
        if (out > WHEEL_PWM_MAX) out = WHEEL_PWM_MAX;
        if (out < -WHEEL_PWM_MAX) out = -WHEEL_PWM_MAX;
        const float desired = out;
        if (out > pwm[i] + pwm_step) out = pwm[i] + pwm_step;
        if (out < pwm[i] - pwm_step) out = pwm[i] - pwm_step;
        /* Freeze accumulation when the actuator ramp cannot follow a demand
         * in the direction of the error; allow integration that unwinds it. */
        if ((target[i] != 0.0f || requested[i] != 0.0f) &&
            (desired - out) * (target[i] - measured[i]) > 0.0f)
            pid[i].integral = old_integral;
        pwm[i] = (int16_t)out;
    }
}

#endif
