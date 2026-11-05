#pragma once

#include <stdint.h>

namespace leap_config {
namespace motion {

static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kDegToRad = kPi / 180.0f;

// Target chassis parameters taken from the STM32H743 reference project under
// 底层驱动/Drivers/User/Inc/mecanum_kinematics.h.  That project drove the
// motor/gearbox pair this ESP32-S3 board is now connected to:
//   ROBOT_WHEEL_BASE   = 0.30 m
//   ROBOT_TRACK_WIDTH  = 0.35 m
//   ROBOT_WHEEL_RADIUS = 0.04 m  (80 mm mecanum wheels)
// All internal distances are millimetres.
static constexpr float kDifferentialWheelDiameterMm = 80.0f;
static constexpr float kMecanumWheelDiameterMm = 80.0f;
static constexpr float kMecanumWheelbaseMm = 300.0f;
static constexpr float kMecanumTrackWidthMm = 350.0f;
// The four-wheel chassis has no separate narrow differential geometry: use
// the physical left/right wheel spacing when differential mode is selected.
static constexpr float kDifferentialEffectiveTrackWidthMm = 350.0f;
static constexpr float kMecanumHalfWheelbaseMm = kMecanumWheelbaseMm / 2.0f;
static constexpr float kMecanumHalfTrackWidthMm = kMecanumTrackWidthMm / 2.0f;
static constexpr float kDifferentialHalfTrackMm = kDifferentialEffectiveTrackWidthMm / 2.0f;
static constexpr float kDifferentialRpmToMmS =
    (kPi * kDifferentialWheelDiameterMm) / 60.0f;
static constexpr float kMecanumRpmToMmS =
    (kPi * kMecanumWheelDiameterMm) / 60.0f;

// STM32 reference ENCODER_PPR = 68000 counts per *wheel* revolution.  That
// figure already includes the 4x quadrature decoding and the gearbox, which
// is exactly what the ESP32 PCNT unit also counts.  Do not multiply it by 4
// or by a gear ratio again.
static constexpr float kEncoderCountsPerWheelRev = 68000.0f;
static constexpr float kPulsesPerWheelRev = kEncoderCountsPerWheelRev;

// STM32 reference wheel speed loop, in its 0..1000 PWM scale:
//   PID_Init(&wheel_pid[i], 7.5f, 0.0f, 0.0f, 500.0f, 1000.0f, -1000.0f);
//   WHEEL_PWM_DEADZONE = 200, WHEEL_PWM_MAX = 1000
//   WHEEL_TARGET_SLEW_RPM_S = 500
// The ESP32 LEDC timer is therefore configured for 10-bit (0..1000 used) so
// these numbers can be carried over unchanged instead of being rescaled.
static constexpr float kDefaultSpeedKp = 7.5f;
static constexpr float kDefaultSpeedKi = 0.0f;
static constexpr float kDefaultSpeedKd = 0.0f;
static constexpr float kMaxPwm = 1000.0f;
static constexpr float kMotorPwmDeadband = 200.0f;
// The reference computes RPM from one encoder sample per 20 ms control period
// without an EMA.  At 68000 counts/rev the quantisation is ~0.04 RPM, so the
// extra filter lag is unnecessary.
static constexpr float kVelocityFilterAlpha = 1.0f;
static constexpr float kMaxAccelRpmPerS = 500.0f;

// Direction calibration, re-ordered to this firmware's wheel order
// [front-left, front-right, back-left, back-right].
//
// STM32 reference (tb6612_motor.c / encoder.c):
//   motor A/B/C/D = FL/BL/RR/FR, signs = {-1, +1, +1, -1}
//   encoder 1/2/3/4 = FL/BL/RR/FR, signs = {-1, -1, +1, +1}
//
// If a wheel turns the wrong way for a positive command, flip only that
// wheel's kMotorSigns entry.  If its measured velocity/odometry sign is
// wrong, flip only its kEncoderSigns entry.  If *all four* measured speeds
// have the opposite sign, the PCNT raw counting direction is globally
// opposite to the STM32 TIM encoder direction: flip all four encoder signs
// instead of chasing individual wheels.  See
// doc/四驱麦轮底盘参数适配.md for the full calibration procedure.
static constexpr int kMotorSigns[4] = {-1, -1, 1, 1};
static constexpr int kEncoderSigns[4] = {-1, 1, -1, 1};

static constexpr float kControlLoopDtS = 0.02f;
static constexpr uint32_t kControlLoopPeriodMs = 20;

}  // namespace motion
}  // namespace leap_config
