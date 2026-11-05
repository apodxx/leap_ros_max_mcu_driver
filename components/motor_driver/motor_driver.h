#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

class At8236Motor {
 public:
  // direction_sign applies the mechanical/electrical installation polarity:
  // positive logical speed must always mean "wheel drives the robot forward".
  At8236Motor(gpio_num_t in1_pin, gpio_num_t in2_pin, int8_t direction_sign = 1);

  void Init();
  void SetSpeed(int16_t speed);
  void Brake();

 private:
  gpio_num_t in1_pin_;
  gpio_num_t in2_pin_;
  ledc_channel_t channel_in1_;
  ledc_channel_t channel_in2_;
  int8_t direction_sign_;
  bool initialized_;
  bool usable_;
};

#endif  // MOTOR_DRIVER_H_
