/*
 * motor.cpp
 *
 *      Author: junyoung / lgkimjy
 */
#include <mobile_robot/motor.h>

DcMotorForRaspberryPi::DcMotorForRaspberryPi()
{
}
DcMotorForRaspberryPi::DcMotorForRaspberryPi(int encoder_pulse_per_rotation, float wheel_radius, float P_gain, float D_gain):
    encoder_pulse_per_rotation_(encoder_pulse_per_rotation),
    wheel_radius_(wheel_radius),
    K_P_(P_gain),
    K_D_(D_gain)
{
  pwm_value_motor = 0;
  direction = 0;
  check_position_control= 0;
  onoff = 0;

  current_pulse = 0;
  previous_pulse = 0;
  
  encoder_pulse = 0;
  encoder_pulse_position = 0;

  before_num = 0;
  num = 0;

  dt = 0;
  vel = 0;
  acc = 0;

  DistancePerCount = (M_PI*2*wheel_radius_) / encoder_pulse_per_rotation_;

  K_P = K_P_;
  K_D = K_D_;

  ref_speed = 0;
  err_speed = 0;
  err_speed_sum = 0;
  err_speed_k_1 = 0;
  err_speed_dot = 0;

  up = 0;
  ud = 0;
  ui = 0;
  usum = 0;
  u_in = 0;
}
DcMotorForRaspberryPi::~DcMotorForRaspberryPi()
{
}
