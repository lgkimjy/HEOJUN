/*
 * motor.h
 *
 *      Author: junyoung / lgkimjy
 */
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>

class DcMotorForRaspberryPi
{
  public:
    DcMotorForRaspberryPi();
    DcMotorForRaspberryPi(int encoder_pulse_per_rotation, float wheel_radius, float K_P, float K_D);
    ~DcMotorForRaspberryPi();

    double pwm_value_motor;
    bool direction;
    bool check_position_control;
    bool onoff;

    float current_pulse;
    float previous_pulse;
    float encoder_pulse;
    int encoder_pulse_position;
    float DistancePerCount;

    int before_num;
    int num;

    //variables for Encoder Counts
    ros::Time current_time, last_time;
    float dt;
    float vel;
    float acc;

    //variables for PD control
    float K_P;
    float K_D;

    float ref_speed;
    float err_speed;
    float err_speed_sum;
    float err_speed_dot;
    float err_speed_k_1;

    float up, ui, ud, usum;
    int u_in;

  private:
    int encoder_pulse_per_rotation_;
    float wheel_radius_;
    float K_P_;
    float K_D_;
};