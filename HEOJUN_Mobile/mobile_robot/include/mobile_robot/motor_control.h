/*
 * motor_control.cpp
 *
 *      Author: junyoung / lgkimjy
 */
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <wiringPi.h>
#include <map>
#include <string>

//ros_communication_message type
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

//custom header
#include <mobile_robot/motor_cmd.h>
#include <mobile_robot/motor.h>

using namespace std;

//pin information
#define motor1_BREAK 26 // Break
#define motor1_IN1   6  // Enable
#define motor1_DIR   19 // Dir
#define motor1_PWM   13 // pwm
#define motor1_H_A   17 // HA
#define motor1_H_B   27 // HB
#define motor1_H_C   22 // HC
#define motor1_FG    23 // FG

#define motor2_BREAK 8  // Break
#define motor2_IN1   25 // Enable
#define motor2_DIR   7  // Dir
#define motor2_PWM   12 // pwm
#define motor2_H_A   16 // HA
#define motor2_H_B   20 // HB
#define motor2_H_C   21 // HC
#define motor2_FG    24 // FG

//mobile and wheel information
#define EncoderCountsPerWheel 312  //per 1 cycle
#define wheelradius 0.076  // units : m
#define M_PI 3.14159265358979323846


//ROS
ros::Subscriber motor_joy_sub;
ros::Subscriber motor_first_command_sub;
ros::Subscriber motor_second_command_sub;

//initialize
DcMotorForRaspberryPi* motor1;
DcMotorForRaspberryPi* motor2;
map<int, int> m;
	
//Joystick 
double joy;

//functions
void initialize();
void go(float spd);
void ControlSpeed(int pwm_pin, int direction_pin, DcMotorForRaspberryPi* motor);
void speedCalc(DcMotorForRaspberryPi* motor);

//callback
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);
void motor_first_command_callback(const std_msgs::Float64::ConstPtr& msg);
void motor_second_command_callback(const std_msgs::Float64::ConstPtr& msg);


//ISR & timer
void motor1_encoder(void);
void motor2_encoder(void);
void controlFunction(const ros::TimerEvent&);