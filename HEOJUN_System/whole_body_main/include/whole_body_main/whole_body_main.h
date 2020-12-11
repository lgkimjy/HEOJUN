/*
 * motor_control.cpp
 *
 *      Author: junyoung / lgkimjy
 */
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>

//ros_communication_message type
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>

//mobile informations
#define mobile_mass 18.9

std_msgs::Float64 mobile_cmd_msg;

//ros communication pubsub declare
ros::Publisher mobile_cmd_vel_pub;
ros::Subscriber control_input_sub;
ros::Subscriber extern_force_sub;

//global variables
float control_input_vx;
float control_input_vy;
float control_input_wz;

float acc_x;
float acc_y;
float force_x;
float force_y;

ros::Time current_time, last_time;

float dt;
