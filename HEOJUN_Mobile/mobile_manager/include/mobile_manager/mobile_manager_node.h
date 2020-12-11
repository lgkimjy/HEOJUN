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

//custom header
#include <mobile_manager/motor_cmd.h>



// mobile robot information
#define L 0.09      // m
#define W 0.1715    // m


//ros communication
ros::Publisher  motor1_pub;
ros::Publisher  motor2_pub;
ros::Publisher  motor3_pub;
ros::Publisher  motor4_pub;

ros::Publisher script_number_pub;

//ros msg
std_msgs::Float64 motor_cmd_msg_1;
std_msgs::Float64 motor_cmd_msg_2;
std_msgs::Float64 motor_cmd_msg_3;
std_msgs::Float64 motor_cmd_msg_4;

//variables
float mobile_vx, mobile_vy, mobile_wz;
float m1_v, m2_v, m3_v, m4_v;
float mobile_acc;

bool rotation_left, rotation_right;
double max_speed;

//function
void initialize();
//callback
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);
void cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg);
void mecanum_kinematics(void);
void force2vel(void);