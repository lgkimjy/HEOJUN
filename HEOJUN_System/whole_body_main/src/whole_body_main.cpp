/*
 * motor_control.cpp
 *
 *      Author: junyoung / lgkimjy
 */
#include <whole_body_main/whole_body_main.h>

void initialize()
{
    control_input_vx = 0;
    control_input_vy = 0;
    control_input_wz = 0;

    force_x = 0;

    dt = 0;
}

//callbacks///////////////////////////////////////////////
void inputForce_twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    control_input_vx = msg->linear.x;
    control_input_vy = msg->linear.y;
    control_input_wz = msg->angular.z;
    
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();

    acc_x = control_input_vx/dt;
    acc_y = control_input_vy/dt;

    force_x = mobile_mass * acc_x;
    force_y = mobile_mass * acc_y;

    last_time = current_time;

    return;
}
void externForce_Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    msg->linear.x;
    msg->linear.y;
	return;
}

void ComplianceSysFunction(const ros::TimerEvent&)
{
    // Mass = u - F : Compliance Control
    // u = u - F;
    return;
}

//////////////////////////////////////////////////////////////////////////////
int main (int argc, char **argv)
{
    ros::init(argc, argv, "whole_body_main_node");

    ros::NodeHandle nh;
    initialize();

    ros::Timer timer_control = nh.createTimer(ros::Duration(0.01), ComplianceSysFunction); // 10ms

    mobile_cmd_vel_pub = nh.advertise<std_msgs::Float64>("/heojun/cmd_vel",10);
    control_input_sub  = nh.subscribe("/control/cmd_vel", 10, inputForce_twistCallback);
    extern_force_sub   = nh.subscribe("/external_force", 10, externForce_Callback);

    ros::Rate loop_rate(125); // 8ms

    while(ros::ok())
    {
        ROS_INFO("------------------------");
        mobile_cmd_vel_pub.publish(mobile_cmd_msg);

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
