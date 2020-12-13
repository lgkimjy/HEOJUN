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
    force_y = 0;

    dt = 0;
}

//callbacks///////////////////////////////////////////////
void inputForceCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    control_input_vx = msg->linear.x;
    control_input_vy = msg->linear.y;
    control_input_wz = msg->angular.z;

    acc_x = control_input_vx/0.05;   // joystick data callback cycle = 0.05
    acc_y = control_input_vy/0.05;   // joystick data callback cycle = 0.05

    force_x = mobile_mass * acc_x;
    force_y = mobile_mass * acc_y;

    // ROS_INFO("control force x =  %f", force_x);
    // ROS_INFO("control force y =  %f", force_y);

    return;
}
void externForceCallback(const load_cell_msgs::LoadCellForceVector::ConstPtr& msg)
{
    e_force_x = msg->x * msg->force;
    e_force_y = msg->y * msg->force;

    // ROS_INFO("%f", msg->x);
    // ROS_INFO("%f", msg->y);
    // ROS_INFO("%f", msg->force);
    // ROS_INFO("external force x = %f", e_force_x);
    // ROS_INFO("external force y = %f", e_force_y);

	return;
}

//TimerInterrupts///////////////////////////////////////////////
void ComplianceSysFunction(const ros::TimerEvent&)
{
    ROS_INFO("------------------------");
    // // control input force
    // ROS_INFO("vel_x = %9.4f    |  vel_y = %9.4f", control_input_vx, control_input_vy);
    // ROS_INFO("acc_x = %9.3f    |  acc_y = %9.3f", acc_x, acc_y);
    // ROS_INFO("force_x = %9.2f  |  force_y = %9.2f", force_x, force_y);
    // // external sensor force
    // ROS_INFO("force_x = : %9.2f  |  force_y = : %9.2f", force_x, force_y);
    // ROS_INFO("acc_x = : %9.3f    |  acc_y = : %9.3f", acc_x, acc_y);
    // ROS_INFO("vel_x = : %9.4f    |  vel_y = : %9.4f", control_input_vx, control_input_vy);

    ROS_INFO("input vel x = %f", control_input_vx);
    ROS_INFO("input vel y = %f", control_input_vy);

    ROS_INFO("-------------------------------------");
    control_input_vx = ((force_x - e_force_x) / mobile_mass ) * 0.05;
    control_input_vy = ((force_y - e_force_y) / mobile_mass ) * 0.05;

    ROS_INFO("control force x =  %f", force_x);
    ROS_INFO("control force y =  %f", force_y);
    ROS_INFO("external force x = %f", e_force_x);
    ROS_INFO("external force y = %f", e_force_y);


    ROS_INFO("output vel x = %f", control_input_vx);
    ROS_INFO("output vel y = %f", control_input_vy);

    mobile_cmd_msg.linear.x = control_input_vx;
    mobile_cmd_msg.linear.y = control_input_vy;
    mobile_cmd_msg.angular.z = control_input_wz;

    mobile_cmd_vel_pub.publish(mobile_cmd_msg);

    return;
}


//////////////////////////////////////////////////////////////////////////////
int main (int argc, char **argv)
{
    ros::init(argc, argv, "whole_body_node");

    ros::NodeHandle nh;
    initialize();

    ros::Timer timer_control = nh.createTimer(ros::Duration(0.01), ComplianceSysFunction); // 10ms

    mobile_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/heojun/cmd_vel",10);

    control_input_sub  = nh.subscribe("/control/cmd_vel", 10, inputForceCallback);
    extern_force_sub   = nh.subscribe("/control/external_force", 10, externForceCallback);

    ros::Rate loop_rate(125); // 10ms

    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
