/*
 * motor_control.cpp
 *
 *      Author: junyoung / lgkimjy
 */
#include <mobile_manager/mobile_manager_node.h>
#include <mobile_manager/motor_cmd.h>

void initialize()
{
  mobile_vx = 0;
  mobile_vy = 0;
  mobile_wz = 0;
}


//callback///////////////////////////////////////////////
/*
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  if(msg->buttons[4] == 1)
  {
    m_vx = 0;
    m_vy = 0;
    m_wz = msg->buttons[4];
  }  
  else if(msg->buttons[5] == 1)
  {
    m_vx = 0;
    m_vy = 0;
    m_wz = -1 *  msg->buttons[5];
  }
  else
  {
    m_vx = msg->axes[1] / 2;
    m_vy = msg->axes[0] / 2;
    m_wz = 0;
  }
}
*/

void cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  mobile_vx = msg->linear.x;
  mobile_vy = msg->linear.y;
  mobile_wz = msg->angular.z;

  mecanum_kinematics();
}

///////////////////////////////////////////////////////////////////////////
void mecanum_kinematics(void)
{
  m1_v = mobile_vx + (-1 * mobile_vy) - (L+W) * mobile_wz;
  m2_v = mobile_vx + mobile_vy + (L+W) * mobile_wz;
  m3_v = mobile_vx + mobile_vy - (L+W) * mobile_wz;
  m4_v = mobile_vx + (-1 * mobile_vy) + (L+W) * mobile_wz;

  ROS_INFO("m1_v | %.4f", m1_v);
  ROS_INFO("m2_v | %.4f", m2_v);
  ROS_INFO("m3_v | %.4f", m3_v);
  ROS_INFO("m4_v | %.4f", m4_v);

  motor_cmd_msg_1.data = m1_v;
  motor_cmd_msg_2.data = -1 * m2_v;
  motor_cmd_msg_3.data = m3_v;
  motor_cmd_msg_4.data = -1 * m4_v;
}

//////////////////////////////////////////////////////////////////////////////
int main (int argc, char **argv)
{
  ros::init(argc, argv, "mobile_manager_node");

  ros::NodeHandle nh;
  initialize();

  motor1_pub = nh.advertise<std_msgs::Float64>("/motor_1",10);
  motor2_pub = nh.advertise<std_msgs::Float64>("/motor_2",10);
  motor3_pub = nh.advertise<std_msgs::Float64>("/motor_3",10);
  motor4_pub = nh.advertise<std_msgs::Float64>("/motor_4",10);

  // ros::Subscriber joy_sub   = nh.subscribe("/joy", 10, joy_callback);
  ros::Subscriber cmd_vel_sub   = nh.subscribe("/heojun/cmd_vel", 10, cmd_vel_Callback);
  ros::Rate loop_rate(125); // 8ms

  while(ros::ok())
  {
    ROS_INFO("------------------------");
    mecanum_kinematics();

    motor1_pub.publish(motor_cmd_msg_1);
    motor2_pub.publish(motor_cmd_msg_2);
    motor3_pub.publish(motor_cmd_msg_3);
    motor4_pub.publish(motor_cmd_msg_4);

    loop_rate.sleep();

    ros::spinOnce();
  }
  return 0;
}