/*
 * motor_control.cpp
 *
 *      Author: junyoung / lgkimjy
 */
#include <mobile_robot/motor_control.h>
#include <mobile_robot/motor_cmd.h>

///////////////////////*    initialize    *///////////////////////
void initialize()
{
  motor1 = new DcMotorForRaspberryPi(EncoderCountsPerWheel, wheelradius, 5000.0);
  motor2 = new DcMotorForRaspberryPi(EncoderCountsPerWheel, wheelradius, 5000.0);

  m.insert(make_pair(1, 0));	m.insert(make_pair(3, 1));
	m.insert(make_pair(2, 2));	m.insert(make_pair(6, 3));
	m.insert(make_pair(4, 4));	m.insert(make_pair(5, 5));
  
  // K_P = 5000.0;
  joy = 0;
}

/////////////////////*    Callback    */////////////////////
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  joy = 0.5 * msg->axes[7];
}
void motor_first_command_callback(const std_msgs::Float64::ConstPtr& msg)
{
  motor1->ref_speed = msg->data;
  return ;
}
void motor_second_command_callback(const std_msgs::Float64::ConstPtr& msg)
{
  motor2->ref_speed = msg->data;
  return ;
}

////////////////////////////*    Control & Funcs    *////////////////////////////
// void go(float speed)
// {
//   ref_speed = speed;
// }

void ControlSpeed(int pwm_pin, int direction_pin, DcMotorForRaspberryPi* motor)
{
  motor->err_speed  =  motor->ref_speed - motor->vel;

  motor->up = motor->K_P * (float)motor->err_speed;

  motor->usum = motor->up;

  ROS_INFO("in control : ref_spd = %f", motor->ref_speed);
  ROS_INFO("in control : motor->vel = %f", motor->vel);
  // ROS_INFO("in control : err_speed = %f", motor->err_speed);

  //  ROS_INFO("in control : usum = %f", motor->usum);

  if(motor->usum < 0){
    motor->u_in = motor->usum * (-1);
    digitalWrite(direction_pin, LOW);
  }
  else{
    motor->u_in = motor->usum;
    digitalWrite(direction_pin, HIGH);
  }

  if(motor->u_in > 512){
    motor->u_in = 512;
  }

  pwmWrite(pwm_pin, motor->u_in);
}

void speedCalc(DcMotorForRaspberryPi* motor)
{
  motor->current_time = ros::Time::now();
  motor->dt = (motor->current_time - motor->last_time).toSec();
  motor->vel = (motor->encoder_pulse - motor->previous_pulse) * motor->DistancePerCount/motor->dt;

  ROS_INFO("in speedCalc : diff_pulse = %f", (motor->encoder_pulse - motor->previous_pulse));
  // ROS_INFO("in speedCalc : vel = %f", motor->vel);
  motor->previous_pulse = motor->encoder_pulse;
  motor->last_time = motor->current_time;
}

////////////////////////*    ISR & Timer    *////////////////////////
void motor1_encoder(void)
{
  motor1->num = 4*digitalRead(motor1_H_A) + 2*digitalRead(motor1_H_B) + digitalRead(motor1_H_C);
  
  // ROS_INFO("%d,%d , %d | %d%d%d", m[motor1->num], m[motor1->before_num] , motor1->num, digitalRead(motor1_H_A), digitalRead(motor1_H_B), digitalRead(motor1_H_C));
  if(m[motor1->before_num] == 5 && m[motor1->num] == 0)
    motor1->encoder_pulse ++;
  else if(m[motor1->before_num] == 0 && m[motor1->num] == 5)
    motor1->encoder_pulse --;
  else if(m[motor1->num] > m[motor1->before_num])
    motor1->encoder_pulse ++;
  else
    motor1->encoder_pulse --;

  motor1->current_pulse = motor1->encoder_pulse;
  motor1->before_num = motor1->num;
}
void motor2_encoder(void)
{
  motor2->num = 4*digitalRead(motor2_H_A) + 2*digitalRead(motor2_H_B) + digitalRead(motor2_H_C);
  
  // ROS_INFO("%d,%d , %d | %d%d%d", m[motor2->num], m[motor2->before_num] , motor2->num, digitalRead(motor2_H_A), digitalRead(motor2_H_B), digitalRead(motor2_H_C));
  if(m[motor2->before_num] == 5 && m[motor2->num] == 0)
    motor2->encoder_pulse ++;
  else if(m[motor2->before_num] == 0 && m[motor2->num] == 5)
    motor2->encoder_pulse --;
  else if(m[motor2->num] > m[motor2->before_num])
    motor2->encoder_pulse ++;
  else
    motor2->encoder_pulse --;

  motor2->current_pulse = motor2->encoder_pulse;
  motor2->before_num = motor2->num;
}

void controlFunction(const ros::TimerEvent&)
{
  ROS_INFO("%f   | %f   ", motor1->encoder_pulse, motor2->encoder_pulse);
  speedCalc(motor1);
  ControlSpeed(motor1_PWM, motor1_DIR, motor1);
  speedCalc(motor2);
  ControlSpeed(motor2_PWM, motor2_DIR, motor2);
}

/////////////////*    MAIN    */////////////////
int main (int argc, char **argv)
{
  printf("Motor node Start \n");

  wiringPiSetupGpio();
  initialize();

  ros::init(argc, argv, "motor_control");
  ros::NodeHandle nh;

  ros::Timer timer_control = nh.createTimer(ros::Duration(0.02), controlFunction); // 20ms

  std::string motor_first_topic = nh.param<std::string>("motor_first", "");
  std::string motor_second_topic = nh.param<std::string>("motor_second", "");

  motor_first_command_sub   = nh.subscribe(motor_first_topic, 1, motor_first_command_callback);
  motor_second_command_sub   = nh.subscribe(motor_second_topic, 1, motor_second_command_callback);

  motor_joy_sub            = nh.subscribe("joy", 10, joy_callback);

  pinMode(motor1_IN1, OUTPUT);
  pinMode(motor1_BREAK, OUTPUT);
  pinMode(motor1_DIR, OUTPUT);
  pinMode(motor1_FG, INPUT);

  pinMode(motor2_IN1, OUTPUT);
  pinMode(motor2_BREAK, OUTPUT);
  pinMode(motor2_DIR, OUTPUT);
  pinMode(motor2_FG, INPUT);

  pullUpDnControl(motor1_H_A, PUD_UP);
  pullUpDnControl(motor1_H_B, PUD_UP);
  pullUpDnControl(motor1_H_C, PUD_UP);
  pullUpDnControl(motor2_H_A, PUD_UP);
  pullUpDnControl(motor2_H_B, PUD_UP);
  pullUpDnControl(motor2_H_C, PUD_UP);

  wiringPiISR(motor1_FG, INT_EDGE_BOTH, &motor1_encoder);
  wiringPiISR(motor2_FG, INT_EDGE_BOTH, &motor2_encoder);

  pinMode(motor1_PWM, PWM_OUTPUT);
  pinMode(motor2_PWM, PWM_OUTPUT);
  pwmSetMode(PWM_MODE_MS);
  pwmSetRange(512);
  pwmSetClock(45);

  digitalWrite(motor1_IN1,LOW);
  digitalWrite(motor1_BREAK,LOW);
  digitalWrite(motor2_IN1,LOW);
  digitalWrite(motor2_BREAK,LOW);

  pwmWrite(motor1_PWM, 0);
  pwmWrite(motor2_PWM, 0);

  ros::Rate loop_rate(1000);  //1ms
  while(ros::ok())
  {
    // go(joy);
    loop_rate.sleep();
    ros::spinOnce();
  }

  delete motor1;
  delete motor2;
  pwmWrite(motor1_PWM, 0);
  pwmWrite(motor2_PWM, 0);

  return 0;

}