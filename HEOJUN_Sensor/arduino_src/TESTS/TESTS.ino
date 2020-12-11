/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include "HX711.h"

#define area1_calibration_factor 155000
#define AREA1_DOUT  A1
#define AREA1_CLK   A0

ros::NodeHandle  nh;

std_msgs::Float64 area1_value;
ros::Publisher area1_publisher("area1_value", &area1_value);
float area1_weight;

HX711 area1_scale(AREA1_DOUT, AREA1_CLK);

void setup()
{
  nh.initNode();

  area1_scale.set_scale(area1_calibration_factor);
  area1_scale.tare();
  
  nh.advertise(area1_publisher);
}

void loop()
{
  area1_weight = area1_scale.get_units();
  area1_value.data = area1_weight;
  
  area1_publisher.publish(&area1_value);
  nh.spinOnce();
  delay(10);
}


/*
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>



ros::NodeHandle nh;

HX711 area1_scale(AREA1_DOUT, AREA1_CLK);

std_msgs::String area1_value;
ros::Publisher area1_publisher("area1_value", &area1_value);
float area1_weight;

void setup() 
{ 
  nh.initNode();
  
  area1_scale.set_scale(area1_calibration_factor);
  area1_scale.tare();

  nh.advertise(area1_publisher);
}

 

void loop() {

  area1_weight = area1_scale.get_units();

  area1_value.data = "area1_weight";
  area1_publisher.publish(&area1_value);

  nh.spinOnce();
  delay(1000);
}*/
