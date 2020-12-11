// calibration factor
// area 1 : 155000
// area 2 : 182500
// area 3 : 176000
// area 4 : 183000

#include "HX711.h"
#include <ros.h>
#include <std_msgs/Float64.h>

#define area1_calibration_factor 155000
#define area2_calibration_factor 182500
#define area3_calibration_factor 176000
#define area4_calibration_factor 183000

#define AREA1_DOUT  A1
#define AREA1_CLK   A0
#define AREA2_DOUT  A3
#define AREA2_CLK   A4
#define AREA3_DOUT  A6
#define AREA3_CLK   A7
#define AREA4_DOUT  A8
#define AREA4_CLK   A9

HX711 area1_scale(AREA1_DOUT, AREA1_CLK);
HX711 area2_scale(AREA2_DOUT, AREA2_CLK);
HX711 area3_scale(AREA3_DOUT, AREA3_CLK);
HX711 area4_scale(AREA4_DOUT, AREA4_CLK);

ros::NodeHandle nh;

std_msgs::Float64 area1_value, area2_value, area3_value, area4_value;
ros::Publisher area1_publisher("area1_value", &area1_value);
ros::Publisher area2_publisher("area2_value", &area2_value);
ros::Publisher area3_publisher("area3_value", &area3_value);
ros::Publisher area4_publisher("area4_value", &area4_value);

void setup() {

  Serial.begin(57600);

  Serial.println("HX711 scale demo");

  area1_scale.set_scale(area1_calibration_factor);
  area2_scale.set_scale(area2_calibration_factor);
  area3_scale.set_scale(area3_calibration_factor);
  area4_scale.set_scale(area4_calibration_factor);

  area1_scale.tare();
  area2_scale.tare();
  area3_scale.tare();
  area4_scale.tare();
  
  Serial.println("Readings:");

  nh.initNode();
  nh.advertise(area1_publisher);
  nh.advertise(area2_publisher);
  nh.advertise(area3_publisher);
  nh.advertise(area4_publisher);
}

 

void loop() {

//  Serial.print("Reading: ");
//  Serial.println();

  float area1_weight = area1_scale.get_units()*453.592;
  float area2_weight = area2_scale.get_units()*453.592;
  float area3_weight = area3_scale.get_units()*453.592;
  float area4_weight = area4_scale.get_units()*453.592;

  area1_value.data = area1_weight;
  area2_value.data = area2_weight;
  area3_value.data = area3_weight;
  area4_value.data = area4_weight;

  area1_publisher.publish(&area1_value);
  area2_publisher.publish(&area2_value);
  area3_publisher.publish(&area3_value);
  area4_publisher.publish(&area4_value);

  Serial.print("area1 weight : ");
  Serial.print(area1_weight, 3);
  Serial.print(" g");
  Serial.println();
//
//  Serial.print("area2 weight : ");
//  Serial.print(area2_weight, 3);
//  Serial.print(" g");
//  Serial.println();
//
//  Serial.print("area3 weight : ");
//  Serial.print(area3_weight, 3);
//  Serial.print(" g");
//  Serial.println();
//
//  Serial.print("area4 weight : ");
//  Serial.print(area4_weight, 3);
//  Serial.print(" g");
//  Serial.println();
//
//  Serial.print("sum weight : ");
//  Serial.print(area1_weight + area2_weight + area3_weight + area4_weight, 3);
//  Serial.print(" g");
//  Serial.println();
//  Serial.print("-------------------------------------------------");

  Serial.println();
  nh.spinOnce();
//  delay(500);
}
