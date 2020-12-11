#include <ros/ros.h>
#include "load_cell_msgs/LoadCellForceArray.h"
#include "load_cell_msgs/LoadCellForce.h"
#include "std_msgs/Float64.h"

#include <vector>
#include <iostream>
#include <stdlib.h>

int sensor_count = 4;
std::vector<double> sensor_value_list(4);

void area1LoadCellCallback(const std_msgs::Float64::ConstPtr& msg){
  sensor_value_list[0] = static_cast<double>(msg->data);
  return;
}

void area2LoadCellCallback(const std_msgs::Float64::ConstPtr& msg){
  sensor_value_list[1] = static_cast<double>(msg->data);
  return;
}

void area3LoadCellCallback(const std_msgs::Float64::ConstPtr& msg){
  sensor_value_list[2] = static_cast<double>(msg->data);
  return;
}

void area4LoadCellCallback(const std_msgs::Float64::ConstPtr& msg){
  sensor_value_list[3] = static_cast<double>(msg->data);
  return;
}

double kg2forceFomular(double kg){
  double force = 0;
  force = kg / 10;

  return force;
}

double* kg2force(){
  double* force_list;
  force_list = (double *)malloc(4*sizeof(double));
  for(int i=0; i<sensor_value_list.size(); i++){
    force_list[i] = kg2forceFomular(sensor_value_list[i]);
  }
  return force_list;
}

void printData(double* force_list){
  for(int i=0; i<4; i++){
    std::cout << "id : " << i << " : " << force_list[i] << std::endl;
  }
  std::cout << "----------------" << std::endl;
  return;
}

void pubProcess(double* force_list, ros::Publisher& pub){
  load_cell_msgs::LoadCellForceArray pub_msgs;
  std::vector<load_cell_msgs::LoadCellForce> force_datas = {};
  for(int i=0; i<4; i++){
    load_cell_msgs::LoadCellForce force_data;
    force_data.id = i;
    force_data.force = force_list[i];

    force_datas.push_back(force_data);
  }

  pub_msgs.data = force_datas;

  pub.publish(pub_msgs);

  return;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "load_cell_node");
  ros::NodeHandle nh;
  
  ROS_INFO("[load cell node] started.");

  ros::Subscriber sub1 = nh.subscribe("area1_value", 1, area1LoadCellCallback);
  ros::Subscriber sub2 = nh.subscribe("area2_value", 1, area2LoadCellCallback);
  ros::Subscriber sub3 = nh.subscribe("area3_value", 1, area3LoadCellCallback);
  ros::Subscriber sub4 = nh.subscribe("area4_value", 1, area4LoadCellCallback);

  ros::Publisher pub = nh.advertise<load_cell_msgs::LoadCellForceArray>("/load_cell/force_value", 10);
  ros::Rate loop_rate(100);

  double* force_list;
  force_list = (double *)malloc(4*sizeof(double));

  while(ros::ok()){
    force_list = kg2force();

    printData(force_list);

    pubProcess(force_list, pub);

    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
