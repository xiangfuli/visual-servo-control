#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <ros/console.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "visual_servo_control");

  ros::NodeHandle n;

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
