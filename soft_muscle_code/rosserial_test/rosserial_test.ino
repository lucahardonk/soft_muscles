#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void setup() {
  nh.initNode();
  nh.advertise(chatter);
}

void loop() {
  str_msg.data = "Hello, ROS!";
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(1000);
}

