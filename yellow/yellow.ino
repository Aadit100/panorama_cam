#include <ros.h>
#include <std_msgs/Int8.h>
ros::NodeHandle nh;
int bjt=8;
bool flag=false;

void callback(const std_msgs::Int8& msg){
  if(msg.data) flag=true;
  else flag=false;
}

ros::Subscriber<std_msgs::Int8> sub("indicator", &callback);
void setup() {
  nh.initNode();
  pinMode(bjt,OUTPUT);
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  if(flag) digitalWrite(bjt,1);
  else digitalWrite(bjt,0);
}
