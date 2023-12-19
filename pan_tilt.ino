#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <Servo.h>
ros::NodeHandle nh;
int flag=0;
Servo myservo;
int myPin=5;
int max_it=20;
int del=500;
int del1=475;

std_msgs::String str_msg;
ros::Publisher chatter("chatter",&str_msg);

void pictures(){
  str_msg.data="taking_pictures";
  chatter.publish(&str_msg);
  for(int i=1; i<=max_it; i++){
    nh.spinOnce();
    delay(1000);
    myservo.attach(myPin);
    myservo.write(50);
    delay(del);
    myservo.detach();
    delay(1000);
  }
  myservo.attach(myPin);
  myservo.write(120);
  delay(max_it*del1);
  myservo.detach();
}


void callback(const std_msgs::Int8& check_msg){
  if(check_msg.data==1) flag=1;
}

ros::Subscriber<std_msgs::Int8> sub("check_node",&callback);
void setup() {  
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  if(flag==1){
    pictures();
    flag=0;
  }
}
