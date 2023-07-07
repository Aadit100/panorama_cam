#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <Servo.h>
ros::NodeHandle nh;
int flag=0;
Servo myservo;
int myPin=6;
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
    myservo.write(86);
    delay(del);
    myservo.detach();
    delay(1000);
  }
  myservo.attach(myPin);
  myservo.write(97);
  delay(max_it*del1);
  myservo.detach();
}
void rotate_back(){
  str_msg.data="rotating_camera_back";
  chatter.publish(&str_msg);
  for(int i=1; i<=(max_it/2); i++){
    nh.spinOnce();
    myservo.attach(myPin);
    myservo.write(86);
    delay(del1);
    myservo.detach();
  }
}
void rotate_forward(){
  str_msg.data="rotating_camera_forward";
  chatter.publish(&str_msg);
  for(int i=1; i<=(max_it/2); i++){
    nh.spinOnce();
    myservo.attach(myPin);
    myservo.write(97);
    delay(del1);
    myservo.detach();
  }
}

void callback(const std_msgs::Int8& check_msg){
  if(check_msg.data==1) flag=1;
  if(check_msg.data==2) flag=2;
  if(check_msg.data==3) flag=3;
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
  else if(flag==2){
    rotate_back();
    flag=0;
  }
  else if(flag==3){
    rotate_forward();
    flag=0;
  }
}
