#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <Servo.h>
ros::NodeHandle nh;
bool flag=false;
Servo myservo;
int myPin=6;
int max=20;
int del=500;

std_msgs::String str_msg;
ros::Publisher chatter("chatter",&str_msg);

void rotate(){
  str_msg.data="active";
  chatter.publish(&str_msg);
  for(int i=1; i<=max; i++){
    //myservo.detach();
    //myservo.attach(myPin);
    delay(1000);
    myservo.attach(myPin);
    myservo.write(86);
    delay(del);
    myservo.detach();
    delay(1000);
  }
  myservo.attach(myPin);
  myservo.write(97);
  delay(max*del);
  myservo.detach();
  //exit(0);
}

void callback(const std_msgs::Bool& check_msg){
  if(check_msg.data) flag=true;
}

ros::Subscriber<std_msgs::Bool> sub("check_node",&callback);
void setup() {  
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  myservo.attach(myPin);
}

void loop() {
  nh.spinOnce();
  if(flag){
    rotate();
    exit(0);
  }
  //flag=false;
  //delay(100);
}
