#include <ros.h>
#include <navigation/enc_feed.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

navigation::enc_feed feed;
ros::Publisher feedback("feedback",&feed);

int ctrl_inp=0;
int thresh=5;

const int encA=4;
const int encB=5;
#define PWM 16
#define DIR 27
//#define INA 26
//#define INB 27
//#define EN 14
volatile long enc_pos=0;
volatile bool A_set=false;
volatile bool B_set=false;
float angle=0;
float angle_prev=0;
float vel=0;
float dt=25; //50 ms
float curr_time=0;
float prev_time=0;

void pwmCb(const std_msgs::Int16& control_msg){
  ctrl_inp=control_msg.data;
}

ros::Subscriber<std_msgs::Int16> pwm("control", pwmCb );

void callback(){
  A_set=digitalRead(encA);
  B_set=digitalRead(encB);
  if(A_set==B_set){
    enc_pos++;
  }
  else{
    enc_pos--;
  }
}

void setup() {
  pinMode(PWM,OUTPUT);
  pinMode(DIR,OUTPUT);
//  pinMode(INA,OUTPUT);
//  pinMode(INB,OUTPUT);
//  pinMode(EN,OUTPUT);
 
  pinMode(encA,INPUT_PULLUP);
  pinMode(encB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encA),callback,RISING);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(feedback);
  nh.subscribe(pwm);
  //Serial.begin(9600);
}

void loop() {
  curr_time=millis();
  if(curr_time-prev_time>=dt){
    nh.spinOnce();
    angle=enc_pos*(360.0/4800.0);
    vel=(angle-angle_prev)*1000.0/dt;
    if(int(angle/360)>=1){
      angle-=360*int(angle/360);
    }
    else if(angle<0){
      angle+=360*(1+int(angle/360));
    }
    feed.vel=vel;
    feed.angle=angle;
    feedback.publish(&feed);
//    Serial.print("Angle: ");
//    Serial.println(angle);
//    Serial.print("\nVelocity: ");
//    Serial.println(vel);
//    Serial.println();
    if(ctrl_inp>0){
//      digitalWrite(EN,HIGH);
//      digitalWrite(INA,HIGH);
//      digitalWrite(INB,LOW);
      digitalWrite(DIR,0);
      if(abs(ctrl_inp)<thresh){
        analogWrite(PWM,0);
      }
      else{
        analogWrite(PWM,abs(ctrl_inp)); 
      }
    }
    else if(ctrl_inp<0){
//      digitalWrite(EN,HIGH);
//      digitalWrite(INA,LOW);
//      digitalWrite(INB,HIGH);
      digitalWrite(DIR,1);
      if(abs(ctrl_inp)<thresh){
        analogWrite(PWM,0);
      }
      else{
        analogWrite(PWM,abs(ctrl_inp)); 
      }
    }
    else {
      analogWrite(PWM,0);
    }
    prev_time=curr_time;
    angle_prev=angle;
  } 
}
