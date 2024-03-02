#include <ros.h>
#include <traversal/WheelVel.h>
#include <std_msgs/Int8.h>
#include <navigation/enc_feed.h>
#include <std_msgs/Float32.h>

#define str_encA_rf 42
#define str_encB_rf 2
#define str_encA_rb 38
#define str_encB_rb 39
#define str_encA_lf 5
#define str_encB_lf 6
#define str_encA_lb 16
#define str_encB_lb 17

#define str_pwm_rf 20
#define str_dir_rf 19
#define str_pwm_rb 47
#define str_dir_rb 21
#define str_pwm_lf 13
#define str_dir_lf 14
#define str_pwm_lb 11
#define str_dir_lb 12

#define dr_pwm_rf 35
#define dr_dir_rf 48
#define dr_pwm_rb 37
#define dr_dir_rb 36
#define dr_pwm_lf 9
#define dr_dir_lf 10
#define dr_pwm_lb 18
#define dr_pwm_lb 8

ros::NodeHandle nh;

navigation::enc_feed feed;

//std_msgs::Float32 pwm;
//ros::Publisher control("control",&pwm);

float angle_tolerance=5.0;
bool rot_status=true;  //used to make sure that joystick inputs are not acted upon unless the wheels are in the correct orientation
float kp=0;
float kd=0;
float ki=0;

float error_rf=0, error_rf_i=0, prev_error_rf=0;  //it is understood that this is for steering
float error_rb=0, error_rb_i=0, prev_error_rb=0;  //prev_error is defined for using with derivative
float error_lf=0, error_lf_i=0, prev_error_lf=0;
float error_lb=0; error_lb_i=0, prev_error_lb=0;

float angle_rf=0;
float angle_rb=0;
float angle_lf=0;
float angle_lb=0;
float calib=1; //angle will be defined by dividing enc position by calibration constant

int vel=0, vel1=0, orient=0, om=0, omega=0, pwmr=0, pwml=0;
int str_pwmrf=0, str_pwmrb=0, str_pwmlf=0, str_pwmlb=0;  //outputs to all the 4 wheels
int dr_pwmrf=0, dr_pwmrb=0, dr_pwmlf=0, dr_pwmlb=0;   //not to be confused with the pins with similar names since these are the values that will be published

float str_target_lf=0;
float str_target_lb=0;
float str_target_rf=0;
float str_target_rb=0;

volatile long str_enc_pos_rf=0, str_enc_pos_rb=0, str_enc_pos_lf=0, str_enc_pos_lb=0;
volatile bool A_set_rf=false, A_set_rb=false, A_set_lf=false, A_set_lb=false;
volatile bool B_set_rf=false, B_set_rb=false, B_set_lf=false, B_set_lb=false;

void callback_rf(){
  A_set_rf=digitalRead(str_encA_rf);
  B_set_rf=digitalRead(str_encB_rf);
  if(A_set_rf==B_set_rf){
    str_enc_pos_rf++;
  }
  else{
    str_enc_pos_rf--;
  }
}

void callback_rb(){
  A_set_rb=digitalRead(str_encA_rb);
  B_set_rb=digitalRead(str_encB_rb);
  if(A_set_rb==B_set_rb){
    str_enc_pos_rb++;
  }
  else{
    str_enc_pos_rb--;
  }
}

void callback_lf(){
  A_set_lf=digitalRead(str_encA_lf);
  B_set_lf=digitalRead(str_encB_lf);
  if(A_set_lf==B_set_lf){
    str_enc_pos_lf++;
  }
  else{
    str_enc_pos_lf--;
  }
}

void callback_lb(){
  A_set_lb=digitalRead(str_encA_lb);
  B_set_lb=digitalRead(str_encB_lb);
  if(A_set_lb==B_set_lb){
    str_enc_pos_lb++;
  }
  else{
    str_enc_pos_lb--;
  }
}

void roverMotionCallback(const traversal::WheelVel& msg){
  vel1=int(msg.vel);
  orient=int(msg.orientation);
  om=int(msg.omega);
  if(orient==0){
    str_target_rf=0, str_target_rb=0, str_target_lf=0, str_target_lb=0;
  }
  else if(orient==1){
    str_target_rf=45, str_target_rb=-45, str_target_lf=-45, str_target_lb=45;
  }
  else if(orient==2){
    str_target_rf=90, str_target_rb=90, str_target_lf=90, str_target_lb=90;
  }
  else if(orient==3){
    str_target_rf=-90, str_target_rb=-90, str_target_lf=-90, str_target_lb=-90;
  }
}

ros::Subscriber<traversal::WheelVel> locomotion_sub("motion", &roverMotionCallback);

void str_control(){
  error_rf=str_target_rf-angle_rf;   //angle_rf needs to be defined
  error_rb=str_target_rb-angle_rb;
  error_lf=str_target_lf-angle_lf;
  error_lb=str_target_lb-angle_lb;
  error_rf_i+=error_rf*(curr_time-prev_time)/1000;
  error_rb_i+=error_rb*(curr_time-prev_time)/1000;
  error_lf_i+=error_lf*(curr_time-prev_time)/1000;
  error_lb_i+=error_lb*(curr_time-prev_time)/1000;
  if(abs(error_rf)>angle_tolerance or abs(error_rb)>angle_tolerance or abs(error_lf)>angle_tolerance or abs(error_lb)>angle_tolerance){
    rot_status=false;  //since rotation is not completed till now
    str_pwmrf=kp*error_rf+kd*(error_rf-prev_error_rf)*1000/(curr_time-prev_time)+ki*error_rf_i;
    str_pwmrb=kp*error_rb+kd*(error_rb-prev_error_rb)*1000/(curr_time-prev_time)+ki*error_rb_i;
    str_pwmlf=kp*error_lf+kd*(error_lf-prev_error_lf)*1000/(curr_time-prev_time)+ki*error_lf_i;
    str_pwmlb=kp*error_lb+kd*(error_lb-prev_error_lb)*1000/(curr_time-prev_time)+ki*error_lb_i;
    rot_status=false;
  }
  else{
    str_pwmrf=0; str_pwmrb=0; str_pwmlf=0; str_pwmlb=0;
    rot_status=true;
  }
  prev_error_rf=error_rf; prev_error_rb=error_rb; prev_error_lf=error_lf; prev_error_lb=error_lb;
}

void setup(){
  pinMode(dr_pwm_rf,OUTPUT); pinMode(dr_dir_rf,OUTPUT);
  pinMode(dr_pwm_rb,OUTPUT); pinMode(dr_dir_rb,OUTPUT);
  pinMode(dr_pwm_lf,OUTPUT); pinMode(dr_dir_lf,OUTPUT);
  pinMode(dr_pwm_lb,OUTPUT); pinMode(dr_dir_lb,OUTPUT);
  pinMode(str_pwm_rf,OUTPUT); pinMode(str_dir_rf,OUTPUT);
  pinMode(str_pwm_rb,OUTPUT); pinMode(str_dir_rb,OUTPUT);
  pinMode(str_pwm_lf,OUTPUT); pinMode(str_dir_lf,OUTPUT);
  pinMode(str_pwm_lb,OUTPUT); pinMode(str_dir_lb,OUTPUT);
  pinMode(str_encA_rf,INPUT_PULLUP); pinMode(str_encB_rf,INPUT_PULLUP);
  pinMode(str_encA_rb,INPUT_PULLUP); pinMode(str_encB_rb,INPUT_PULLUP);
  pinMode(str_encA_lf,INPUT_PULLUP); pinMode(str_encB_lf,INPUT_PULLUP);
  pinMode(str_encA_lb,INPUT_PULLUP); pinMode(str_encB_lb,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(str_encA_rf,callback_rf,RISING);
  attachInterrupt(digitalPinToInterrupt(str_encA_rb,callback_rb,RISING);
  attachInterrupt(digitalPinToInterrupt(str_encA_lf,callback_lf,RISING);
  attachInterrupt(digitalPinToInterrupt(str_encA_lb,callback_lb,RISING);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(locomotion_sub);
}

void loop(){
  nh.spinOnce();
  curr_time=millis();
  vel=vel1-127;
  omega=om-127;
  pwmr=int(vel+omega);
  pwml=int(vel-omega);
  str_control();
  if(str_pwmrf>=0) {
    digitalWrite(str_dir_rf,HIGH);
  }
  else{
    digitalWrite(str_dir_rf,LOW);
  }
  if(str_pwmrb>=0){
    digitalWrite(str_dir_rb,HIGH);
  }
  else{
    digitalWrite(str_dir_rb,LOW);
  }
  if(str_pwmlf>=0){
    digitalWrite(str_dir_lf,HIGH);
  }
  else{
    digitalWrite(str_dir_lf,LOW);
  }
  if(str_pwmlb>=0){
    digitalWrite(str_dir_lb,HIGH);
  }
  else{
    digitalWrite(str_dir_lb,LOW);
  }
  analogWrite(str_pwm_rf,abs(str_pwmrf));
  analogWrite(str_pwm_rb,abs(str_pwmrb));
  analogWrite(str_pwm_lf,abs(str_pwmlf));
  analogWrite(str_pwm_lb,abs(str_pwmlb));

  if(rot_status){
    if(orient==0){
      if(pwmr>=0){
        digitalWrite(dr_dir_rf,HIGH);
        digitalWrite(dr_dir_rb,HIGH);
      }
      else{
        digitalWrite(dr_dir_rf,LOW);
        digitalWrite(dr_dir_rb,LOW);
      }
      if(pwml>=0){
        digitalWrite(dr_dir_lf,HIGH);
        digitalWrite(dr_dir_lb,HIGH);
      }
      else{
        digitalWrite(dr_dir_lf,LOW);
        digitalWrite(dr_dir_lb,LOW);
      }
      analogWrite(dr_pwm_rf,abs(pwmr));
      analogWrite(dr_pwm_rb,abs(pwmr));
      analogWrite(dr_pwm_lf,abs(pwml));
      analogWrite(dr_pwm_lb,abs(pwml));
    }

    else if(orient==1){
      if(vel>=0){
        digitalWrite(dr_dir_rf,HIGH);
        digitalWrite(dr_dir_rb,HIGH);
        digitalWrite(dr_dir_lf,LOW);
        digitalWrite(dr_dir_lb,LOW);
      }
      else{
        digitalWrite(dr_dir_rf,LOW);
        digitalWrite(dr_dir_rb,LOW);
        digitalWrite(dr_dir_lf,HIGH);
        digitalWrite(dr_dir_lb,HIGH);
      }
      analogWrite(dr_pwm_rf,abs(vel));
      analogWrite(dr_pwm_rb,abs(vel));
      analogWrite(dr_pwm_lf,abs(vel));
      analogWrite(dr_pwm_lb,abs(vel));
    }
    
    else{
      if(vel>=0){
        digitalWrite(dr_dir_rf,HIGH);
        digitalWrite(dr_dir_rb,HIGH);
        digitalWrite(dr_dir_lf,HIGH);
        digitalWrite(dr_dir_lb,HIGH);
      }
      else{
        digitalWrite(dr_dir_rf,LOW);
        digitalWrite(dr_dir_rb,LOW);
        digitalWrite(dr_dir_lf,LOW);
        digitalWrite(dr_dir_lb,LOW);
      }
      analogWrite(dr_pwm_rf,abs(vel));
      analogWrite(dr_pwm_rb,abs(vel));
      analogWrite(dr_pwm_lf,abs(vel));
      analogWrite(dr_pwm_lb,abs(vel));
    }
  }
  
  else{
    analogWrite(dr_pwm_rf,0);
    analogWrite(dr_pwm_rb,0);
    analogWrite(dr_pwm_lf,0);
    analogWrite(dr_pwm_lb,0);
  }
  
  delay(10);
}
