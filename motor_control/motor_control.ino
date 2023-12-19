#include <ros.h>
#include <navigation/enc_feed.h>
#include <std_msgs/Float32.h>
#include <navigation/target.h>

#define encA 4
#define encB 5
#define PWM 16
#define DIR 27
//#define INA 26
//#define INB 27
//#define EN 14

ros::NodeHandle nh;

navigation::enc_feed feed;
ros::Publisher feedback("feedback",&feed);

std_msgs::Float32 pwm;
ros::Publisher control("control",&pwm); //we'll use these topics while testing to know what is happening

int state=0;
float val=0;
float angle=0;
float angle_prev=0;
float vel=0;
int ctrl_inp=0; //this is the pwm signal that is being given to motors

float error=0;
float error_prev=0;
float error_int=0;
float prev_time=0;
float curr_time=0;

float angle_tolerance=5.0;
float vel_tolerance=2.0;
float kp_pos=3.2;  //different PID constants for position and velocity control
float kd_pos=0.45;
float kp_vel=3.0;
float ki_vel=0.0005;
float kd_vel=0.8;
float output_p=0.0;
float output_d=0.0;
float output_i=0.0;
float output=0.0;

volatile long enc_pos=0;
volatile bool A_set=false;
volatile bool B_set=false;
float dt=50; //50 ms
int thresh=6;  //this threshold is so that when ctrl_inp value goes below this it should stop giving signals

void tarCB(const navigation::target& target_msg){
  if(target_msg.state){
    state=target_msg.state;
    val=target_msg.value;
  }
}

ros::Subscriber<navigation::target> goal("goal",tarCB); //tarCB stands for target callback
 
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

void output_pos(float angle){
  if(abs(val-angle)>angle_tolerance){
    error=val-angle;
     
    if (error>180.0){   //changes the way/direction the motor should move in depending on the goal angle
      error=error-360;
    }
    else if (error<(-180.0)){
      error=error+360;
    }
    if(prev_time!=0){
      output_d=((error-error_prev)/dt)*kd_pos;
      output_p=error*kp_pos;
    }
    else{
      output_d=0;   //if dt=0 then just give both outputs as 0
      output_p=0;
    }
    output=(output_p+output_d);  //see whether it is +ve or -ve
  }
}

void output_vel(float vel){
  if(abs(val-vel)>vel_tolerance){
    error=val-vel;
    if(prev_time!=0){
      output_d=((error-error_prev)/dt)*kd_vel;
      output_p=error*kp_vel;
    }
    else{
      output_d=0;   //if dt=0 then just give both outputs as 0
      output_p=0;
    }
    error_int += error*(curr_time-prev_time);   //this is the integrated error
    output_i=error_int*ki_vel;
    output=(output_p+output_d+output_i);  //see whether it is +ve or -ve
  }
}

void setup(){
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
  nh.advertise(control);
  nh.subscribe(goal);
}

void loop(){
  curr_time=millis();
  if(curr_time-prev_time>=dt){
    nh.spinOnce();
    angle=enc_pos*(360.0/4800.0);
    //Take care of this velocity value
    vel=(angle-angle_prev)*1000.0/(curr_time-prev_time); //either we can use difference between current time and previous time or we can put dt
    angle_prev=angle;
    
    if(int(angle/360)>=1){
      angle-=360*int(angle/360);
    }
    else if(angle<0){
      angle+=360*(1+int(angle/360));
    }
    if(state==1){     //for position control
      output_pos(angle);
    }
    else if(state==2){  //for velocity control
      output_vel(vel);
    }
    else if(state==3){   //for returning to the initial position
      val=0;
      output_pos(angle);
    }
    else{
      output=0.0;
    }
    
    if (output>0){
      ctrl_inp=min(127,int(output));  //so that the maximum output isn't more than 127(half maximum voltage) //this may need changing
    }
    else{
      ctrl_inp=max(-127,int(output));
    }
    
    if(abs(ctrl_inp)<thresh or ctrl_inp==0){  //encompasses the ctrl_inp==0 also
      analogWrite(PWM,0);
    }
    else{
      if(ctrl_inp>0){
//      digitalWrite(EN,HIGH);
//      digitalWrite(INA,HIGH);
//      digitalWrite(INB,LOW);
        digitalWrite(DIR,0);
        analogWrite(PWM,abs(ctrl_inp));
      }
      else if(ctrl_inp<0){
        digitalWrite(DIR,1);
        analogWrite(PWM,abs(ctrl_inp));
      }
    }
    prev_time=curr_time;
    error_prev=error;
    feed.vel=vel;
    feed.angle=angle;
    feedback.publish(&feed);
    pwm.data=output;  //you can switch between output and ctrl_inp to check if putting that limit actually works or not
    control.publish(&pwm);
  }
}
