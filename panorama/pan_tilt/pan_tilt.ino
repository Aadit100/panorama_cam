#include <ESP32Servo.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
ros::NodeHandle nh;
Servo servo1;  //pan motor
Servo servo2;  //tilt motor
int servoPin1=4;  //attach pan motor to pin32
int servoPin2=0;  //attach tilt motor to pin33
int init_pan=96;
int init_tilt=95;
int val1=init_pan;  //this is for pan motor so this is the value where it does not rotate
int val2=init_tilt; //this is the value for tilt motor so value is 0 initially
float feed1=0;
float feed2=0;
float curr_time=0;
float prev_time=0;
bool flag=0;
int it=0;   // this is the current iteration while taking photos
bool state=0; //whether the pan servo is attached or detached
int max_it=20;  //this is the max iterations
int del=200; //this is the delay between writing values
int del1=1000;

std_msgs::Bool cam_state;  //whether joystick controlled or panorama mode
ros::Publisher check("check",&cam_state);

void joyCallback(const sensor_msgs::Joy& msg){
  feed1=msg.axes[6];  //we will have to change these axes according to what we want to map them to
  feed2=msg.axes[7];
  if(msg.buttons[4]){
  flag=(not flag);
    if(flag){
      it=0;
      prev_time=curr_time;
    }
  }
}

void pantilt(){
  if(feed1!=0){
    servo1.attach(servoPin1);
    if(feed1==1){
      val1=99;
    }
    else{
      val1=92;
    }
  }
  else{
    //val1=init_pan;  //not needed since it will have already detached so write won't work anyways
    servo1.detach();  //better to detach continuous rotation servo when not in use
  }
  if(feed2!=0){
    servo2.attach(servoPin2);
    if(feed2==1){
      val2=99;
    }
    else{
      val2=92;
    }
  }
  else{
    //val2=init_tilt;
    servo2.detach();  //better to detach continuous rotation servo when not in use
  }
}

ros::Subscriber<sensor_msgs::Joy> sub("joy",&joyCallback);

void setup(){
  //Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(check);  //to see whether it is being controlled by joystick or not
}

void loop(){
  curr_time=millis();
  nh.spinOnce();
  if(curr_time-prev_time>del and flag==0){  //provides a delay after writing as well as after detaching
    pantilt();
    servo1.write(val1);
    servo2.write(val2);
    prev_time=curr_time;
  }
  else if(flag==1){   //only pan is needed in this case so tilt has to be set constant before this is run
    if(it<max_it){
      if(curr_time-prev_time>del1){
        if(state=0){
          servo1.attach(servoPin1);
          servo1.write(99);
        }
        else{
          servo1.detach();
          it++;
        }
        state=not state;
        prev_time=curr_time;
      }
    }
    else{   //this will cause it to rotate back and then shift control back to joystick
      if(curr_time-prev_time<del1*max_it){
        servo1.attach(servoPin1);
        servo1.write(92);
      }
      else{
        servo1.detach();
        flag=0;
        prev_time=curr_time;
      }
    }
  }
  cam_state.data=bool(flag);
  check.publish(&cam_state);
}
