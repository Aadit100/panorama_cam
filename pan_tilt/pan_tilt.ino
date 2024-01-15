#include <ESP32Servo.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>
ros::NodeHandle nh;
Servo servo1;  //pan motor
Servo servo2;  //tilt motor
int servoPin1=4;  //attach pan motor to pin4
int servoPin2=0;  //attach tilt motor to pin0
int ADC_Max=4096;
int init_pan=96;
int init_tilt=80;
int val1=init_pan;  //this is for pan motor so this is the value where it does not rotate
int val2=init_tilt; //this is the value for tilt motor so value is 0 initially
float feed1=0;
float feed2=0;
ros::Subscriber<sensor_msgs::Joy> sub("joy",&joyCallback);

void joyCallback(const sensor_msgs::Joy& msg){
  feed1=msg.axes[4];  //we will have to change these axes according to what we want to map them to
  feed2=msg.axes[5];
}

void pantilt(){
  if(feed1!=0){
    servo1.attach(servoPin1);
    val1=int(feed1*15.0+init_pan);   //to keep speed between 96 and 110;
  }
  else{
    val1=init_pan;
    servo1.detach();  //better to detach continuous rotation servo when not in use
  }
  if(feed2!=0){
    val2=constrain(int(init_tilt+80.0*feed2),10,170);
  }
}

void setup(){
  //Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo2.attach(servoPin2);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(commands);
}

void setup(){
  
}

void loop(){
  nh.spinOnce();
  pantilt();
  val1=map(val1,0,ADC_Max,0,180);
  val2=map(val2,0,ADC_Max,0,180);
  servo1.write(val1);
  servo2.write(val2);
}
