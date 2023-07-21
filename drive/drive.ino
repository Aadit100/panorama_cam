/* rosserial Subscriber For Locomotion Control */
#include <ros.h>
#include <traversal/WheelRpm.h>
#include <sensors/PanTilt.h>
#include <std_msgs/Int8.h>

#include <Wire.h>
#include <Servo.h>

#define b1 15
#define b2 16
#define b3 17
#define b4 20

Servo pan;
Servo tilt;
Servo myservo;

int vel = 0, omega = 0;
bool hb = false;
int panAngle = 0;
int tiltAngle = 0;
int pan_pos = 10, tilt_pos = 10;
int flag=0;
int flag_stop_joy=0;
const int PIN_RED   = 2;
const int PIN_GREEN = 3;
const int PIN_BLUE  = 4;
int max_it=25;
ros::NodeHandle nh;

void loco(int address)
{
  Wire.beginTransmission(address);
  Wire.write(byte(vel));
  
  //Wire.write(byte(vel>>8)); byte shifting disabled for old stm new stm problem
  Wire.write(byte(omega));
  //Wire.write(byte(omega>>8));
  Wire.write(byte(hb));
  Wire.endTransmission();
}

void pictures(){
  for(int i=1; i<=max_it; i++){
    nh.spinOnce(); //for rosserial to not loose link of the other topics ( removing it might work but i am not sure yet)
    delay(1000);
    myservo.attach(5);
    myservo.write(84);
    delay(500);
    myservo.detach();
    delay(1000);
  }
  myservo.attach(5);
  myservo.write(100);
  delay(max_it*475);
  myservo.detach();
}

void roverMotionCallback(const traversal::WheelRpm& RoverRpm)
{
  vel = int(RoverRpm.vel);
  omega = int(RoverRpm.omega);
  hb = RoverRpm.hb;

  loco(b1);
  loco(b2);
  loco(b3);
  loco(b4);
}

void servoCallback(const sensors::PanTilt& Control)
{
  panAngle = constrain(Control.pan,1,170);
  tiltAngle = constrain(Control.tilt,5,150);
  
  pan.write(panAngle);
  tilt.write(tiltAngle);

}


void ledCallback(const std_msgs::Int8& Mode)
{ 
  if (Mode.data==0)
  {
      analogWrite(PIN_RED,   255);
      analogWrite(PIN_GREEN, 255);
      analogWrite(PIN_BLUE,  0);
  }

  else if(Mode.data==1)
  {
      analogWrite(PIN_RED, 0);
      analogWrite(PIN_GREEN, 255);
      analogWrite(PIN_BLUE,  255);
  }

  else
  {
      analogWrite(PIN_RED, 255);
      analogWrite(PIN_GREEN, 0);
      analogWrite(PIN_BLUE,  255);
      delay(500);
      analogWrite(PIN_RED, 255);
      analogWrite(PIN_GREEN, 255);
      analogWrite(PIN_BLUE,  255);
      delay(500);
  }
}

void rotateCallback(const std_msgs::Int8& check_msg){
  if(not flag_stop_joy){
    if(check_msg.data==-1) 
      {
        myservo.write(97);
        delay(100);
      }
    else if(check_msg.data==1)
      {
        myservo.write(80);
        delay(100);
      }
  
    else
       {
        myservo.write(90);
        delay(100);
       }
  }
}

void callback(const std_msgs::Int8& check_msg){
  if(check_msg.data==1){
    flag=1;
    flag_stop_joy=1;
  }
}



ros::Subscriber<traversal::WheelRpm> locomotion_sub("motion", &roverMotionCallback);
ros::Subscriber<sensors::PanTilt> pantilt_sub("pan_tilt_ctrl", &servoCallback);
ros::Subscriber<std_msgs::Int8>led_sub("led",&ledCallback);
ros::Subscriber<std_msgs::Int8>servo_sub("servo",&rotateCallback);
ros::Subscriber<std_msgs::Int8> sub("check_node",&callback);

void setup() {
  nh.initNode();
  nh.subscribe(locomotion_sub);
  nh.subscribe(pantilt_sub);
  nh.subscribe(led_sub);
  nh.subscribe(sub);
  nh.subscribe(servo_sub);
  Wire.begin();
  pan.attach(10);
  tilt.attach(11);
  myservo.attach(5);
  pinMode(PIN_RED,   OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE,  OUTPUT);

}

void loop() {
  nh.spinOnce();
  if(flag==1){
    pictures();
    flag=0;
    flag_stop_joy=0;
  }

}
