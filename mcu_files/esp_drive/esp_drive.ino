/* rosserial Subscriber For Locomotion Control */
#include <ros.h>
#include <traversal/WheelRpm.h>
#include <sensors/PanTilt.h>

#include <Wire.h>
//#include <Servo.h>

//#define b1 15
//#define b2 16
//#define b3 17
//#define b4 20

//Servo pan;
//Servo tilt;

int vel = 0,vel1=0, omega = 0, om=0, pwmr=0, pwml=0;
bool hb = false;
//#define CSr1 
//#define CSl1 
//#define slpr1
//#define slpl1
#define DIRr1 16
#define DIRl1 7
#define PWMr1 15
#define PWMl1 6

//#define CSr2
//#define CSl2
//#define slpr2
//#define slpl2
#define DIRr2 46
#define DIRl2 8
#define PWMr2 3
#define PWMl2 18

//#define CSr3
//#define CSl3
//#define slpr3
//#define slpl3
#define DIRr3 13
#define DIRl3 11
#define PWMr3 12
#define PWMl3 10
//int panAngle = 0;
//int tiltAngle = 0;
//int pan_pos = 10, tilt_pos = 10;
//int servo_then; //not used anywhere else

ros::NodeHandle nh;
/*
void loco(int address)
{
  Wire.beginTransmission(address);
  Wire.write(byte(vel));
  Wire.write(byte(omega));
  Wire.write(byte(hb));
  Wire.endTransmission();
}*/

void roverMotionCallback(const traversal::WheelRpm& RoverRpm)
{
  vel1 = int(RoverRpm.vel);
  om = int(RoverRpm.omega);
  hb = RoverRpm.hb;
/*
  loco(b1);
  loco(b2);
  loco(b3);
  loco(b4);
  */
}
/*
void servoCallback(const sensors::PanTilt& Control)
{
  //panAngle = constrain(Control.pan,1,170);
  tiltAngle = constrain(Control.tilt,5,150);
  
  pan.write(panAngle);
  tilt.write(tiltAngle);

}
*/

ros::Subscriber<traversal::WheelRpm> locomotion_sub("motion", &roverMotionCallback);
//ros::Subscriber<sensors::PanTilt> pantilt_sub("pan_tilt_ctrl", &servoCallback);

void setup() {
  pinMode(PWMl1, OUTPUT);
  pinMode(PWMr1, OUTPUT);
//  pinMode(slpl1, OUTPUT);
//  pinMode(slpr1, OUTPUT);
  pinMode(DIRr1, OUTPUT);
  pinMode(DIRl1, OUTPUT);
//  pinMode(CSr1, INPUT);
//  pinMode(CSl1, INPUT);

  pinMode(PWMl2, OUTPUT);
  pinMode(PWMr2, OUTPUT);
//  pinMode(slpl2, OUTPUT);
//  pinMode(slpr2, OUTPUT);
  pinMode(DIRr2, OUTPUT);
  pinMode(DIRl2, OUTPUT);
//  pinMode(CSr2, INPUT);
//  pinMode(CSl2, INPUT);

  pinMode(PWMl3, OUTPUT);
  pinMode(PWMr3, OUTPUT);
//  pinMode(slpl3, OUTPUT);
//  pinMode(slpr3, OUTPUT);
  pinMode(DIRr3, OUTPUT);
  pinMode(DIRl3, OUTPUT);
//  pinMode(CSr3, INPUT);
//  pinMode(CSl3, INPUT);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(locomotion_sub);
  //nh.subscribe(pantilt_sub);
  //Wire.begin();
  //pan.attach(10);
  //tilt.attach(11);
}

void loop() {
  nh.spinOnce();
  vel=vel1-127;
  omega=om-127;
  pwmr = int(vel+omega);
  pwml = int(vel-omega); 

  if (hb==0)
  {
    if (pwmr > 0)
    {
      //digitalWrite(slpr1, HIGH);
      digitalWrite(DIRr1, HIGH);
      analogWrite(PWMr1, abs(pwmr));

      digitalWrite(DIRr2, HIGH);
      analogWrite(PWMr2, abs(pwmr));

      digitalWrite(DIRr3, HIGH);
      analogWrite(PWMr3, abs(pwmr));
    }
    else if (pwmr < 0)
    {
      //digitalWrite(slpr, HIGH);
      digitalWrite(DIRr1, LOW);
      analogWrite(PWMr1, abs(pwmr));

      digitalWrite(DIRr2, LOW);
      analogWrite(PWMr2, abs(pwmr));

      digitalWrite(DIRr3, LOW);
      analogWrite(PWMr3, abs(pwmr));
    }
    else
    {
      //digitalWrite(slpr, LOW);
      analogWrite(PWMr1, 0);

      analogWrite(PWMr2, 0);

      analogWrite(PWMr3, 0);
    }

    if (pwml > 0)
    {
      //digitalWrite(slpl, HIGH);
      digitalWrite(DIRl1, LOW);
      analogWrite(PWMl1, abs(pwml));

      digitalWrite(DIRl2, LOW);
      analogWrite(PWMl2, abs(pwml));

      digitalWrite(DIRl3, LOW);
      analogWrite(PWMl3, abs(pwml));
    }
    else if (pwml < 0)
    {
      //digitalWrite(slpl, HIGH);
      digitalWrite(DIRl1, HIGH);
      analogWrite(PWMl1, abs(pwml));

      digitalWrite(DIRl2, HIGH);
      analogWrite(PWMl2, abs(pwml));

      digitalWrite(DIRl3, HIGH);
      analogWrite(PWMl3, abs(pwml));
    }
    else
    {
      //digitalWrite(slpl, LOW);
      analogWrite(PWMl1, 0);

      analogWrite(PWMl2, 0);

      analogWrite(PWMl3, 0);
    }
  }
//  else
//  {
//    //digitalWrite(slpr, HIGH);
//    analogWrite(PWMr, 0);
//    //digitalWrite(slpl, HIGH);
//    analogWrite(PWMl, 0);
//  }

  // give PWM time to properly write
    delay (10);
}
