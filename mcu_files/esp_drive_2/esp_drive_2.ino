/* rosserial Subscriber For Locomotion Control */
#include <ros.h>
#include <traversal/WheelRpm.h>
//#include <sensors/PanTilt.h>

//#include <Wire.h>
//#include <Servo.h>
//#include <ESP32Servo.h>

//#define I2C_DEV_ADDR 0x55

//Servo pan;
//Servo tilt;

int vel = 0,vel1=0, omega = 0, om=0, pwmr=0, pwml=0;
bool hb = false;
#define DIRr1 16
#define DIRl1 2
#define PWMr1 4
#define PWMl1 15

#define DIRr2 19
#define DIRl2 5
#define PWMr2 18
#define PWMl2 17

#define DIRr3 23
#define DIRl3 13
#define PWMr3 22
#define PWMl3 21
//int panAngle = 0;
//int tiltAngle = 0;

ros::NodeHandle nh;

void roverMotionCallback(const traversal::WheelRpm& RoverRpm)
{
  vel1 = int(RoverRpm.vel);
  om = int(RoverRpm.omega);
  hb = RoverRpm.hb;
}

//void servoCallback(const sensors::PanTilt& Control)
//{
//  panAngle = constrain(Control.pan,1,170);
//  tiltAngle = constrain(Control.tilt,5,150);
//  
//  pan.write(panAngle);
//  tilt.write(tiltAngle);
//
//}


ros::Subscriber<traversal::WheelRpm> locomotion_sub("motion", &roverMotionCallback);
//ros::Subscriber<sensors::PanTilt> pantilt_sub("pan_tilt_ctrl", &servoCallback);

void setup() {
  pinMode(PWMl1, OUTPUT);
  pinMode(PWMr1, OUTPUT);
  pinMode(DIRr1, OUTPUT);
  pinMode(DIRl1, OUTPUT);

  pinMode(PWMl2, OUTPUT);
  pinMode(PWMr2, OUTPUT);
  pinMode(DIRr2, OUTPUT);
  pinMode(DIRl2, OUTPUT);

  pinMode(PWMl3, OUTPUT);
  pinMode(PWMr3, OUTPUT);
  pinMode(DIRr3, OUTPUT);
  pinMode(DIRl3, OUTPUT);

//  digitalWrite(DIRr1, LOW);
//  digitalWrite(DIRl1, LOW);
//  digitalWrite(DIRr2, LOW);
//  digitalWrite(DIRl2, LOW);
//  digitalWrite(DIRr3, LOW);
//  digitalWrite(DIRl3, LOW);
//
//  analogWrite(PWMr1, 0);
//  analogWrite(PWMl1, 0);
//  analogWrite(PWMr2, 0);
//  analogWrite(PWMl2, 0);
//  analogWrite(PWMr3, 0);
//  analogWrite(PWMl3, 0);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(locomotion_sub);
//  nh.subscribe(pantilt_sub);
//  ESP32PWM::allocateTimer(0);
//  ESP32PWM::allocateTimer(1);
//  ESP32PWM::allocateTimer(2);
//  ESP32PWM::allocateTimer(3);
//  pan.setPeriodHertz(50);
//  tilt.setPeriodHertz(50);
//  pan.attach(14,1000,2000);
//  tilt.attach(15,1000,2000);

  //Wire.begin(32,33);
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
      digitalWrite(DIRr1, HIGH);
      analogWrite(PWMr1, abs(pwmr));

      digitalWrite(DIRr2, HIGH);
      analogWrite(PWMr2, abs(pwmr));

      digitalWrite(DIRr3, HIGH);
      analogWrite(PWMr3, abs(pwmr));
    }
    else if (pwmr < 0)
    {
      digitalWrite(DIRr1, LOW);
      analogWrite(PWMr1, abs(pwmr));

      digitalWrite(DIRr2, LOW);
      analogWrite(PWMr2, abs(pwmr));

      digitalWrite(DIRr3, LOW);
      analogWrite(PWMr3, abs(pwmr));
    }
    else
    {
      analogWrite(PWMr1, 0);
      //digitalWrite(DIRr1, LOW);
      analogWrite(PWMr2, 0);
      //digitalWrite(DIRr2, LOW);
      analogWrite(PWMr3, 0);
      //digitalWrite(DIRr3, LOW);
    }

    if (pwml > 0)
    {
      digitalWrite(DIRl1, HIGH);
      analogWrite(PWMl1, abs(pwml));

      digitalWrite(DIRl2, HIGH);
      analogWrite(PWMl2, abs(pwml));

      digitalWrite(DIRl3, HIGH);
      analogWrite(PWMl3, abs(pwml));
    }
    else if (pwml < 0)
    {
      digitalWrite(DIRl1, LOW);
      analogWrite(PWMl1, abs(pwml));

      digitalWrite(DIRl2, LOW);
      analogWrite(PWMl2, abs(pwml));

      digitalWrite(DIRl3, LOW);
      analogWrite(PWMl3, abs(pwml));
    }
    else
    {
      analogWrite(PWMl1, 0);
      //digitalWrite(DIRr1, LOW);
      analogWrite(PWMl2, 0);
      //digitalWrite(DIRr2, LOW);
      analogWrite(PWMl3, 0);
      //digitalWrite(DIRr3, LOW);
    }
  }
  else
  {
    analogWrite(PWMr1, 0);
    analogWrite(PWMr2, 0);
    analogWrite(PWMr3, 0);
//    digitalWrite(DIRr1, LOW);
//    digitalWrite(DIRr2, LOW);
//    digitalWrite(DIRr3, LOW);

    analogWrite(PWMl1, 0);
    analogWrite(PWMl2, 0);
    analogWrite(PWMl3, 0);
//    digitalWrite(DIRl1, LOW);
//    digitalWrite(DIRl2, LOW);
//    digitalWrite(DIRl3, LOW);
  }
//  Wire.beginTransmission(I2C_DEV_ADDR);
//  //Add tft code
//  
//  //End tft code
//  uint8_t error=Wire.endTransmission(true);
  // give PWM time to properly write
    delay (10);
}
