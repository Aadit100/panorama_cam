#include <stdlib.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

//The #define <pin_no> is GPIO pin_no, not ESP's overall pin_no

#define INA2 26 // Labelled as Arm Pin 2 in EasyEDA
#define INB2 27 // Arm Pin 3
#define PWM2 14 // Arm Pin 4

#define INA1 32  // Arm Pin 6
#define INB1 33  // Arm Pin 7
#define PWM1 25  // Arm Pin 8

#define INA4 4  // Arm Pin 10
#define INB4 16  // Arm Pin 11
#define PWM4 17  // Arm Pin 12

#define INA3 13  // Arm Pin 15
#define INB3 12  // Arm Pin 14  //irregularity in nos.
#define PWM3 2  // Arm Pin 16

#define INA6 21  // Arm Pin 18
#define INB6 22  // Arm Pin 19
#define PWM6 23  // Arm Pin 20

#define INA5 5  // Arm Pin 22
#define INB5 18  // Arm Pin 23
#define PWM5 19  // Arm Pin 24

#define PWMMAX 0xFFF

void callback (const std_msgs::Int32MultiArray& msg); //if you want to write the code of a function after the main code, you should declare the functions like this.
void initAndClearOutputs ();
void fixPWM ();
void updatePinBuff ();    //Buff is a general name for a array
void writeMotors ();

//We need a subscriber because we want to read the Joystick commands from the Xavier (via rosserial).
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32MultiArray> sub ("stm_write", callback); //Declares that subscriber object 'sub' reads from 'stm_write' topic

#define LOST_COMM_INTERVAL 2000
unsigned long last_comm_time = 0;
unsigned long curr_time = 0;

#define INP_LEN 6
int32_t recv_buff[INP_LEN] = {0, 0, 0, 0, 0, 0};  //Xbox = joysticks' + buttons' inputs

// PWM, EN, INA, INB (in that order)
uint8_t pin_buff[INP_LEN * 4];  //[pwm0, en0, ina0, inb0, pwm1, en1, ina1, inb1, pwm2, pwm3, pwm4, pwm5, en2, en3, en4, en5, 2*ina2, 2*ina3, 2*ina4, 2*ina5, 4*inb2, 4*inb3, 4*inb4, 4*inb5]
uint8_t *motor_buff = pin_buff;

void setup () {
 
  initAndClearOutputs (); //only called once in the beginning
 
  updatePinBuff();    //(pwm, en, ina, inb) = (0, 1, 0, 0) for initial conditions (recv_buff = [0,0,0,0,0,0])
          // pin_buff = [0,1,0,0,0,1,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,] for initial conditions
  writeMotors();
 
  curr_time = millis();   //Returns the number of milliseconds passed since the Arduino board began running the current program. This number overflows (goes back to zero) after approximately 50 days.
  last_comm_time = curr_time;   //We only have to comm (get the other 4 motor driver inputs via CAN), we don't have to write anything.
 
  nh.getHardware()->setBaud(115200);
  nh.initNode ();
  nh.subscribe (sub); //To link subscriber object sub and the node nh.
}

void loop () {

  curr_time = millis();

  nh.spinOnce ();
 
  if (curr_time - last_comm_time > LOST_COMM_INTERVAL)  //last_comm_time is not updated in the loop outside this if conditional. This conditional makes sure that a 0 is written to the motors every 200ms. Here, last_comm_time chatches up to the curr_time. Then for the next 200ms, the actual joystick inputs are written to the motors.
//When it writes 0 to the motors, the LED goes off. Then, the next time it enters the callback function, the LED becomes HIGH.
  {
    for (int i = 0; i < INP_LEN; i++)
      recv_buff[i] = 0;
    last_comm_time = curr_time;
  }

  updatePinBuff();
  writeMotors();

}



void callback (const std_msgs::Int32MultiArray& msg) {  //multi-dimensional array
  if (msg.layout.dim[0].size != INP_LEN ||
    msg.layout.dim[0].stride != INP_LEN || msg.layout.data_offset != 0)
  {
    return;
  }

  for (int i = 0; i < INP_LEN; i++)
  {
    if (msg.data[i]>190 || msg.data[i]<-190)
    {
      recv_buff[i] = msg.data[i];
    }
    else
    {
      recv_buff[i] = 0;
    }
  }
   
  last_comm_time = curr_time;
}

void updatePinBuff () {   //after initAndClearOutputs()
  for (int i = 0; i < INP_LEN; i++)
  {
    int pwm, en, ina, inb;
    en = recv_buff[i] == 0;
    ina = recv_buff[i] > 0; //forward is ina=1, inb=0. Backward is ina=0, inb=1
    inb = recv_buff[i] < 0;
    pwm = (ina ? recv_buff[i] : -recv_buff[i]); //basically pwm = abs(recv_buff[i]);

    pin_buff[4*i+0] = pwm;
    pin_buff[4*i+1] = en;
    pin_buff[4*i+2] = ina;
    pin_buff[4*i+3] = inb;
  }

}


void writeMotors () {
  // motor_buff in this order: pwm, en, ina, inb
  analogWriteResolution(12); // sets analogWrite resolution to 12 bits
 
  //digitalWrite(pin_no, HIGH/LOW) is the usual syntax, but if we use digitalWrite(pin_no, x), then x will behave as LOW if x<1 and HIGH if x>=1. (TinkerCAD)
 
  digitalWrite (PWM1, HIGH);
  digitalWrite (INA1, motor_buff[3]);
  digitalWrite (INB1, motor_buff[2]);
 
  digitalWrite (PWM2, HIGH);
  digitalWrite (INA2, motor_buff[6]);
  digitalWrite (INB2, motor_buff[7]);
 
  digitalWrite (PWM3, HIGH);
  digitalWrite (INA3, motor_buff[10]);
  digitalWrite (INB3, motor_buff[11]);
   
  digitalWrite (PWM4, HIGH);
  digitalWrite (INA4, motor_buff[14]);
  digitalWrite (INB4, motor_buff[15]);
 
  digitalWrite (PWM5, HIGH);
  digitalWrite (INA5, motor_buff[18]);
  digitalWrite (INB5, motor_buff[19]);
 
  digitalWrite (PWM6, HIGH);
  digitalWrite (INA6, motor_buff[22]);
  digitalWrite (INB6, motor_buff[23]);

}

//void fixPWM() {
  //timer.setPrescaleFactor(4);
  //timer.setOverflow(1024);
  //timer.refresh();
//}



void initAndClearOutputs () {   //first function which is called, to initialize and clear all current outputs

   //All these pins will give output to the motor drivers from the given pins on the Pico slave

  pinMode (PWM1, OUTPUT);
  pinMode (INB1, OUTPUT);
  pinMode (INA1, OUTPUT);

  pinMode (PWM2, OUTPUT);
  pinMode (INB2, OUTPUT);
  pinMode (INA2, OUTPUT);

  pinMode (PWM3, OUTPUT);
  pinMode (INB3, OUTPUT);
  pinMode (INA3, OUTPUT);

  pinMode (PWM4, OUTPUT);
  pinMode (INB4, OUTPUT);
  pinMode (INA4, OUTPUT);
 
  pinMode (PWM5, OUTPUT);
  pinMode (INB5, OUTPUT);
  pinMode (INA5, OUTPUT);

  pinMode (PWM6, OUTPUT);
  pinMode (INB6, OUTPUT);
  pinMode (INA6, OUTPUT);
 
//All inputs are active LOW
  analogWriteResolution(12); // sets analogWrite resolution to 12 bits  // 12 bits allows values to be in [0,4095] instead of the default [0,255] 8 bits  //for better accura  

  analogWrite (PWM1, PWMMAX); //PWMMAX is 0xFFF which is 4095, which is the highest possible   //PWM pin is used to control the speed of the motor, by controlling the duty cycle of the square pulse
  digitalWrite (INB1, HIGH);   //INA = 1 and INB = 0 means forward
  digitalWrite (INA1, HIGH);   //INA = 0 and INB = 1 means backward

  digitalWrite (PWM2, PWMMAX);
  digitalWrite (INB2, HIGH);
  digitalWrite (INA2, HIGH);

  analogWrite (PWM3, PWMMAX);
  digitalWrite (INB3, HIGH);
  digitalWrite (INA3, HIGH);

  digitalWrite (PWM4, HIGH);
  digitalWrite (INB4, HIGH);
  digitalWrite (INA4, HIGH);

  analogWrite (PWM5, PWMMAX);
  digitalWrite (INB5, HIGH);
  digitalWrite (INA5, HIGH);

  analogWrite (PWM6, PWMMAX);
  digitalWrite (INB6, HIGH);
  digitalWrite (INA6, HIGH);
}
