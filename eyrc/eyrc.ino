#include <Wire.h>
#include <MPU6050_light.h>
#include <SPI.h>
#include <RF24.h>

#define encA 2
#define encB 3
#define en_omni 6
#define en_drive 5
#define inA_omni A2
#define inB_omni A3
#define inA_drive 9
#define inB_drive 4

#define LEDR 7
#define LEDG A0

#define buzz A1
#define acs A6
#define hall A7

MPU6050 mpu(Wire);
RF24 radio(10,8);

//Radio variables
const byte add_1 = 5;
const byte add_2 = 10;
byte Array[3];

//Encoder variables
volatile long enc_pos=0;
volatile bool A_set=false;
volatile bool B_set=false;
float calib_const=4800.0;

//Timing and State variables
float curr_time=0.0;
float prev_time=0.0;       //general time tracking
float last_comm_time=0.0;  //for the connection between joystick and onboard arduino
float last_beep_time=0.0;  //for buzzer
float last_glow_time=0.0;  //for leds
int button=0;
int buzz_state=0;  //0:off and 1:on
int buzz_count=0;
int magnet_state=0; //0:no magnet  1:North polarity  2:South polarity   also can be the LED state
int stage=0; //tells which stage of the mission we are in   0:before starting    1:starting buzz    2:starting to move

//Control variables
float wheel_angle=0;
float prev_wheel_angle=0;
float wheel_yaw=0;
float wheel_angle_vel=0;
float yaw_angle=0;
float yaw_vel=0;
float roll_angle=0;
float prev_roll_angle=0;
float prev_imu_yaw=0;
float roll_vel=0;
float drive_target=0;
float target_yaw=0.0;
int K[4]={1,1,1,1};  //may not use since modelling this mathematically is going to be f***ed up
float factor=1;

//Bike dimension variables
float R_wheel=;  //the radius of the omni wheel
float L=;   //length from the drive wheel (pivot) to the omni wheel

//Sensor threshold variables
float init_acs_val=0.0;
float acs_val=0.0;
float acs_thresh=;
float init_hall_val=0.0;
float hall_val=0.0;
float hall_thresh=;


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

void calc_wheel_yaw(){
  wheel_angle=enc_pos*(360.0/calib_const);
  wheel_angle_vel=(wheel_angle-wheel_angle_prev)*1000.0/(curr_time-prev_time);
  wheel_angle_prev=wheel_angle;
  wheel_yaw=(float(wheel_angle)*R_wheel)/L;

  if(int(wheel_yaw/360)>=1){
    wheel_yaw-=360*int(wheel_yaw/360);
  }
  else if(angle<0){
    wheel_yaw+=360*(1+int(wheel_yaw/360));
  }
}

void wheel_control(){
  float error=target_yaw-yaw_angle;
  if(error<-180){
    error+=360;
  }
  if(error>180){
    error-=360;
  }
  ctrl_inp=-K[0]*(error)-K[1]*(0-yaw_vel)-K[2]*(0-roll_angle)-K[3]*(0-roll_vel);
  if(ctrl_inp>=0){
    digitalWrite(inA_omni,1);  //We need to make sure this is the correct direction
    digitalWrite(inB_omni,0);
  }
  else{
    digitalWrite(inA_omni,0);
    digitalWrite(inB_omni,1);
  }
  analogWrite(en_omni,abs(ctrl_inp*factor));

  if(drive_target>=0){
    digitalWrite(inA_drive,1);
    digitalWrite(inB_drive,0);
  }
  else{
    digitalWrite(inA_drive,0);
    digitalWrite(inB_dirve,1);
  }
  analogWrite(en_drive,abs(drive_target));
}

void get_target(){
  if(radio.available()){
    radio.read(&Array,sizeof(Array));
    target_yaw+=((float(Array[0])-511.5)*10.0)/1023.0;  //since middle value is 512 so to adjust for that also this is wrong the 255 part will come afterwards
    if (target_yaw>180){
      target_yaw-=360;
    }  
    else if (target_yaw<-180){
      target_yaw+=360;
    }
    drive_target=((float(Array[1])-511.5)*255)/1023.0;  //this is voltage unlike the above target that is yaw angle
    if(Array[2]){
      button=float(Array[2]);
    }
    last_comm_time=curr_time;
  }
  if(curr_time-last_comm_time>=100){  //setting the interval for lost communication
    target_yaw=drive_target=button=0;
  }
}


void setup() {  //remember to initialize the pins
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  pinMode(en_omni, OUTPUT);
  pinMode(en_drive, OUTPUT);
  pinMode(inA_omni, OUTPUT);
  pinMode(inB_omni, OUTPUT);
  pinMode(inA_drive, OUTPUT);
  pinMode(inB_drive, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(buzz, OUTPUT);
  pinMode(hall, INPUT);
  pinMode(acs, INPUT);

  analogWrite(en_omni,0);
  analogWrite(en_drive,0);
  digitalWrite(inA_omni,0);
  digitalWrite(inB_omni,0);
  digitalWrite(inA_drive,0);
  digitalWrite(inB_drive,0);
  digitalWrite(LEDR,0);
  digitalWrite(LEDG,0);  //I don't think buzzer pin needs to be initialized
  
  attachInterrupt(digitalPinToInterrupt(encA),callback,RISING);

  Wire.begin();
  byte status=mpu.begin();
  while(status!=0){}
  mpu.calcOffsets();

  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setPayloadSize(sizeof(Array));
  radio.openReadingPipe(1,add_1);
  radio.startListening();
}

void loop() {
  // See where previous time is to be updated to current time
  curr_time=millis();
  
  if(curr_time>=5000){  //play the buzzer for 1s at the start of the play
    if(stage==1){
      if(not buzz_state){
        tone(buzz,500);
        buzz_state=1;
      }
      else{
        if(curr_time-last_beep_time>=1000){
          noTone(buzz);
          stage=2;
          buzz_state=0;
        }
      }
    }
    else if(stage==0){
      last_beep_time=curr_time;
      stage=1;
      init_acs_val=analogRead(acs);
      init_hall_val=analogRead(hall);
    }
  }
  
  if(curr_time-prev_time>=50){
    calc_wheel_yaw();
    
    mpu.update();
    
    roll_angle=mpu.getAngleX();
    roll_vel=(roll_angle-prev_roll_angle)*1000.0/(curr_time-prev_time);
    prev_roll_angle=roll_angle;
    
    imu_yaw=mpu.getAngleZ();  //We need to see how the yaw changes
    imu_yaw_vel=(imu_yaw-prev_imu_yaw)*1000.0/(curr_time-prev_time);
    prev_imu_yaw=imu_yaw; 
    yaw_angle=0.7*imu_yaw+0.3*wheel_yaw;  //this has filter for imu_yaw and wheel_yaw
    yaw_vel=0.7*imu_yaw_vel+0.3*(wheel_angle_vel*R_wheel)/L;

    get_target();

    wheel_control();

    if(button){
      if(magnet_state){
        if(not curr_time-last_glow_time<=4000 or buzz_count==2){
          buzz_count=0;
          button=0;
          magnet_state=0;
          digitalWrite(LEDR,0);
          digitalWrite(LEDG,0);
        }
        else{
          if(buzz_state==0 and (curr_time-last_beep_time)>=1000){
            tone(buzz,500);
            buzz_state=1;
            buzz_count++;
            last_beep_time=curr_time;
          }
          else{
            if((curr_time-last_beep_time)>=1000){
              noTone(buzz);
              buzz_state=0;
              last_beep_time=curr_time;
            }
          }
          if((curr_time-last_glow_time)>=1000){
            if(magnet_state==1){
              digitalWrite(LEDR,1);
              digitalWrite(LEDG,0);
            }
            else if(magnet_state==2){
              digitalWrite(LEDR,0);
              digitalWrite(LEDG,1);
            }
          }
        }
      }
      else{
        acs_val=analogRead(acs);
        hall_val=analogRead(hall);
        if(abs(acs_val-init_acs_val)>=acs_thresh and abs(hall_val-init_hall_val)>=hall_thresh){
          if(acs_val>init_acs_val){
            magnet_state=1;
          }
          else{
            magnet_state=2;
          }
          last_beep_time=last_glow_time=curr_time;
        }
        else{
          button=0;
          magnet_state=0;
        }
      }
    }
    prev_time=curr_time;
  }
}
