// NOTE all outputs are ACTIVE LOW

//HardwareTimer timer(3);

#include <stdlib.h>

#include <mcp_can.h>
#include <SPI.h>

#define SPI_CS_PIN 17  //the pico has cs(chip select) pin as 17
MCP_CAN CAN (SPI_CS_PIN);


#define LED 25  //onboard led

#define PWM1 15  //these are the pins for the dual channel motor drivers for the 4 motors pico controls
#define INB1 14
#define INA1 13
#define EN1  12

#define PWM2 11
#define INB2 10
#define INA2 9
#define EN2  8

#define PWM3 7
#define INB3 6
#define INA3 5
#define EN3  4

#define PWM4 3
#define INB4 2
#define INA4 1
#define EN4  0
//#define CAN_INT 22


#define PWMMAX 0xFFF  //max precision is 4095

void initAndClearOutputs ();  //similar to arm master to initialize the pins
void fixPWM ();
void initCAN ();
void updatePinBuff ();  //updates the buffer
void writeMotors ();

// for CAN
#define STD_FRAME 0
unsigned long SELF_ID = 0xF1;

#define LOST_COMM_INTERVAL 2000
unsigned long last_comm_time = 0;   //similar to arm master
unsigned long curr_time = 0;

// PWM, EN, INA, INB (in that order)
uint8_t buff[16] = {0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0};  //we need 4 inputs for 4 motors so 16 values in total

void setup () {
  //fixPWM ();
  
  writeMotors();
  initAndClearOutputs ();
  initCAN();

  curr_time = millis();
  last_comm_time = curr_time; //updating last comm time
}

void loop () {
  //Serial.println("received");  //to see if message received
  curr_time = millis();
  if (CAN_MSGAVAIL == CAN.checkReceive())
  //if(!digitalRead(CAN_INT))
  {
    Serial.println("received");  //to see if message received
    unsigned char len; 
    unsigned char tmp[8];
    CAN.readMsgBuf (&SELF_ID,&len, tmp); //gives the length of the message along with the buffer
    if (CAN.checkReceive())//CAN.getCanId() == SELF_ID)  //checking if the id on the message is same as the id of the pico
    {
      for (int i = 0; i < 4; i++)
        buff[4*i] = tmp[i];  //the pwm values are stored at every 4 intervals for each motor like stored in the 2 motors of master

       for (int i=4;i<8;i++)
       {
        buff[4*(i-4)+1] = tmp[i]&1;  //converting this back to pwms, ina and inb. genius!!!
        buff[4*(i-4)+2] = (tmp[i]>>1)&1;
        buff[4*(i-4)+3] = (tmp[i]>>2)&1;
        
       }
      last_comm_time = curr_time;
      digitalWrite (LED, HIGH);
    }
    last_comm_time = curr_time; //updating last comm time
  }
  
  if (curr_time - last_comm_time > LOST_COMM_INTERVAL)
  {
    for (int i = 0; i < 16; i++)
      buff[i] = 0;  //if no communication has occured then stop the motors
    last_comm_time = curr_time;
    digitalWrite (LED, LOW);
  }

  writeMotors(); //write the values to motors
}




void writeMotors () {

  analogWriteResolution(12); //with precision from 0 to 4095
  



  
  // motor_buff in this order: pwm, en, ina, inb
  analogWrite (PWM1,    (buff[0]*0xFFF)/0xFF);  //just writing the values you get in buffer to motor drivers
  digitalWrite (EN1,  buff[1]);
  digitalWrite (INA1, buff[3]);
  digitalWrite (INB1, buff[2]);
  analogWrite (PWM2,    (buff[4]*0xFFF)/0xFF);
  digitalWrite (EN2,  buff[5]);
  digitalWrite (INA2, buff[6]);
  digitalWrite (INB2, buff[7]);
  analogWrite (PWM3,    (buff[8]*0xFFF)/0xFF);
  digitalWrite (EN3,  buff[9]);
  digitalWrite (INA3, buff[10]);
  digitalWrite (INB3, buff[11]);
  digitalWrite (PWM4, buff[13]);
  analogWrite (EN4,    (buff[12]*0xFFD)/0xFF); //this may be switched between en and pwm
  digitalWrite (INA4, buff[15]);
  digitalWrite (INB4, buff[14]);
}

/*void fixPWM() {
  //timer.setPrescaleFactor(4);
  //timer.setOverflow(1024);
  //timer.refresh();
}*/

void initCAN () {   //to initially check if can communication is established
  Serial.begin(500000);
  digitalWrite (LED, LOW);

  while (CAN_OK != CAN.begin(MCP_ANY,CAN_500KBPS,MCP_16MHZ))
  {
      digitalWrite (LED, HIGH);
      delay (500);
      digitalWrite (LED, LOW);
      delay (500);
  }
  CAN.setMode(MCP_NORMAL);
  digitalWrite (LED, HIGH);
  delay(1000);
  digitalWrite (LED, LOW);
  delay(1000);
  digitalWrite (LED, HIGH);
  delay(1000);
  Serial.println("CAN Working");  //verification is led blinking and message receiving
  delay(500);
}

void initAndClearOutputs () {
  pinMode (LED, OUTPUT);
  pinMode (PWM1, OUTPUT);
  pinMode (INB1, OUTPUT);
  pinMode (INA1, OUTPUT);
  pinMode (EN1, OUTPUT);
  pinMode (PWM2, OUTPUT);
  pinMode (INB2, OUTPUT);
  pinMode (INA2, OUTPUT);
  pinMode (EN2, OUTPUT);
  pinMode (PWM3, OUTPUT);
  pinMode (INB3, OUTPUT);
  pinMode (INA3, OUTPUT);
  pinMode (EN3, OUTPUT);
  pinMode (PWM4, OUTPUT);
  pinMode (INB4, OUTPUT);
  pinMode (INA4, OUTPUT);
  pinMode (EN4, OUTPUT);

  analogWriteResolution(12);
  

  digitalWrite (LED, HIGH);
  analogWrite (PWM1, PWMMAX);
  digitalWrite (INB1, HIGH);
  digitalWrite (INA1, HIGH);
  digitalWrite (EN1, HIGH);
  digitalWrite (PWM2, PWMMAX);
  digitalWrite (INB2, HIGH);
  digitalWrite (INA2, HIGH);
  digitalWrite (EN2, HIGH);
  analogWrite (PWM3, PWMMAX);
  digitalWrite (INB3, HIGH);
  digitalWrite (INA3, HIGH);
  digitalWrite (EN3, HIGH);
  digitalWrite (PWM4, HIGH);
  digitalWrite (INB4, HIGH);
  digitalWrite (INA4, HIGH);
  analogWrite (EN4, PWMMAX);

  //pinMode(CAN_INT, INPUT);
}
