// NOTE all outputs are ACTIVE LOW

//HardwareTimer timer(3);
#include <stdlib.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

#include <mcp_can.h>
#include <SPI.h>
//#include<SD.h>

#define SPI_CS_PIN 1
MCP_CAN CAN (SPI_CS_PIN);


#define LED 25

#define PWM1 16
#define INB1 17
#define INA1 18
#define EN1  19

#define PWM2 11
#define INB2 10
#define INA2 9
#define EN2  8

#define PWMMAX 0xFFF

void callback (const std_msgs::Int32MultiArray& msg);
void initAndClearOutputs ();
void fixPWM ();
void initCAN ();
void updatePinBuff ();
void writeMotors ();
void writeCanBus ();

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32MultiArray> sub ("stm_write", callback);

// for CAN
#define STD_FRAME 0
#define ID_A 0xF1


#define WRITE_INTERVAL 20
#define LOST_COMM_INTERVAL 100
unsigned long last_write_time = 0;
unsigned long last_comm_time = 0;
unsigned long curr_time = 0;

#define INP_LEN 6
int32_t recv_buff[INP_LEN] = {0, 0, 0, 0, 0, 0};

// PWM, EN, INA, INB (in that order)
uint8_t pin_buff[INP_LEN * 4];
uint8_t *motor_buff = pin_buff;
//uint8_t *can_buff = pin_buff + 8;

void setup () {
  //fixPWM ();
  Serial.begin(500000);

  SPI.setRX(0);
    SPI.setTX(3);
    SPI.setSCK(2);
    SPI.setCS(1);
  
  initAndClearOutputs ();
  updatePinBuff();
  writeMotors();
  initCAN();
  writeCanBus();

  curr_time = millis();
  last_write_time = curr_time;
  last_comm_time = curr_time;
  nh.getHardware()->setBaud(115200);
  nh.initNode ();
  nh.subscribe (sub);
}

void loop () {
  curr_time = millis();
  nh.spinOnce ();

  if (curr_time - last_comm_time > LOST_COMM_INTERVAL)
  {
    for (int i = 0; i < INP_LEN; i++)
      recv_buff[i] = 0;
    last_comm_time = curr_time;
    digitalWrite (LED, LOW);
  }

  updatePinBuff();
  writeMotors();
  if (curr_time - last_write_time > WRITE_INTERVAL)
  {
    writeCanBus();
    last_write_time = curr_time;
    Serial.println("sending");
  }
}



void callback (const std_msgs::Int32MultiArray& msg) {
  if (msg.layout.dim[0].size != INP_LEN ||
    msg.layout.dim[0].stride != INP_LEN || msg.layout.data_offset != 0)
  {
    return;
  }

  for (int i = 0; i < INP_LEN; i++)
    recv_buff[i] = msg.data[i];

  last_comm_time = curr_time;
  digitalWrite (LED, HIGH);
}

void updatePinBuff () {
  for (int i = 0; i < 2; i++)
  {
    int pwm, en, ina, inb;
    en = recv_buff[i] == 0;
    ina = recv_buff[i] > 0;
    inb = recv_buff[i] < 0;
    pwm = (ina ? recv_buff[i] : -recv_buff[i]);

    pin_buff[4*i+0] = pwm;
    pin_buff[4*i+1] = en;
    pin_buff[4*i+2] = ina;
    pin_buff[4*i+3] = inb;
  }

  for (int i = 2; i < INP_LEN; i++)
  {
    int pwm, en, ina, inb;
    en = recv_buff[i] == 0;
    ina = recv_buff[i] > 0;
    inb = recv_buff[i] < 0;
    pwm = (ina ? recv_buff[i] : -recv_buff[i]);

    pin_buff[i+6] = pwm;
    pin_buff[i+10] = en;
    pin_buff[i+14] = ina<<1;
    pin_buff[i+18] = inb<<2;
  }

  pin_buff[12] = pin_buff[12]|pin_buff[16]|pin_buff[20];
  pin_buff[13] = pin_buff[13]|pin_buff[17]|pin_buff[21];
  pin_buff[14] = pin_buff[14]|pin_buff[18]|pin_buff[22];
  pin_buff[15] = pin_buff[15]|pin_buff[19]|pin_buff[23];

  Serial.println(pin_buff[12]);
/*for (int i = 0; i < INP_LEN; i++)
  {
    int pwm, en, ina, inb;
    en = recv_buff[i] == 0;
    ina = recv_buff[i] < 0;
    inb = recv_buff[i] > 0;
    pwm = 0xFF - (inb ? recv_buff[i] : -recv_buff[i]);

    pin_buff[4*i+0] = pwm;
    pin_buff[4*i+1] = en;
    pin_buff[4*i+2] = ina;
    pin_buff[4*i+3] = inb;
  }
  */
  
}

void writeCanBus () {
  CAN.sendMsgBuf(ID_A, STD_FRAME, 8, pin_buff+8);
}

void writeMotors () {
  // motor_buff in this order: pwm, en, ina, inb
  analogWriteResolution(12); // sets analogWrite resolution to 12 bits
  
  analogWriteResolution(12); // sets analogWrite resolution to 12 bits
  analogWrite (PWM1,    ( motor_buff[0]*0xFFF)/0xFF);
  digitalWrite (EN1,  motor_buff[1]);
  digitalWrite (INA1, motor_buff[2]);
  digitalWrite (INB1, motor_buff[3]);
  analogWrite (PWM2,   ( motor_buff[4]*0xFFF)/0xFF);
  digitalWrite (EN2,  motor_buff[5]);
  digitalWrite (INA2, motor_buff[7]);
  digitalWrite (INB2, motor_buff[6]);
}

//void fixPWM() {
  //timer.setPrescaleFactor(4);
  //timer.setOverflow(1024);
  //timer.refresh();
//}

void initCAN () {
  digitalWrite (LED, LOW);

  while (CAN_OK != CAN.begin(MCP_ANY,CAN_500KBPS,MCP_16MHZ))
  {
      digitalWrite (LED, HIGH);
      delay (1000);
      digitalWrite (LED, LOW);
      delay (1000);
  }
  CAN.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  digitalWrite (LED, HIGH);
  delay(1000);
  digitalWrite (LED, LOW);
  delay(1000);
  digitalWrite (LED, HIGH);
    
  
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
  analogWriteResolution(12); // sets analogWrite resolution to 12 bits
  
  analogWriteResolution(12); // sets analogWrite resolution to 12 bits
  

  digitalWrite (LED, HIGH);
  analogWrite(PWM1, PWMMAX); 
  digitalWrite (INB1, HIGH);
  digitalWrite (INA1, HIGH);
  digitalWrite (EN1, HIGH);
  analogWrite(PWM2, PWMMAX); 
  digitalWrite (INB2, HIGH);
  digitalWrite (INA2, HIGH);
  digitalWrite (EN2, HIGH);

 
}
