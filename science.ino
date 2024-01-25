#include <ros.h>  //rosserial
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <SFE_BMP180.h>  //BMP180 sensor
#include <DHT11.h>  //DHT11 sensor
#include <Adafruit_MLX90614.h>   //infrared temperature sensor
 
Adafruit_MLX90614 mlx = Adafruit_MLX90614();  //making object for mlx

// Stepper Constants
const int STEP = 14;  //for stepper motor pin on esp
const int  steps_per_rev = 200;   //defining steps per revolution

// Pump Constants - L   //pump 1 and pump 2
const int PWMA_L = 33; //Speed control pin
const int AIN1_L = 27; //Direction  //like normal motor drivers
const int AIN2_L = 25; //Direction

// Pump Constants - C   //pump 3 and pump 4
const int PWMA_C = 15; //Speed control pin
const int AIN1_C = 4; //Direction
const int AIN2_C = 18; //Direction

// Pump Constants - R  //pump 5
const int PWMA_R = 2; //Speed control pin
const int AIN1_R = 17; //Direction
const int AIN2_R = 16; //Direction

// Halogen Lamp
const int HALO = 5;  //pin for halogen lamp

// BMP180
SFE_BMP180 temp;  //object for bmp180 for recording temperature and pressure

// DHT 11 (DATA)
DHT11 dht11(13);   //used for humidity sensing

// LM393 (AOs)
const int LM393_L = 19;  //soil humidity along with the 2 long pins thingy sensor pins
const int LM393_C = 23;
const int LM393_R = 32;

// MLX90614 (PINS)

bool flag_l=flag_c=flag_r=true;

ros::NodeHandle nh;

std_msgs::Float32 bmp180_temperature_msg;  //defining message types for publishers
std_msgs::Float32 bmp180_pressure_msg;
std_msgs::Float32 dht11_humidity_msg;
std_msgs::Float32 mlx90614_temperature_l_msg;
std_msgs::Float32 mlx90614_temperature_c_msg;
std_msgs::Float32 mlx90614_temperature_r_msg;
std_msgs::Float32 lm393_humidity_l_msg;
std_msgs::Float32 lm393_humidity_c_msg;
std_msgs::Float32 lm393_humidity_r_msg;
ros::Publisher bmp180_temperature("bmp180_temperature",&bmp180_temperature_msg);  //defining the publishers
ros::Publisher bmp180_pressure("bmp180_pressure",&bmp180_pressure_msg);
ros::Publisher dht11_humidity("dht11_humidity",&dht11_humidity_msg);
ros::Publisher mlx90614_temperature_l("mlx90614_temperature_l",&mlx90614_temperature_l_msg);  //only one mlx sensor is being used
ros::Publisher lm393_humidity_l("lm393_humidity_l",&lm393_humidity_l_msg);
ros::Publisher lm393_humidity_c("lm393_humidity_c",&lm393_humidity_c_msg);
ros::Publisher lm393_humidity_r("lm393_humidity_r",&lm393_humidity_r_msg);



void move(int motor, int speed, int direction){
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise
  boolean inPin1=LOW;
  boolean inPin2=LOW;
  if(direction==1){
    inPin1 = LOW;
    inPin2 = HIGH;
  }
  else if(direction==0){
    inPin1=HIGH;
    inPin2=LOW; 
  }
//  else{
//    boolean inPin1=LOW;
//    boolean inPin2=LOW;
//  }

  if(motor == 1){
    digitalWrite(AIN1_L, inPin1);
    digitalWrite(AIN2_L, inPin2);
    analogWrite(PWMA_L, speed);
  }
  if(motor == 2){
    digitalWrite(AIN1_C, inPin1);
    digitalWrite(AIN2_C, inPin2);
    analogWrite(PWMA_C, speed);
  }
  if(motor == 3){
    digitalWrite(AIN1_R, inPin1);
    digitalWrite(AIN2_R, inPin2);
    analogWrite(PWMA_R, speed);
  }
}


void pump_l_cb(const std_msgs::Int8& cmd_msg)
{
  if (int(cmd_msg.data) == 1)
  {
    move(1, 255, 1);  //making the motor work at full pwm
    if(flag_l){
      delay(15000);
    }
    else{
      delay(3000);
    }
    move(1, 0, 1);
    delay(50);
    flag_l=false
  }
  else if(int(cmd_msg.data)==0){
    move(1,255,0);
    delay(20000);
    move(1,0,0);
    delay(50);
  }
}

void pump_c_cb(const std_msgs::Int8& cmd_msg)
{
  if (int(cmd_msg.data) == 1)
  {
    move(2, 255, 1);
    if(flag_c){
      delay(15000);
    }
    else{
      delay(3000);
    }
    move(2, 0, 1);
    delay(50);
    flag_c=false;
  }
  else if(int(cmd_msg.data)==0){
    move(2,255,0);
    delay(20000);
    move(2,0,0);
    delay(50);
  }
}

void pump_r_cb(const std_msgs::Int8& cmd_msg)
{
  if (int(cmd_msg.data) == 1)
  {
    move(3, 255, 1);
    if(flag_r){
      delay(15000);
    }
    else{
      delay(3000);
    }
    move(3, 0, 1);
    delay(50);
    flag_r=false;
  }
  else if(int(cmd_msg.data)==0){
    move(3,255,0);
    delay(20000);
    move(3,0,0);
    delay(50);
  }
}

void halo_cb(const std_msgs::Int8& cmd_msg)
{
  if (int(cmd_msg.data) == 1)
  {
    digitalWrite(HALO, HIGH); //blinks the halogen lamp with five second interval but then cmd_msg is 1 only once most probably
    delay(5000);
    digitalWrite(HALO, LOW);
  }
}

void spectro_cb(const std_msgs::Int8& cmd_msg)  //for stepper motor
{
  int n = int(cmd_msg.data);
   if(n==1)
  {
    for(int i = 0; i<60; i++)
  {
    digitalWrite(STEP, HIGH);
    delayMicroseconds(2000);
    digitalWrite(STEP, LOW);
    delayMicroseconds(2000);
  }
  }
  if(n==2)
  {
    for(int i = 0; i<60; i++)
  {
    digitalWrite(STEP, HIGH);  //maybe the direction should be changed
    delayMicroseconds(2000);
    digitalWrite(STEP, LOW);
    delayMicroseconds(2000);
  }
  }
  if(n==3)
  {for(int i = 0; i<120; i++)
  {
    digitalWrite(STEP, HIGH);
    delayMicroseconds(2000);
    digitalWrite(STEP, LOW);
    delayMicroseconds(2000);
  }
}
}

ros::Subscriber<std_msgs::Int8> pump_l_sub("pump_l_status", pump_l_cb);  //define all the subscribers for pumps
ros::Subscriber<std_msgs::Int8> pump_c_sub("pump_c_status", pump_c_cb);
ros::Subscriber<std_msgs::Int8> pump_r_sub("pump_r_status", pump_r_cb);
ros::Subscriber<std_msgs::Int8> spectro_sub("spectro_status", spectro_cb);  //for spectrometer
ros::Subscriber<std_msgs::Int8> halo_sub("halo_status", halo_cb); //for halogen lamps

void setup(){
  pinMode(PWMA_L, OUTPUT);
  pinMode(AIN1_L, OUTPUT);
  pinMode(AIN2_L, OUTPUT);
  pinMode(PWMA_C, OUTPUT);
  pinMode(AIN1_C, OUTPUT);
  pinMode(AIN2_C, OUTPUT);
  pinMode(PWMA_R, OUTPUT);
  pinMode(AIN1_R, OUTPUT);
  pinMode(AIN2_R, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(HALO, OUTPUT);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(pump_l_sub);
  nh.subscribe(pump_c_sub);
  nh.subscribe(pump_r_sub);
  nh.subscribe(spectro_sub);
  nh.subscribe(halo_sub);
  nh.advertise(bmp180_temperature);
  nh.advertise(bmp180_pressure);
  nh.advertise(dht11_humidity);
  nh.advertise(mlx90614_temperature_l);
  nh.advertise(lm393_humidity_l);
  nh.advertise(lm393_humidity_c);
  nh.advertise(lm393_humidity_r);

  if (temp.begin()){}
  else {};
  if (mlx.begin()) {}
  else {};
}

  void loop(){

  // BMP180 temperature and pressure data
  char status;
  double T,P,p0;  //T is for storing temperature, P is for storing pressure
  status = temp.startTemperature();
  if (status != 0){
    delay(status);
    status = temp.getTemperature(T);
    if (status != 0){
      bmp180_temperature_msg.data=(float)T;
      status = temp.startPressure(3);
      if (status != 0){
        delay(status);
        status = temp.getPressure(P,T);
        if (status != 0) bmp180_pressure_msg.data=(float)P;
      }
    }
  }

  // LM393 moisture data
  int sensor_analog_l = analogRead(LM393_L);
  lm393_humidity_l_msg.data = ( 100 - ( (sensor_analog_l/4095.00) * 100 ) );
  int sensor_analog_c = analogRead(LM393_C);
  lm393_humidity_c_msg.data = ( 100 - ( (sensor_analog_c/4095.00) * 100 ) );
  int sensor_analog_r = analogRead(LM393_R);
  lm393_humidity_r_msg.data = ( 100 - ( (sensor_analog_r/4095.00) * 100 ) );

  // MLX90614 temperature data
  mlx90614_temperature_l_msg.data = float(mlx.readObjectTempC());

  // DHT11 humidity data
  dht11_humidity_msg.data = float(dht11.readHumidity());

  nh.spinOnce();
  bmp180_temperature.publish(&bmp180_temperature_msg);
  bmp180_pressure.publish(&bmp180_pressure_msg);
  dht11_humidity.publish(&dht11_humidity_msg);
  mlx90614_temperature_l.publish(&mlx90614_temperature_l_msg);
  lm393_humidity_l.publish(&lm393_humidity_l_msg);
  lm393_humidity_c.publish(&lm393_humidity_c_msg);
  lm393_humidity_r.publish(&lm393_humidity_r_msg);
  delay(1000);
}
