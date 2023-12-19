#include <SFE_BMP180.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <dht11.h>
#define DHT11PIN 4
dht11 DHT11;
ros::NodeHandle nh;
std_msgs::Float32 temp_msg;
std_msgs::Float32 pressure_msg;
std_msgs::Float32 humidity_msg;
ros::Publisher temp1("temp",&temp_msg);
ros::Publisher pressure("pressure",&pressure_msg);
ros::Publisher humidity("humidity",&humidity_msg);

SFE_BMP180 temp;
void setup() {
  nh.initNode();
  nh.advertise(temp1);
  nh.advertise(pressure);
  nh.advertise(humidity);
  if (temp.begin()){}
  else while(1);
}

void loop(){
  char status;
  double T,P,p0;
  status = temp.startTemperature();
  if (status != 0){
    delay(status);
    status = temp.getTemperature(T);
    if (status != 0){
      temp_msg.data=(float)T;
      status = temp.startPressure(3);
      if (status != 0){
        delay(status);
        status = temp.getPressure(P,T);
        if (status != 0) pressure_msg.data=(float)P;
      }
    }
  }
  int chk=DHT11.read(DHT11PIN);
  humidity_msg.data=(float)DHT11.humidity;
  nh.spinOnce();
  temp1.publish(&temp_msg);
  pressure.publish(&pressure_msg);
  humidity.publish(&humidity_msg);
  delay(2000);
}
