#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <MPU6050_light.h>
#include <ros.h>
#include <navigation/imu_angle.h>
#include <navigation/gps_data.h>
#define I2C_DEV_ADD_GPS 0x42
#define I2C_DEV_ADD_IMU 0x68

ros::NodeHandle nh;
navigation::gps_data gps_pos;
navigation::imu_angle angles_msg;
ros::Publisher gps("gps_coordinates",&gps_pos);
ros::Publisher angles_pub("imu_angles",&angles_msg);

SFE_UBLOX_GNSS myGNSS;  //SFE_UBLOX_GNSS uses I2C.
MPU6050 mpu(Wire);
unsigned long timer=0;

void setup(){
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(gps);
  nh.advertise(angles_pub);
  Wire.begin();

  byte status=mpu.begin();
  while(status!=0){}
  delay(100);
  mpu.calcOffsets();

  while (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    delay (100);
  }
  myGNSS.setI2COutput(COM_TYPE_UBX);  
}

void loop(){
  if(myGNSS.getPVT()==true){
    int32_t latitude=myGNSS.getLatitude();
    int32_t longitude=myGNSS.getLongitude();
    double latitude_deg=static_cast<double>(latitude)/10000000.0;
    double longitude_deg=static_cast<double>(longitude)/10000000.0;
    gps_pos.latitude=latitude_deg;
    gps_pos.longitude=longitude_deg; 
  }
  mpu.update();
  if((millis()-timer)>10){
    int32_t Pitch=mpu.getAngleX();
    int32_t Roll=mpu.getAngleY();
    int32_t Yaw=mpu.getAngleZ();
    float angle_x_Pitch=static_cast<float>(Pitch);
    float angle_y_Roll=static_cast<float>(Roll);
    float angle_z_Yaw=static_cast<float>(Yaw);
  
    angles_msg.Pitch=angle_x_Pitch;
    angles_msg.Roll=angle_y_Roll;
    angles_msg.Yaw=angle_z_Yaw;
  
    gps.publish(&gps_pos);
    angles_pub.publish(&angles_msg);
    nh.spinOnce();
    timer=millis();
  }
}
