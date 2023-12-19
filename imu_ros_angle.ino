/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 * 
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>
#include<ros.h>
#include <navigation/imu_angle.h>
ros::NodeHandle nh;
navigation::imu_angle angles_msg;
ros::Publisher angles_pub("imu_angles", &angles_msg);

MPU6050 mpu(Wire);
unsigned long timer = 0;

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(angles_pub);
//  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
//  Serial.print(F("MPU6050 status: "));
//  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
//  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
//  Serial.println("Done!\n");
}

void loop() {
  mpu.update();
  
  if((millis()-timer)>10){ // print data every 10ms
    int32_t Pitch=mpu.getAngleX();
//  Serial.print("X : ");
//  Serial.print(mpu.getAngleX());
    int32_t Roll=mpu.getAngleY();
//  Serial.print("\tY : ");
//  Serial.print(mpu.getAngleY());
    int32_t Yaw=mpu.getAngleZ();
//  Serial.print("\tZ : ");
//  Serial.println(mpu.getAngleZ());
    
    float angle_x_Pitch=static_cast<float>(Pitch);
    float angle_y_Roll=static_cast<float>(Roll);
    float angle_z_Yaw= static_cast<float>(Yaw);

    angles_msg.Pitch=angle_x_Pitch;
    angles_msg.Roll=angle_y_Roll;
    angles_msg.Yaw=angle_z_Yaw;
    angles_pub.publish(&angles_msg);
    nh.spinOnce();
    timer = millis();

  }
}
