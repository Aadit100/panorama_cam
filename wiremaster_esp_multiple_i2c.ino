#include "Wire.h"

#define I2C_DEV_ADDR0 0x50
#define I2C_DEV_ADDR1 0x60

uint32_t i = 0;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.begin();
}

void loop() {
  delay(5000);

  //Write message to the slave
  Wire.beginTransmission(I2C_DEV_ADDR0);
  Wire.printf("Hello World! %u", i++);
  uint8_t error0 = Wire.endTransmission(true);
  Wire.beginTransmission(I2C_DEV_ADDR1);
  Wire.printf("Hello World! %u", i++);
  uint8_t error1 = Wire.endTransmission(true);
  Serial.printf("endTransmission0: %u\n", error0);
  Serial.printf("endTransmission1: %u\n", error1);
  
  //Read 16 bytes from the slave
  uint8_t bytesReceived0 = Wire.requestFrom(I2C_DEV_ADDR0, 16);
  Serial.printf("requestFrom0: %u\n", bytesReceived0);
  if((bool)bytesReceived0){ //If received more than zero bytes
    uint8_t temp0[bytesReceived0];
    Wire.readBytes(temp0, bytesReceived0);
    log_print_buf(temp0, bytesReceived0);
  }
  uint8_t bytesReceived1 = Wire.requestFrom(I2C_DEV_ADDR1, 16);
  Serial.printf("requestFrom1: %u\n", bytesReceived1);
  if((bool)bytesReceived1){ //If received more than zero bytes
    uint8_t temp1[bytesReceived1];
    Wire.readBytes(temp1, bytesReceived1);
    log_print_buf(temp1, bytesReceived1);
  }
}
