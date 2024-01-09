#include <SPI.h>
#include <RF24.h>

RF24 radio(8, 9);
const byte address[6] = "00001";

byte Array[4];

void setup() {
 radio.begin();
 radio.openWritingPipe(address);
 radio.setPALevel(RF24_PA_MIN);
 radio.stopListening();
 Serial.begin(9600);
}

void loop() {
 Array[0] = analogRead(A0);  
 Array[1] = analogRead(A1);
 Array[2] = analogRead(A2);  
 Array[3] = analogRead(A3);

 radio.write(&Array, sizeof(Array));
