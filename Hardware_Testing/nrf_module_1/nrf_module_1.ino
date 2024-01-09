#include <SPI.h>
#include <RF24.h>

RF24 radio(10,9);

const byte add_1 = 5;
const byte add_2 = 10;

char msg_2[50] = "";

void setup() {
  Serial.begin(9600);
  
  radio.begin();
  
  radio.openWritingPipe(add_1);
  
  radio.openReadingPipe(1,add_2);
  
  radio.setPALevel(RF24_PA_MIN);

}

void loop() {
  delay(250);
  radio.stopListening();
  const char msg_1[] = "msg_from_1st : 'Lunar_scout_1'";
  radio.write(&msg_1,sizeof(msg_1));

  delay(250);
  radio.startListening();
  radio.read(&msg_2,sizeof(msg_2));
  
  Serial.println(msg_1);
  Serial.println(msg_2);
}
