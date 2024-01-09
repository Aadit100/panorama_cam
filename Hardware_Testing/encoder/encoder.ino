#include <Encoder.h>

#define enA 6  
#define in1 A2
#define in2 A3

Encoder DC_Encoder(2, 3);

long old_ticks = -999;

void encoder(){
  long ticks = DC_Encoder.read();
  if (ticks != old_ticks) {
    old_ticks = ticks;
  }
  Serial.println(ticks);
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  encoder();
}
