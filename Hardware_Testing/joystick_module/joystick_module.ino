const int VRx = A1; // analog pin for x-axis 
const int VRy = A2; // analog pin for y-axis
const int SW  = 7;  // Digital pin for the button
 
int x_value, y_value, button_state;

void setup() {
  pinMode(SW, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  x_value = analogRead(VRx);
  y_value = analogRead(VRy);
  button_state = !digitalRead(SW);
  Serial.print(x_value);
  Serial.print(" ; ");
  Serial.print(y_value);
  Serial.print(" ; ");
  Serial.println(button_state);
}
