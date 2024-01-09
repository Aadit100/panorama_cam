#define curr_out A7

int check;

void setup() {
  Serial.begin(9600);
  pinMode(curr_out,INPUT);
}

void loop() {
  check = analogRead(curr_out);
  Serial.println(check);
}
