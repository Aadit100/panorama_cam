
#define enB 5
#define in3 9
#define in4 4

void bo_motor_init(){
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void forward(){
  analogWrite(enB, 100);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);  
}

void backward(){
  analogWrite(enB, 100);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);  
}

void setup() {
  bo_motor_init();
}

void loop() {
  forward();
  delay(3000);
  backward();
  delay(3000);    
}
