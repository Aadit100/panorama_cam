const int encA=2;
const int encB=3;

volatile long enc_pos=0;
volatile bool A_set=false;
volatile bool B_set=false;

void callback(){
  A_set=digitalRead(encA);
  B_set=digitalRead(encB);
  if(A_set==B_set)
  {
    enc_pos++;
  }
  else
  {
    enc_pos--;
  }
}
void setup() {
  pinMode(encA,INPUT_PULLUP);
  pinMode(encB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encA),callback,RISING);

  Serial.begin(57600);

}

void loop() {
  Serial.println(enc_pos);
  delay(10);
  

}
