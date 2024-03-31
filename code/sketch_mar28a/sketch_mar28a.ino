#include <util/atomic.h>

#define ENC1_1 4 // right ga25
#define ENC2_1 5 
#define ENA 6
#define EN1 9
#define EN2 10
#define ENC1_2 2 // left ga25
#define ENC2_2 3
#define EN3 8
#define EN4 7
#define ENB 11
int posi = 0; // specify posi as volatile

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //attachInterrupt(digitalPinToInterrupt(4),readEncoder,RISING);
  pinMode(4, INPUT);
  pinMode(4, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  
}
void setMotor_right(int dir, int pwmVal, int ena, int en1, int en2){
  analogWrite(ena,pwmVal);
  if(dir == 1){ // forward
    digitalWrite(en1,HIGH);
    digitalWrite(en2,LOW);
  }
  else if(dir == -1){ //reverse
    digitalWrite(en1,LOW);
    digitalWrite(en2,HIGH);
  }
  else{ // stop
    digitalWrite(en1,LOW);
    digitalWrite(en2,LOW);
  }
}
void loop() {
  // put your main code here, to run repeatedly
  float t = millis();
      setMotor_right(1,255,ENA,EN1,EN2);
      setMotor_right(1,255,ENB,EN3,EN4);
  while(posi<335){
    int b = digitalRead(4);
    int c = digitalRead(4);
   if(b != c){
    posi++;
  }
  else{
    posi--;
  }
  Serial.print(posi);
  Serial.println("");
  Serial.println(t*1000);
  }
    setMotor_right(1,0,ENA,EN1,EN2);
    setMotor_right(1,0,ENB,EN3,EN4);
}
