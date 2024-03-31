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
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

volatile int posi_right = 0; 
volatile int posi_left = 0; 

void setup() {
  Serial.begin(9600);
  pinMode(ENC1_1,INPUT);
  pinMode(ENC1_2,INPUT);
  pinMode(ENC2_1,INPUT);
  pinMode(ENC2_2,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1_1),readEncoder_right,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_2),readEncoder_left,RISING);

  pinMode(ENA,OUTPUT);
  pinMode(EN1,OUTPUT);
  pinMode(EN2,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(EN3,OUTPUT);
  pinMode(EN4,OUTPUT);
}

void loop() {
  
    setMotor_right(1, 255, ENB, EN3, EN4);
    setMotor_left(1, 255, ENA, EN1, EN2);
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
void setMotor_left(int dir, int pwmVal, int enb, int en3, int en4){
  analogWrite(enb,pwmVal);
  if(dir == 1){ // forward
    digitalWrite(en3,HIGH);
    digitalWrite(en4,LOW);
  }
  else if(dir == -1){ // reverse
    digitalWrite(en3,LOW);
    digitalWrite(en4,HIGH);
  }
  else{ // stop
    digitalWrite(en3,LOW);
    digitalWrite(en4,LOW);
  }
}

void readEncoder_right(){
  int x = digitalRead(ENC1_1);
  if(x > 0)
    posi_right++;
  else
    posi_right--;
}
void readEncoder_left(){
  int x = digitalRead(ENC2_2);
  if(x > 0)
    posi_left++;
  else
    posi_left--;
}
