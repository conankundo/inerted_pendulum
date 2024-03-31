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

/* Tính vận tốc xe bằng encoder
  chiều quay của động cơ: Xét thời điểm đầu của xung A & B
  số vòng = số xung / độ phân giải  
  vận tốc = số vòng / thời gian (dùng hàm milis() )
*/
float Ts = 200; // ms, khoảng tg tính vận tốc
float do_phan_giai = 374; // số xung sau 1 vòng quay
float count = 0; // đếm số xung
float so_vong;
float tanso;
float R = 3; // Bán kính bánh xe
float Pi = 3.141592654;
float time_begin = 0;
bool dung = false;
float vantoc()
{
  if(dung == false){
    time_begin = millis();
    dung = true;
  }
  if( digitalRead(5) == 1) count++;
  float thoigian = millis() - time_begin;
  if( thoigian > Ts)
  {
     so_vong = count / do_phan_giai;
     tanso =1000 * so_vong / thoigian;
     count = 0;
     dung = false;
     return tanso*R*2*Pi;
  } 
  
}
void setup() {
  pinMode(4, INPUT);
  pinMode(5, INPUT);  
  Serial.begin(115200);
}
void setMotor_right(int dir, int pwmVal, int ena, int en1, int en2){ //code động cơ chạy để tính vận tốc
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
  else if(dir == -1){ //reverse
    digitalWrite(en3,LOW);
    digitalWrite(en4,HIGH);
  }
  else{ // stop
    digitalWrite(en3,LOW);
    digitalWrite(en4,LOW);
  }
}

void loop() {
  setMotor_right(1,250,ENA,EN1,EN2);
  setMotor_left(1,0,ENB,EN3,EN4);
  Serial.println("van toc ");
  Serial.println(vantoc());

}
