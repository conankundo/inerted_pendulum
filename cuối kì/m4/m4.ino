#include <util/atomic.h> 
//define chân động cơ
#define ENC1_1 4 // right ga25
#define ENC2_1 5 
#define ENA 6
#define EN1 9
#define EN2 10
//#define EN1 10
//#define EN2 9
#define ENC1_2 2 // left ga25
#define ENC2_2 3
#define EN3 8
#define EN4 7
#define ENB 11

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//khai báo biến tính vận tốc
float Ts = 200; // ms, khoảng tg tính vận tốc
float do_phan_giai = 335; // số xung sau 1 vòng quay
volatile int count = 0; // đếm số xung
float so_vong;
float tanso;
float R = 3; // Bán kính bánh xe
float Pi = 3.141592654;
float time_begin = 0;
bool dung = false;

//biến cho pwm, pid
int a=1, b=0;//biến quy định khoảng cân bằng 1 -1,2 -2,
long pe=0,p,i,d,pt=0,prePID=150 ;
long kp=50, ki=100, kd=14; //100 0 10,100 0 15,30 1 15,55 1 15,55 1 14,15k 0 10,200 20 0

//khai báo thư viện mpu và tính góc
Adafruit_MPU6050 mpu;

float preGoc=0;
static float trucX = 9.6;
static float trucYtruoc = -1;
static float trucYsau = 0.7;
static float gocBandau = 170;

volatile int posi_right = 0; 
volatile int posi_left = 0; 

void setup() {
  Serial.begin(115200);
  pinMode(ENC1_1,INPUT);
  pinMode(ENC1_2,INPUT);
  pinMode(ENC2_1,INPUT);
  pinMode(ENC2_2,INPUT);
  
  pinMode(ENA,OUTPUT);
  pinMode(EN1,OUTPUT);
  pinMode(EN2,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(EN3,OUTPUT);
  pinMode(EN4,OUTPUT);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Adafruit MPU6050 test!");
 // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  } 
  Serial.println("");
}
float tinhGoc(float giatocXHientai)
{
    float gocHientai;
    gocHientai = acos(giatocXHientai /trucX)*180/3.1416;
    Serial.println("GOC HIEN TAI");//in ra góc hiện tại theo x
    Serial.println(gocHientai);
    pt=millis();
    gocHientai=(preGoc+gocHientai)/2;
      preGoc=gocHientai;
      return preGoc;
 
}
float goc_vantoc(float x, float y){
  float gocHt = gocBandau-tinhGoc(x);
  p=gocHt;
  i += p;
  d=p-pe;
  float vt;
  if (gocHt<=80){
    vt=PID(gocHt,i,d);
  } 
  else if (gocHt>80) vt=0;
  if (vt > 255) {vt=255;ki=0;}
  else if (vt<80) {vt=0;ki=100;}
  else {vt=vt;ki=100;}
  return vt;
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
  else if(dir == -1){ //reverse
    digitalWrite(en3,LOW);
    digitalWrite(en4,HIGH);
  }
  else{ // stop
    digitalWrite(en3,LOW);
    digitalWrite(en4,LOW);
  }
}
int dir(int x){
  if (x>0) x=-1; else if (x<0) x=1;
  return x;
}
void setMotor (int dir, int pwmVal,int ena, int enb,int en1, int en2, int en3, int en4){
  if(pwmVal>130){
  setMotor_right( dir,  pwmVal, ena, en1,  en2);
  setMotor_left( dir,  pwmVal-20,  enb,  en3, en4);
  }
  else {
    setMotor_right( dir,  0, ena, en1,  en2);
    setMotor_left( dir,  0,  enb,  en3, en4);
  }
}
void readEncoder_left(){
  int x = digitalRead(ENC2_2);
  if(x > 0)
    posi_left++;
  else
    posi_left--;
}
long PID(long error, long I, long D )
{
  float delta = (millis()-pt)/1000;
  Serial.print("delta: ");
  Serial.print(delta);
  Serial.print(" / ");
  long PID_value;
    I = I + error;
    D = (error - pe)/delta;
//    if (0<error<10 ) return 255;
//    else PID_value = (kp*error) + (ki*I) + (kd*D);
    PID_value = (kp*error) + (ki*I) - (kd*D);
    pe=error;
    return PID_value;
}
    
void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  setMotor(dir(a.acceleration.y), goc_vantoc(a.acceleration.x ,a.acceleration.y),ENA, ENB, EN1,EN2,EN3,EN4 );
  
}
