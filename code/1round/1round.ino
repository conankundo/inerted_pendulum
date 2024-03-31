// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <util/atomic.h>

#define ENA 6
#define ENB 11
#define EN1 9
#define EN2 10 //2 động cơ ngược nên phải khai báo ngược
#define EN3 8
#define EN4 7
#define C1R 4
#define C2R 5 
#define C1L 2
#define C2L 3

volatile int posi = 0; // specify posi as volatile

//Servo myservo;

Adafruit_MPU6050 mpu;

static float trucX = 9.6;
static float trucYtruoc = -1;
static float trucYsau = 0.7;
static float gocBandau = 170;


bool sauHaytruoc(float giatocYHientai)
{
  if (giatocYHientai > trucYsau) return 0;
  if (giatocYHientai < trucYtruoc) return 1;
}
float tinhGoc(float giatocXHientai)
{
  float gocHientai;
  gocHientai = acos(giatocXHientai / trucX) * 180 / 3.14;
  Serial.println(gocHientai);
  return gocHientai;
}

void setup(void) {
  Serial.begin(115200);
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
  delay(100);

  attachInterrupt(digitalPinToInterrupt(C1R),readEncoder,RISING);
  pinMode(C1R, INPUT);
  pinMode(C2R, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
}
void readEncoder(){
  int b = digitalRead(C1R);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
void loop() {

  /* Get new sensor events with the readings */
  Serial.println(digitalRead(C1R));
}
