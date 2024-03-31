void setup() {
  // put your setup code here, to run once:


Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  float a;
  float l = 0.5, g = 9.8, e = 2.73, t=0.5;
  a = 3/(2*l)+3*g*(2*l/(3*g))^(1/2)*(e^((3*g/(2*l))^(1/2)*t)-e^(-(3*g/(2*l))^(1/2)*t))/(4*l);
  Serial.println(a);
}
