#include <Servo.h>
// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

int a, b; // unit: mm
float dist_min, dist_max, dist_raw;
int duty_target, duty_curr;
Servo myservo;

void setup() {
  myservo.attach(PIN_SERVO); 
  duty_curr = 1600;
  myservo.writeMicroseconds(duty_curr);
  while(1){}

// initialize serial port
  Serial.begin(57600);
  a = 100;
  b = 270;
}
float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(raw_dist > 255){
    myservo.writeMicroseconds(1000);
  }
  else{
    myservo.writeMicroseconds(2200);
  }
}
