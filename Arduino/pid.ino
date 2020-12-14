#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9                            // [1234] LED를 아두이노의 GPIO 9번 핀에 연결
#define PIN_SERVO 10    //[3152] 서보모터를 아두이노의 10번 핀에 연결
#define PIN_IR A0     //[3158] IR센서를 아두이노의 A0 핀에 연결

// Framework setting
#define _DIST_TARGET 255  //[3166]목표로 하는 탁구공 중심 위치까지 거리255mm로 고정
#define _DIST_MIN 10                       //[3164] 최소 측정 거리 10mm로 고정 
#define _DIST_MAX 410   // [3401] 측정 거리의 최댓값를 410mm로 설정

// Distance sensor
//#define _INTERVAL_DIST 30  // DELAY_MICROS * samples_num^2 의 값이 최종 거리측정 인터벌임. 넉넉하게 30ms 잡음.
#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)
#define EMA_ALPHA 0.35
#define _DIST_ALPHA 0.35  // [3162] ema 필터의 alpha 값을 0.0으로 설정

// Servo range
#define _DUTY_MIN 1300    //[3148]  서보의 가동 최소 각도(0
#define _DUTY_NEU 1600      //[3150] servo neutral position (90 degree)
#define _DUTY_MAX 1900                // [3169] 서보의 최대 가동 각도(180º)

// Servo speed control
#define _SERVO_ANGLE 30   //[3159] 서보의 각도(30º) 
//[3150] 레일플레이트가 사용자가 원하는 가동범위를 움직일때, 이를 움직이게 하는 서보모터의 가동범위
#define _SERVO_SPEED 1250             //[3147]  서보 속도를 30으로 설정
#define INTERVAL 20

// Event periods
#define _INTERVAL_DIST 30   // [3153] Distance Sensing을 20(ms) 마다 실행한다.
#define _INTERVAL_SERVO 20 // [3401] 서보를 20ms마다 조작하기
#define _INTERVAL_SERIAL 100  // [3151] 시리얼 0.1초 마다 업데이트

// PID parameters
#define _KP 2.2       // [3158] 비례상수 설정
#define _KD 19.0 * 1.2     // [3159] 미분제어 상수
#define _KI 0.02 * 1.15    //[3157] 적분제어 상수

float ema_dist=0;            // EMA 필터에 사용할 변수
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 3;     // 스파이크 제거를 위한 부분필터에 샘플을 몇개 측정할 것인지. 3개로 충분함! 가능하면 수정하지 말 것.

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;  // [3153] Servo를 제어할 Object를 생성해 준다
// Distance sensor
float dist_target; // location to send the ball 
float dist_raw, dist_ema;    //[3160] 실제 거리측정값과 ema필터를 적용한 값을 저장할 변수

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; // [3162] 거리, 서보, 시리얼에 대한 마지막 샘플링 시간을 나타내는 변수
bool event_dist, event_servo, event_serial; // [5283] 거리센서, 서보, 시리얼 모니터의 발생을 지시하는 변수

// Servo speed control
int duty_chg_per_interval;    //[3167] 주기동안 duty 변화량 변수
int duty_target, duty_curr;    // [3157] 서보의 목표위치와 서보에 실제로 입력할 위치

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; // [3401] 비례 제어를 위한 전에 측정한 오차, 새로 측정한 오차 값, 비례항, 적분항, 미분항 변수
int a,b;
void setup() {
// initialize GPIO pins for LED and attach servo 
  myservo.attach(PIN_SERVO);  // [3155] Servo 핀 설정
// initialize global variables
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0; // [3155] 샘플링 시각 기록 변수 초기화
  event_dist = event_servo = event_serial = false;  // [3155] 이벤트 bool값 초기화
  dist_target = _DIST_TARGET; // [3147] 목표지점 변수 초기화


// move servo to neutral position
  myservo.writeMicroseconds(1600); // [3147] 서보를 중간으로 이동
// initialize serial port
    Serial.begin(57600);                          // [3169] 57600 보드레이트로 아두이노와 통신
    a = 68.5;
    b = 271;
    iterm = 0.0;
// convert angle speed into duty change per interval.
  duty_chg_per_interval = int((_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED /  _SERVO_ANGLE) * (float(INTERVAL) / 1000.0));    //[3159] 서보 각속도에 맞추어 센서 인식 간격 변경? [3150] 서보의 각속도를 원하는 Angle로 나누어 interval을 설정한다.
}
  
void loop() {
/////////////////////
// Event generator // [3155] 설정된 주기마다 이벤트 생성
/////////////////////
unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }


////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
     event_dist = false;
  // get a distance reading from the distance sensor
     float dist_filtered = filtered_ir_distance();  // [3157] 적외선 센서로 측정한 값에 필터를 적용한 값
     
     dist_filtered = 100 + 300.0 / (b - a) * (dist_filtered - a);
     
  // PID control logic
    error_curr = _DIST_TARGET - dist_filtered; // [3158] 목표값 에서 현재값을 뺀 값이 오차값
    dterm = _KD*(error_curr-error_prev); //[3148] dterm 추가
    pterm = _KP*error_curr; 
    iterm += _KI*error_curr;
    control = pterm + dterm + iterm;      // [3158] P제어 이기때문에 pterm만 있음
//[3162] D제어를 위해 dterm 추가
  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control  ;   //[3162] duty target 설정

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) {
      duty_target = _DUTY_MIN;  // duty_target < _DUTY_MIN 일 때 duty_target 를 _DUTY_MIN 로 고정
    } else if (duty_target > _DUTY_MAX) {
       duty_target = _DUTY_MAX; // duty_target > _DUTY_MAX 일 때 duty_target 를 _DUTY_MAX 로 고정
    }  // [3153] [_DUTY_MIN, _DUTY_MAX] 로 서보의 가동범위를 고정하기 위한 최소한의 안전장치]
    error_prev = error_curr;      //[3159] error_prev 값 갱신
  }
  
  if(event_servo) {
    event_servo = false; // [3153] servo EventHandler Ticket -> false
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }  
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }       //[3166] 서보가 현재위치에서 목표위치에 도달할 때까지 duty_chg_per_interval값 마다 움직임(duty_curr에 duty_chg_per_interval값 더하고 빼줌)
    
    // update servo position
    myservo.writeMicroseconds(duty_curr);   //[3166]위에서 바뀐 현재위치 값을 갱신
  }

  //dist_raw =  100 + 300.0 / (b - a) * (dist_raw - a);
  if(event_serial) {
    event_serial = false; // [3153] serial EventHandler Ticket -> false
    float dist_raw = filtered_ir_distance();
    dist_raw = 100 + 300.0 / (b - a) * (dist_raw - a);
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
  }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;       //[3150] 적외선 센서를 통해 거리를 return시킨다.
}

// ================
float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}
