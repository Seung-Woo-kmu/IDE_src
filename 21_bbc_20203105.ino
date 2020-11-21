#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_LED 9

// configurable parameters
#define _DUTY_MIN 1200 
#define _DUTY_NEU 1400 // servo neutral position (90 degree)
#define _DUTY_MAX 1600 
#define _SERVO_SPEED 2000 // servo speed limit (unit: degree/second)
#define INTERVAL 20  // servo update interval

// global variables
unsigned long last_sampling_time; // unit: ms
int duty_chg_per_interval; // maximum duty difference per interval
Servo myservo;
int duty_curr;
int a, b; // unit: mm

void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO); 
  duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr);
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
// initialize serial port
  Serial.begin(57600);
  a = 70;
  b = 300;

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (INTERVAL / 1000.0);
  
// initialize last sampling time
  last_sampling_time = 0;
}
float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
// wait until next sampling time.
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a); 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// adjust duty_curr toward duty_target by duty_chg_per_interval
  if(dist_cali > 255) {
    duty_curr -= duty_chg_per_interval;
    if (duty_curr >= _DUTY_MIN){
    }
    else{
      duty_curr = _DUTY_MIN;
    }
    myservo.writeMicroseconds(duty_curr);
  }
  else {
    duty_curr += duty_chg_per_interval;
    if (duty_curr <= _DUTY_MAX){
    }
    else{
      duty_curr = _DUTY_MAX;
    }
    myservo.writeMicroseconds(duty_curr);
  }


// output the read value to the serial port
  Serial.print(",duty_curr:");
  Serial.print(duty_curr);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);



// update last sampling time
  last_sampling_time += INTERVAL;
}
