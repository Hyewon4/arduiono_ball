#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_INFRARED  0  // infrared sensor analog A0
#define PIN_SERVO 10  // servi motor

// configurable parameters for sonar
#define INTERVAL 20      // sampling interval (unit: msec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 250.0   // maximum distance to be measured (unit: mm)

#define _EMA_ALPHA 0.11  // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.

// Target Distance
#define _TARGET_LOW  100.0
#define _TARGET_HIGH 250.0

// duty duration for myservo.writeMicroseconds()
// NEEDS TUNING (servo by servo)
 
#define _DUTY_MIN 553 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1476// servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counterclockwise position (180 degree)

// global variables
float  dist_ema, dist_prev = _DIST_MAX; // unit: mm
unsigned long last_sampling_time; // unit: ms

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  //pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_INFRARED, INPUT);   // infrared sensor

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

  // initialize USS related variables
  dist_prev = _DIST_MIN; // raw distance output from USS (unit: mm)

  // initialize serial port
  Serial.begin(2000000);
}

void loop() {
  float  dist_raw;
  int a_value, duty;
  
  // wait until next sampling time. 
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  //dist_raw = USS_measure(PIN_TRIG, PIN_ECHO); // read distance
  a_value = analogRead(PIN_INFRARED);
  dist_raw = (6762.0/(a_value-9)-4.0)*10.0 - 60.0;

  if (dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;           // cut lower than minimum
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else if (dist_raw > _DIST_MAX) {
    dist_raw = dist_prev;           // Cut higher than maximum
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else {    // In desired Range
    digitalWrite(PIN_LED, 0);       // LED ON      
    dist_prev = dist_raw;
  }

  // Apply ema filter here  
  dist_ema = _EMA_ALPHA* dist_raw + (1 - _EMA_ALPHA) * dist_ema;

  // adjust servo position according to the USS read value

  // add your code here!
  if (dist_ema <= _TARGET_LOW){
    myservo.writeMicroseconds(_DUTY_MIN);
  } else if(dist_ema >= _TARGET_HIGH){
    myservo.writeMicroseconds(_DUTY_MAX);
  } else{
    duty = int(round(_DUTY_MIN + ((_DUTY_MAX - _DUTY_MIN) / 150.0 * (dist_ema - _TARGET_LOW))));
    myservo.writeMicroseconds(duty);
  }
  // Use _TARGET_LOW, _TARGTE_HIGH

  // output the distance to the serial port
  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(",IR:");    Serial.print(a_value);
  Serial.print(",dist:");  Serial.print(dist_raw);
  Serial.print(",ema:");  Serial.print(dist_ema);
  Serial.print(",servo_duty:"); Serial.print(duty);
  Serial.print(",Max:");   Serial.print(_DIST_MAX);
  Serial.println("");
 
  // update last sampling time
  last_sampling_time += INTERVAL;
}


