#include <Wire.h>
#include <AccelStepper.h>
#include "Controller.h"

#define PIN_DIR 2
#define PIN_STEP 3
#define PIN_SLEEP 4
#define PIN_MS1 7
#define PIN_MS2 6
#define PIN_MS3 5
#define PIN_Trig 11
#define PIN_Echo 12
#define RAD_TO_STEP 31.831 // (180/PI)/1.8
#define LSB_TO_RAD 5.321E-4 // PI/(32.8*180)
#define SAMPLING_TIME 10
#define MICROSTEPS 8

AccelStepper MotorStepper(1, PIN_STEP, PIN_DIR);
unsigned long prevTime;
float pitch_error, pitch_rate_error, pitch;

unsigned long prevTime_micros;

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.beginTransmission(0x68);          
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);                  // +/-8g full scale range
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x10);                  // +/-1000°/s full scale range
  Wire.endTransmission(true);
  
  calibrateSensor();

  pinMode(13, OUTPUT);
  pinMode(PIN_Trig, OUTPUT);
  pinMode(PIN_Echo, INPUT);
  pinMode(PIN_SLEEP, OUTPUT);
  pinMode(PIN_MS1, OUTPUT);
  pinMode(PIN_MS2, OUTPUT);
  pinMode(PIN_MS3, OUTPUT);

  digitalWrite(13, HIGH);
  digitalWrite(PIN_SLEEP, HIGH);
  digitalWrite(PIN_MS1, HIGH);
  digitalWrite(PIN_MS2, HIGH);
  digitalWrite(PIN_MS3, LOW);

  MotorStepper.setMaxSpeed(100000);
  prevTime = millis();
  pitch = measurePitch();
}

void loop() {
  prevTime_micros = micros();
  unsigned long elapsedTime = millis()-prevTime;
  if(elapsedTime >= SAMPLING_TIME) {
    prevTime = millis();

    float pitch = measurePitch();
    Serial.println(1000000/(micros()-prevTime_micros));
    prevTime_micros = micros();
    float pitch_rate = measurePitchRate();

    // float control = controller.step(0, pitch, pitch_rate);
    float Kp = 4000.0;
    float Kd = 0;
    //pitch = pitch+pitch_rate*(elapsedTime/1000.0);
    float control = Kp*pitch+Kd*pitch_rate;
    control = 1000;
    MotorStepper.setSpeed(control);

    Serial.print(",pitch_rate:");
    Serial.println(pitch_rate);
    Serial.print("pitch:");
    Serial.print(pitch);
    Serial.print(",control:");
    Serial.println(control);
  }
  Serial.println(1000000/(micros()-prevTime_micros));
  MotorStepper.runSpeed();
  delay(1000);
}

void calibrateSensor() {
  pitch_error = 0.0;
  pitch_rate_error = 0.0;

  for(int i = 0; i < 200; i++) {
    pitch_error = pitch_error+measurePitch();
    pitch_rate_error = pitch_rate_error+measurePitchRate();
  }

  pitch_error = pitch_error/200.0;
  pitch_rate_error = pitch_rate_error/200.0;
}

float measurePitch() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(0x68, 6, false);
  int16_t acc_x = (Wire.read() << 8 | Wire.read());
  int16_t acc_y = (Wire.read() << 8 | Wire.read());
  int16_t acc_z = (Wire.read() << 8 | Wire.read());

  Serial.println("angles");
  Serial.println(acc_x);
  Serial.println(acc_y);
  Serial.println(acc_z);
  return atan(-acc_x/float(acc_z))-pitch_error;
}

float measurePitchRate() {
    Wire.beginTransmission(0x68);
    Wire.write(0x45);
    Wire.endTransmission(false);

    Wire.requestFrom(0x68, 2, true);
    int16_t pitch_rate = Wire.read() << 8 | Wire.read();

    return pitch_rate*LSB_TO_RAD-pitch_rate_error;
}