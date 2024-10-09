#include "PitchSensor.h"
#include <BasicLinearAlgebra.h>
using namespace BLA;

#define SAMPLING_TIME 20
#define MAX_SPEED 5000
#define DIR_PIN 2
#define STEP_PIN 3
#define SLP_PIN 4
#define MS3_PIN 5
#define MS2_PIN 6
#define MS1_PIN 7
#define RAD_TO_STEP 31.831 // (180/PI)/1.8
#define MICROSTEPS 8
#define MAX_PITCH 0.5

PitchSensor pitch_sensor(SAMPLING_TIME);

void setup() {
  Serial.begin(115200);

  pitch_sensor.initialize();
  Serial.println("Calibrating the pitch sensor..");
  pitch_sensor.calibrate();

  // distance_sensor.initialize();

  pinMode(8, INPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(SLP_PIN, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);
  digitalWrite(SLP_PIN, HIGH);
  digitalWrite(MS1_PIN, HIGH);
  digitalWrite(MS2_PIN, HIGH);
  digitalWrite(MS3_PIN, LOW);

  Serial.println("Ready!");
}

void loop() {
  static unsigned long prev_time, elapsed_time;
  static float control, pitch_integral, pitch;
  static float filtered_pitch;

  elapsed_time = millis()-prev_time;
  if(elapsed_time >= SAMPLING_TIME) {
    pitch_integral = pitch_integral+pitch/2*elapsed_time/1000.0;
    pitch = pitch_sensor.measurePitch();
    float pitch_rate = pitch_sensor.measurePitchRate();
    pitch_sensor.filter(pitch, pitch_rate);
    pitch_integral = pitch_integral+pitch/2*elapsed_time/1000.0;
    //filtered_pitch = 0.98*(filtered_pitch+pitch_rate*((millis()-prev_time)/1000.0))+0.02*pitch;
    //pitch = filtered_pitch;

    //float ref = -0*3*vel_integral;

    // control = -50*pitch-5*pitch_rate-200*integral;
    if(abs(pitch) <= MAX_PITCH) {
      digitalWrite(SLP_PIN, HIGH);
      
      control = -50*(pitch)-1*pitch_rate-375*pitch_integral; //or pitch-ref
    }
    else {
      digitalWrite(SLP_PIN, LOW);
      control = 0;
      pitch_integral = 0;
    }
     
    //vel_integral = vel_integral+0.033*control*elapsed_time/1000.0;
    
    Serial.print("pitch:");
    Serial.print(pitch);
    Serial.print(",pitch_rate:");
    Serial.println(pitch_rate);
    Serial.print("control:");
    Serial.println(control);
    prev_time = millis();
  }

  controlMotor(control);
}

void controlMotor(float control) {
  static bool step;
  static unsigned long prev_time;
  float speed = constrain(abs(control)*RAD_TO_STEP*MICROSTEPS, 0, MAX_SPEED);

  if(speed > 31) {
    tone(STEP_PIN, speed);
  }
  else {
    if(millis()-prev_time >= 1000/(2*speed)) {
      digitalWrite(STEP_PIN, step);
      step = !step;
      prev_time = millis();
    }
  }

  digitalWrite(DIR_PIN, control >= 0);
}