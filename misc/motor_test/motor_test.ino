#include <Arduino.h>
#include "A4988.h"

#define STEP_PIN 6
#define DIR_PIN 5
#define SLEEP_PIN 
#define MS1_PIN 4
#define MS2_PIN 5
#define MS3_PIN 6

// Motor specs
const int spr = 200; // Steps per revolution
int u = 3.14; // Motor speed in rad/s
int microsteps = 1; // Step size (1 for full steps, 2 for half steps, 4 for quarter steps, etc)

//Providing parameters for motor control
A4988 stepper(spr, DIR_PIN, STEP_PIN, MS1_PIN, MS2_PIN, MS3_PIN);

void setup() {
  Serial.begin(9600);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(SLEEP_PIN,  OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  
  // Set target motor speed and microstepping
  stepper.begin(u, microsteps);
}

void loop() {
    digitalWrite(SLEEP_PIN, HIGH); //A logic high allows normal operation of the A4988 by removing from sleep
    stepper.rotate(360);
}