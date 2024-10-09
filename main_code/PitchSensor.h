#ifndef PitchSensor_h
#define PitchSensor_h

#include "Arduino.h"
#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <HardwareSerial.h>
using namespace BLA;

#define LSB_TO_RAD 5.321E-4 // PI/(32.8*180)

class PitchSensor {
  public:
    PitchSensor(int sampling_time) {
      A = {1, sampling_time/1000.0, 0, 1};
      state = {0, 0};
      pred_err_cov = {1, 0, 0, 1};
      pitch_offset = 0;
      pitch_rate_offset = 0;
    }

    void initialize() {
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
      
      Wire.beginTransmission(0x68);
      Wire.write(0x1A);
      Wire.write(0x05);                  // low-pass filter
      Wire.endTransmission(true);
    }

    void calibrate() { // to be improved
      float new_pitch_offset = 0;
      float new_pitch_rate_offset = 0;

      for(int i = 0; i < 200; i++) {
        new_pitch_offset = new_pitch_offset+measurePitch();
        new_pitch_rate_offset = new_pitch_rate_offset+measurePitchRate();
        delay(10);
      }

      pitch_offset = new_pitch_offset/200.0;
      pitch_rate_offset = new_pitch_rate_offset/200.0;
    }

    float measurePitch() {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission(false);

      Wire.requestFrom(0x68, 6, false);
      int16_t acc_x = (Wire.read() << 8 | Wire.read());
      int16_t acc_y = (Wire.read() << 8 | Wire.read());
      int16_t acc_z = (Wire.read() << 8 | Wire.read());

      return atan(-acc_x/float(acc_z))-pitch_offset;
    }

    float measurePitchRate() {
        Wire.beginTransmission(0x68);
        Wire.write(0x45);
        Wire.endTransmission(false);

        Wire.requestFrom(0x68, 2, true);
        int16_t pitch_rate = Wire.read() << 8 | Wire.read();

        return pitch_rate*LSB_TO_RAD-pitch_rate_offset;
    }

    void filter(float &pitch, float &pitch_rate) {
      BLA::Matrix<2, 1> meas = {pitch, pitch_rate};
      
      BLA::Matrix<2, 2> k_gain = pred_err_cov*Inverse(pred_err_cov+G*~G);
      state = A*state+k_gain*(meas-A*state);
      BLA::Matrix<2, 2> err_cov = (I-k_gain)*pred_err_cov;
      pred_err_cov = A*err_cov*~A+H*~H;

      pitch = state(0);
      pitch_rate = state(1);
    }

  private:
    float pitch_offset;
    float pitch_rate_offset;

    const BLA::Matrix<2, 2> I = {1, 0, 0, 1};
    const BLA::Matrix<2, 2> A;
    // const BLA::Matrix<2, 2> C = I;
    const BLA::Matrix<2, 2> H = {0, 0, 0, 0.1};
    const BLA::Matrix<2, 2> G = {0.2, 0, 0, 0.2};
    BLA::Matrix<2, 1> state;
    BLA::Matrix<2, 2> pred_err_cov;
};

#endif