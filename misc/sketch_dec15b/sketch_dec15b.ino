#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
MPU6050 mpu;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

void setup() {

    Wire.begin();
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    fifoCount = mpu.getFIFOCount();

    Serial.begin(115200);

}

void loop() {
    while (fifoCount < packetSize) {

      fifoCount = mpu.getFIFOCount();
    }

    if (fifoCount == 1024) {
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
        
    }
    else{
      if (fifoCount % packetSize != 0) {
        mpu.resetFIFO();
    }
        else{
          while (fifoCount >= packetSize) {
            mpu.getFIFOBytes(fifoBuffer,packetSize);
            fifoCount -= packetSize;
          }    
        
          mpu.dmpGetQuaternion(&q,fifoBuffer);
          mpu.dmpGetGravity(&gravity,&q);
          mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);          
            
          Serial.print("ypr\t");
          Serial.print(ypr[0]*180/PI);
          Serial.print("\t");
          Serial.print(ypr[1]*180/PI);
          Serial.print("\t");
          Serial.print(ypr[2]*180/PI);
          Serial.println();
            
    }
   
    }

}