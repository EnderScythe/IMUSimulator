#include "MPU.h"

MPU *mpu = new MPU();

float roll = 0, pitch = 0, yaw = 0;
float prev = 0, current = 0;

void setup() {
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  mpu->startMPU();
  mpu->calibrateMPU(2000);
}

void loop() {
  current = micros();
  mpu->prepareMeasurements();
  roll = mpu->getRoll();
  pitch = mpu->getPitch();
  yaw = mpu->getYaw();
  //Serial.println(current - prev);
  Serial.print(roll); Serial.print(","); Serial.print(pitch); Serial.print(","); Serial.println(yaw);
  delay(50);
  prev = current;
}