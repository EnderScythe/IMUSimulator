//A lot of this code was taken from carbon aeronautics: https://github.com/CarbonAeronautics?tab=repositories

#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "math.h"

class MPU{
  public:
    prepareMeasurements();
    startMPU();
    calibrateMPU(int);
    float getRoll();
    float getPitch();
    float getYaw();
  private:
    lowPassFilter();
    getRotationRate(bool);
    getAcceleration();
    kalmanFilter1D(float, float, float, float);
    float rateRoll, ratePitch, rateYaw;
    float rateCaliRoll, rateCaliPitch, rateCaliYaw;
    float accX, accY, accZ;
    float roll, pitch, yaw;
    float accZInertial;
    float kalmanRoll = 0, kalmanPitch = 0, kalmanYaw = 0; // inital predictions when program first starts
    float kalmanUncertaintyRoll = 4, kalmanUncertaintyPitch = 4, kalmanUncertaintyYaw = 4;
    float kalman1DOutput[2] = {0, 0};
    float iterationLength = 0.0545;
};

MPU::startMPU(){
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

MPU::calibrateMPU(int measurements){
  for(int i = 0; i < measurements; i++){
    lowPassFilter();
    getRotationRate(false);
    rateCaliRoll += rateRoll;
    rateCaliPitch += ratePitch;
    rateCaliYaw += rateYaw;
    delay(1);
  }
  rateCaliRoll /= measurements;
  rateCaliPitch /= measurements;
  rateCaliYaw /= measurements;
  Serial.println("MPU Calibration Finished.");
}

MPU::lowPassFilter(){
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
}

MPU::getRotationRate(bool calibration){
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();
  rateRoll = (float) gyroX / 65.5;
  ratePitch = (float) gyroY / 65.5;
  rateYaw = (float) gyroZ / 65.5;
  if(calibration){
    rateRoll -= rateCaliRoll;
    ratePitch -= rateCaliPitch;
    rateYaw -= rateCaliYaw;
    //Serial.print(rateRoll); Serial.print(", ");
    //Serial.print(ratePitch); Serial.print(", ");
    //Serial.println(rateYaw);
  }
}

MPU::getAcceleration(){
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t accXLSB = Wire.read() << 8| Wire.read();
  int16_t accYLSB = Wire.read() << 8| Wire.read();
  int16_t accZLSB = Wire.read() << 8| Wire.read();
  accX = (float) accXLSB / 4096 - 0.01;  
  accY = (float) accYLSB / 4096 + 0.01;
  accZ = (float) accZLSB / 4096;
  //Serial.print(accX); Serial.print(", ");
  //Serial.print(accY); Serial.print(", ");
  //Serial.println(accZ);
}

float MPU::getRoll(){
  roll = 180 * atan(accY / sqrt(accX * accX + accZ * accZ)) / M_PI;
  kalmanFilter1D(kalmanRoll, kalmanUncertaintyRoll, rateRoll, roll);
  kalmanRoll = kalman1DOutput[0];
  kalmanUncertaintyRoll = kalman1DOutput[1];
  return kalmanRoll;
}

float MPU::getPitch(){
  pitch = 180 * atan(accX / sqrt(accY * accY + accZ * accZ)) / M_PI;
  kalmanFilter1D(kalmanPitch, kalmanUncertaintyPitch, ratePitch, pitch);
  kalmanPitch = kalman1DOutput[0];
  kalmanUncertaintyPitch = kalman1DOutput[1];
  return kalmanPitch;
}

float MPU::getYaw(){
  yaw = 180 * atan(sqrt(accX * accX + accY * accY) / accZ) / M_PI;
  kalmanFilter1D(kalmanYaw, kalmanUncertaintyYaw, rateYaw, yaw);
  kalmanYaw = kalman1DOutput[0];
  kalmanUncertaintyYaw = kalman1DOutput[1];
  return kalmanYaw;
}

//Right now ive set the iteration length to 0.055s set it to the time the arduino needs to do 1 loop
MPU::kalmanFilter1D(float state, float uncertainty, float input, float measurement){
  state = state + iterationLength * input;
  uncertainty = uncertainty + iterationLength * iterationLength * 4 * 4;
  float gain = uncertainty * 1 / ( 1 * uncertainty + 3 * 3);
  state = state + gain * (measurement - state);
  uncertainty = (1 - gain) * uncertainty;
  kalman1DOutput[0] = state;
  kalman1DOutput[1] = uncertainty;
}

MPU::prepareMeasurements(){
  lowPassFilter();
  getRotationRate(true);
  getAcceleration();
}
