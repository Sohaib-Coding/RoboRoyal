// SOURCE FILE (RR_6050.cpp)
#include "RR_6050.h"
#include <Wire.h>
#include <math.h>

// Gyro scale factor (250dps range)
#define GYRO_SCALE ((250.0 / 32768.0) * (PI / 180.0))

// Accelerometer scale factor (±2g range)
#define ACC_SCALE (2.0 / 32768.0)

RR_6050::RR_6050(uint8_t address) {
  MPU_addr = address;
  gyroOffsets[0] = gyroOffsets[1] = gyroOffsets[2] = 0.0;
  accOffsets[0] = accOffsets[1] = accOffsets[2] = 0.0;
  quat[0] = 1.0; quat[1] = quat[2] = quat[3] = 0.0;
  yaw = pitch = roll = 0.0;
  accX = accY = accZ = 0.0;
  gravityX = gravityY = gravityZ = 0.0;
  print_interval = 100;
  calibrated = false;
  last_print = 0;
  last_micros = 0;
}

void RR_6050::begin() {
  Wire.begin();
  // Wake up MPU-6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up device
  Wire.endTransmission(true);
  
  // Configure accelerometer for ±2g range
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x00); // 0x00 = ±2g, 0x08 = ±4g, 0x10 = ±8g, 0x18 = ±16g
  Wire.endTransmission(true);
  
  // Configure gyroscope for ±250°/s range
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x00); // 0x00 = ±250°/s, 0x08 = ±500°/s, 0x10 = ±1000°/s, 0x18 = ±2000°/s
  Wire.endTransmission(true);
}

void RR_6050::calibrate(uint16_t samples) {
  long gyroSums[3] = {0, 0, 0};
  long accSums[3] = {0, 0, 0};
  
  for (uint16_t i = 0; i < samples; i++) {
    // Read accelerometer
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 6);
    
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    
    accSums[0] += ax;
    accSums[1] += ay;
    accSums[2] += az;
    
    // Read gyroscope
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x43); // GYRO_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 6);
    
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();
    
    gyroSums[0] += gx;
    gyroSums[1] += gy;
    gyroSums[2] += gz;
    
    delay(2); // Short delay between samples
  }
  
  // Calculate average offsets
  for (int k = 0; k < 3; k++) {
    gyroOffsets[k] = gyroSums[k] / float(samples);
    
    // For accelerometer, we only calibrate X and Y
    // For Z, we subtract gravity
    if (k == 2) {
      // Expected value for Z is 1g (16384 for ±2g range)
      accOffsets[k] = accSums[k] / float(samples) - 16384.0;
    } else {
      accOffsets[k] = accSums[k] / float(samples);
    }
  }
  
  calibrated = true;
}

bool RR_6050::isCalibrated() {
  return calibrated;
}

void RR_6050::update() {
  // Read accelerometer data
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6);
  
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  
  // Apply calibration offsets to accelerometer data
  accX = ax - accOffsets[0];
  accY = ay - accOffsets[1];
  accZ = az - accOffsets[2];
  
  // Convert to g units
  gravityX = accX * ACC_SCALE;
  gravityY = accY * ACC_SCALE;
  gravityZ = accZ * ACC_SCALE;
  
  // Read gyro data
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43); // GYRO_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6);
  
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();
  
  // Convert to radians/sec and apply offsets
  float gyroData[3] = {
    (gx - gyroOffsets[0]) * GYRO_SCALE,
    (gy - gyroOffsets[1]) * GYRO_SCALE,
    (gz - gyroOffsets[2]) * GYRO_SCALE
  };
  
  // Calculate time since last update
  unsigned long now = micros();
  float deltat = (now - last_micros) * 1.0e-6f;
  last_micros = now;
  
  // Update orientation using Mahony filter
  mahonyUpdate(gyroData[0], gyroData[1], gyroData[2], deltat);
  
  // Calculate Euler angles from quaternion
  roll  = atan2((quat[0]*quat[1] + quat[2]*quat[3]), 
         0.5f - (quat[1]*quat[1] + quat[2]*quat[2])) * 180.0f / PI;
  pitch = asin(-2.0f * (quat[1]*quat[3] - quat[0]*quat[2])) * 180.0f / PI;
  yaw   = atan2((quat[1]*quat[2] + quat[0]*quat[3]), 
         0.5f - (quat[2]*quat[2] + quat[3]*quat[3])) * 180.0f / PI;
  
  // Normalize yaw to 0-360 degrees
  if (yaw < 0) yaw += 360.0f;
}

void RR_6050::mahonyUpdate(float gx, float gy, float gz, float deltat) {
  float q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];
  
  // Apply gyro integration
  gx *= 0.5f * deltat;
  gy *= 0.5f * deltat;
  gz *= 0.5f * deltat;
  
  quat[0] += (-q1 * gx - q2 * gy - q3 * gz);
  quat[1] += (q0 * gx + q2 * gz - q3 * gy);
  quat[2] += (q0 * gy - q1 * gz + q3 * gx);
  quat[3] += (q0 * gz + q1 * gy - q2 * gx);
  
  // Normalize quaternion
  float recipNorm = 1.0f / sqrt(quat[0]*quat[0] + quat[1]*quat[1] + 
                    quat[2]*quat[2] + quat[3]*quat[3]);
  quat[0] *= recipNorm;
  quat[1] *= recipNorm;
  quat[2] *= recipNorm;
  quat[3] *= recipNorm;
}

void RR_6050::printData() {
  if (millis() - last_print >= print_interval) {
    last_print = millis();
    Serial.print("Yaw: "); Serial.print(yaw, 1);
    Serial.print("°\tPitch: "); Serial.print(pitch, 1);
    Serial.print("°\tRoll: "); Serial.print(roll, 1);
    Serial.println("°");
  }
}

void RR_6050::printAccData() {
  if (millis() - last_print >= print_interval) {
    last_print = millis();
    Serial.print("AccX: "); Serial.print(gravityX, 3);
    Serial.print("g\tAccY: "); Serial.print(gravityY, 3);
    Serial.print("g\tAccZ: "); Serial.print(gravityZ, 3);
    Serial.println("g");
  }
}