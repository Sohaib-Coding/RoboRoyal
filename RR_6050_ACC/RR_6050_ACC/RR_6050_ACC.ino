#include "RR_6050.h"

RR_6050 mpu;

void setup() {
  Serial.begin(115200);
  
  Serial.println("Starting MPU-6050 initialization...");
  mpu.begin();
  
  Serial.println("Calibrating sensors...");
  Serial.println("Please keep the device still during calibration");
  delay(2000);
  mpu.calibrate();
  
  if (mpu.isCalibrated()) {
    Serial.println("Calibration successful!");
  } else {
    Serial.println("Calibration failed!");
  }
  
  // Set print interval to 100 milliseconds
  mpu.setPrintInterval(100);
}

void loop() {
  // Update sensor readings
  mpu.update();
  
  // Print accelerometer values
  Serial.print("AccX: "); Serial.print(mpu.getAccX());
  Serial.print("\tAccY: "); Serial.print(mpu.getAccY());
  Serial.print("\tAccZ: "); Serial.print(mpu.getAccZ());
  
  // Print gravity values in g units
  Serial.print("\tGX: "); Serial.print(mpu.getGravityX(), 2);
  Serial.print("\tGY: "); Serial.print(mpu.getGravityY(), 2);
  Serial.print("\tGZ: "); Serial.print(mpu.getGravityZ(), 2);
  
  // Print orientation angles
  Serial.print("\tYaw: "); Serial.print(mpu.getYaw(), 1);
  Serial.print("\tPitch: "); Serial.print(mpu.getPitch(), 1);
  Serial.print("\tRoll: "); Serial.println(mpu.getRoll(), 1);
  
  delay(100);
}
