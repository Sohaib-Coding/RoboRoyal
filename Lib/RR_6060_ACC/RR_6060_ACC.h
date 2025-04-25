#ifndef RR_6060_h
#define RR_6060_h

#include <Arduino.h>
#include <Wire.h>

class RR_6060 {
  public:
    // Constructor with default I2C address (0x68)
    RR_6060(uint8_t address = 0x68);
    
    // Initialize the sensor
    void begin();
    
    // Update sensor readings and calculate angles
    void update();
    
    // Calibrate gyroscope (recommended before use)
    void calibrate(uint16_t samples = 500);
    
    // Check if calibration is done
    bool isCalibrated();
    
    // Get Euler angles
    float getYaw() { return yaw; }
    float getPitch() { return pitch; }
    float getRoll() { return roll; }
    
    // Get accelerometer raw readings
    float getAccX() { return accX; }
    float getAccY() { return accY; }
    float getAccZ() { return accZ; }
    
    // Get accelerometer in g units
    float getGravityX() { return gravityX; }
    float getGravityY() { return gravityY; }
    float getGravityZ() { return gravityZ; }
    
    // Set print interval in milliseconds
    void setPrintInterval(uint16_t interval) { print_interval = interval; }
    
    // Print angles to Serial
    void printData();
    
    // Print accelerometer data to Serial
    void printAccData();
    
  private:
    // Mahony AHRS update
    void mahonyUpdate(float gx, float gy, float gz, float deltat);
    
    // I2C address
    uint8_t MPU_addr;
    
    // Gyro offsets
    float gyroOffsets[3];
    
    // Accelerometer offsets
    float accOffsets[3];
    
    // Quaternion
    float quat[4];
    
    // Euler angles
    float yaw, pitch, roll;
    
    // Accelerometer values (raw)
    float accX, accY, accZ;
    
    // Accelerometer values (g units)
    float gravityX, gravityY, gravityZ;
    
    // Timing variables
    unsigned long last_print;
    unsigned long last_micros;
    uint16_t print_interval;
    
    // Calibration flag
    bool calibrated;
};

#endif
