#include <Wire.h>

const int MPU_addr = 0x68;
float G_off[3] = {0.0, 0.0, 0.0};
#define gscale ((250.0 / 32768.0) * (PI / 180.0))

float q[4] = {1.0, 0.0, 0.0, 0.0}; // quaternion
float yaw, pitch, roll;

unsigned long last_ms = 0;
int cal_gyro = 1;
long gsum[3] = {0, 0, 0};
unsigned int i = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void loop() {
  int16_t gx, gy, gz;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43); // GYRO_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6);
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();

  if (cal_gyro) {
    gsum[0] += gx; gsum[1] += gy; gsum[2] += gz;
    i++;
    if (i == 500) {
      for (int k = 0; k < 3; k++) G_off[k] = gsum[k] / 500.0;
      cal_gyro = 0;
    }
    return;
  }

  float Gxyz[3] = {
    (gx - G_off[0]) * gscale,
    (gy - G_off[1]) * gscale,
    (gz - G_off[2]) * gscale
  };

  static unsigned long last = 0;
  unsigned long now = micros();
  float deltat = (now - last) * 1.0e-6;
  last = now;

  Mahony_update(Gxyz[0], Gxyz[1], Gxyz[2], deltat);

  if (millis() - last_ms > 100) {
    last_ms = millis();
    roll  = atan2((q[0]*q[1] + q[2]*q[3]), 0.5 - (q[1]*q[1] + q[2]*q[2])) * 180.0 / PI;
    pitch = asin(2.0 * (q[0]*q[2] - q[1]*q[3])) * 180.0 / PI;
    yaw   = -atan2((q[1]*q[2] + q[0]*q[3]), 0.5 - (q[2]*q[2] + q[3]*q[3])) * 180.0 / PI;
    if (yaw < 0) yaw += 360.0;

    Serial.print("Yaw: "); Serial.print(yaw, 1);
    Serial.print(" Pitch: "); Serial.print(pitch, 1);
    Serial.print(" Roll: "); Serial.println(roll, 1);
  }
}

void Mahony_update(float gx, float gy, float gz, float deltat) {
  float qa = q[0], qb = q[1], qc = q[2];

  gx *= 0.5 * deltat;
  gy *= 0.5 * deltat;
  gz *= 0.5 * deltat;

  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  float recipNorm = 1.0 / sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  for (int i = 0; i < 4; i++) q[i] *= recipNorm;
}
