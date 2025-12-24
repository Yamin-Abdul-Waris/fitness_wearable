#include <Wire.h>

#define MPU_ADDR 0x68

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // Wake MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  delay(1000);
}

void loop() {
  float ax, ay, az;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);

  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)6, true);

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();

  ax = rawAx / 16384.0;
  ay = rawAy / 16384.0;
  az = rawAz / 16384.0;

  // CSV output (VERY IMPORTANT)
  Serial.print(ax, 3);
  Serial.print(",");
  Serial.print(ay, 3);
  Serial.print(",");
  Serial.println(az, 3);

  delay(20); // ~50 Hz
}
