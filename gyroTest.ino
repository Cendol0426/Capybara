#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

float angleZ = 0;
float offset = 0;
unsigned long lastTime;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  // Calibrate
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  offset = sum / 500.0;

  lastTime = millis();
}

void loop() {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  float gyroZ = (gz - offset) / 131.0;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  angleZ += gyroZ * dt;

  Serial.println(angleZ);
}