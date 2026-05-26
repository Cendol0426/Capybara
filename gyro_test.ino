#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
float angleZ = 0;
float gyroZoffset = 0;
unsigned long lastTime;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("FATAL: MPU6050 connection failed! Check SDA (Pin 20) and SCL (Pin 21).");
    while (1); // Halt if not found
  }
  
  Serial.println("Calibrating (Keep robot perfectly still!)...");
  long sum = 0;
  for(int i = 0; i < 500; i++){
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gyroZoffset = sum / 500.0;
  
  Serial.println("Calibration complete!");
  lastTime = millis();
}

void loop() {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  
  // Convert raw data to degrees per second
  float gyroRateZ = (gz - gyroZoffset) / 131.0; 
  
  // Integrate rate over time to get the absolute angle
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  
  angleZ += gyroRateZ * dt;
  
  Serial.print("Raw Rate: ");
  Serial.print(gyroRateZ);
  Serial.print(" | Accumulated Angle: ");
  Serial.println(angleZ);
  
  delay(50);
}
