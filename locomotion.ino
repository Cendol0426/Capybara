//Robosprint 2026 - Locomotion Control
//Step 1 Class setup for Motor, Encoder, and Movement
//Step 2 Implement basic movement functions (forward, backward, turn)
//Step 3 PID Correction for wheel slip and different speed
//Step 4 Gyro integration for better turn accuracy and straight movement correction
//Step 5 Command handler for Rpi communication and tuning parameters on the fly
//Step 6 Command status and data update for Rpi feedback and debugging
//Step 7 Development of control panel on Rpi for manual control and tuning
//Step 8 Testing and Iteration for optimal performance on the field
//Step 9 Integration with Rpi

/*Required Pins:
Motor A: IN1, IN2, ENA
Motor B: IN3, IN4, ENB
Encoder A: pinA, pinB
Encoder B: pinA, pinB
Ultrasonic: trigPin, echoPin
MPU6050: SDA, SCL
*/

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

// Pin definitions (adjust these to match your wiring)
#define LEFT_IN1 2
#define LEFT_IN2 3
#define LEFT_ENA 4
#define RIGHT_IN1 5
#define RIGHT_IN2 6
#define RIGHT_ENB 7
#define LEFT_ENCA 8
#define LEFT_ENCB 9
#define RIGHT_ENCA 10
#define RIGHT_ENCB 11

// Global objects
Motor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_ENA);
Motor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_ENB);
Encoder leftEnc(LEFT_ENCA, LEFT_ENCB, LEFT_ENCA, RIGHT_ENCA);
Encoder rightEnc(RIGHT_ENCA, RIGHT_ENCB, LEFT_ENCA, RIGHT_ENCA);
Gyro gyro;
Movement movement(leftMotor, rightMotor, leftEnc, rightEnc, gyro);
CommandHandler cmd(movement);

class Motor{
  private:
    int IN1, IN2, PWM;
    int speed;//0-255
  public:
    Motor(int in1, int in2, int pwm){
      IN1 = in1;
      IN2 = in2;
      PWM = pwm;
      speed = 0;
    }

    void begin(){
      pinMode(IN1, OUTPUT);
      pinMode(IN2, OUTPUT);
      pinMode(PWM, OUTPUT);
    }

    void forward(){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    }

    void backward(){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }

    void setSpeed(int spd) {
      speed = constrain(abs(spd), 0, 255);
      analogWrite(PWM, speed);
    }

    int getSpeed() {
      return speed;
    }

    void stop(){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(PWM, 0);
    }

    void brake() {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, HIGH);
      analogWrite(PWM, 0);
    }
};

class Encoder{
  private:
    int pinA, pinB;
    int enA1 = 2; // example pin for left encoder A
    int enA2 = 3; // example pin for right encoder A
    volatile long ticks;

  public:
    Encoder(int a, int b, int enA1, int enA2) {
      pinA = a;
      pinB = b;
      this->enA1 = enA1; // store encoder A pins for interrupt setup
      this->enA2 = enA2; // store encoder A pins for interrupt setup
      ticks = 0;
    }

    void begin() {
      pinMode(pinA, INPUT_PULLUP);
      pinMode(pinB, INPUT_PULLUP);
    }

    long getTicks() {
      noInterrupts();   // stop ISR temporarily
      long value = ticks;
      interrupts();     // resume ISR
      return value;
    }

    void reset() {
      ticks = 0;
    }

    void update() {
      if (digitalRead(pinB) == HIGH) ticks++;
      else ticks--;
    }

    static Encoder* leftInstance;
    static Encoder* rightInstance;

    static void leftISR() {
      leftInstance->update();
    }

    static void rightISR() {
      rightInstance->update();
    }

    void attachInterruptHandler() {
      if (pinA == enA1) {
        leftInstance = this;
        attachInterrupt(digitalPinToInterrupt(pinA), leftISR, CHANGE);
      }
      else if (pinA == enA2) {
        rightInstance = this;
        attachInterrupt(digitalPinToInterrupt(pinA), rightISR, CHANGE);
      }
    }
    //=============================================================================
    /*
    Encoder leftEnc(2, 4, 2, 3);
    Encoder rightEnc(3, 5, 2, 3);

    void setup() {
      Serial.begin(9600);

      // Initialize pins
      leftEnc.begin();
      rightEnc.begin();

      // Attach interrupts
      leftEnc.attachInterruptHandler();
      rightEnc.attachInterruptHandler();*/
//=============================================================================
};  
Encoder* Encoder::leftInstance = nullptr;
Encoder* Encoder::rightInstance = nullptr;

class PID{
  private:
    float Kp; // proportional gain
    float Ki; // integral gain
    float Kd; // derivative gain
    float prevError; // to calculate derivative
    float integral; // to prevent integral windup
    float integralLimit; // to prevent integral windup

  public:
    PID(float p, float i, float d, float integralLimit){
      Kp = p; Ki = i; Kd = d;
      prevError = 0;
      integral = 0;
      this->integralLimit = integralLimit;
    }

    void reset(){
      prevError = 0;
      integral = 0;
    }

    int compute(float error){
      //Integral - accumulated error
      integral += error;
      integral = constrain(integral, -integralLimit, integralLimit); // prevent windup

      //Derivative - change in error
      float derivative = error - prevError;

      //PID output 
      float output = Kp * error + Ki * integral + Kd * derivative;
      prevError = error; // update previous error for next derivative calculation
      return (int)output;
    }
};

class Gyro{
  private:
    MPU6050 mpu;
    float angleZ;
    float gyroZoffset;
    unsigned long lastTime;

    //Optional Smoothing, Tune Later!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    float alpha = 0.98; // complementary filter constant
    float filteredZ;

  public:
    Gyro() : angleZ(0), gyroZoffset(0), filteredZ(0) {}

    void begin(){
      Wire.begin();
      mpu.initialize();

      if (!mpu.testConnection()) {//Add check for MPU6050 connection later !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        Serial.println("MPU6050 connection failed");
        while (1);
      }

      calibrate();
      lastTime = millis();
    }

    void calibrate(int samples = 500){
      long sum = 0;
      for(int i = 0; i < samples; i++){
        int16_t gx, gy, gz;
        mpu.getRotation(&gx, &gy, &gz);
        sum += gz;
        delay(2);
      }
      gyroZoffset = sum / (float)samples;
      //When calibrating, robot is stationary and measure the average movement of the gyro to determine average offset.
    }

    void update(){
      int16_t gx, gy, gz;
      mpu.getRotation(&gx, &gy, &gz);
      float gyroZ = (gz - gyroZoffset) / 131.0; // convert to degrees/s

      //Optional Smoothing
      filteredZ = alpha * filteredZ + (1 - alpha) * gyroZ; // complementary filter for smoothing gyro data

      unsigned long now = millis();
      float dt = (now - lastTime) / 1000.0; // convert to seconds
      lastTime = now;

      angleZ += filteredZ * dt; // integrate gyro Z to get angle
    }

    float getAngleZ() {
      return angleZ;
    }

    void reset(){
      angleZ = 0;
    }

    float getRawRateZ(){
      int16_t gx, gy, gz;
      mpu.getRotation(&gx, &gy, &gz);
      return (gz - gyroZoffset) / 131.0; // return raw gyro Z rate for use in complementary filter or direct correction
    }
};

class Movement{
  private:
    //Declare motors and encoders
    Motor &leftMotor;
    Motor &rightMotor;
    Encoder &leftEnc;
    Encoder &rightEnc;
    Gyro gyro;

    //(YOU MUST TUNE THESE)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Distance Constants
    int ticksPerCm = 20;   // example value
    int ticksPerDegree = 5; // example value
    int decelRange = 100; // start decelerating when within 100 ticks of target
    float turnSlowZone = 20; // degrees within target angle to start slowing down for turns
    float turnStopZone = 5; // degrees within target angle to stop for turns
    float remainingGyroRate = 8; // degrees/s threshold to consider turn complete to prevent overshooting due to inertia

    //Time Constants
    unsigned long timeout = 5000; // 5 second timeout for movements
    int brakeTime = 30; // time to apply brake in ms
    
    //Callibration Constants
    PID straightPID = PID(2.0, 0.0, 0.0, 100.0); // PID for straight movement correction, adjust Kp and integral limit as needed
    PID turnPID     = PID(2.0, 0.0, 0.0, 100.0); // PID for turn correction, adjust Kp and integral limit as needed
    PID balancePID  = PID(1.0, 0.0, 0.0, 100.0); // PID for balancing encoder ticks during turns, adjust Kp and integral limit as needed
    float weightEnc = 0.5;// weight for encoder correction
    float weightGyro = 1.0;// weight for gyro correction

    //Speed Constants
    int minSpeed = 80; // minimum speed to prevent stalling
    int baseSpeed = 150;
    int turnSpeed = 120;

    long prevError = 0;
    float integral = 0;

    void moveStraight(long target, bool forward){//STRAIGHT FUNCTION
      leftEnc.reset();
      rightEnc.reset();
      straightPID.reset();
      gyro.reset();

      if (forward) { //forward or backward
        leftMotor.forward();
        rightMotor.forward();
      } else {
        leftMotor.backward();
        rightMotor.backward();
      }

      leftMotor.setSpeed(baseSpeed); //set initial speed
      rightMotor.setSpeed(baseSpeed);

      unsigned long startTime = millis();

      while ((abs(leftEnc.getTicks()) + abs(rightEnc.getTicks())) / 2 < target) {
        if (millis() - startTime > timeout) break; // 5 second timeout
      
        long leftTicks = leftEnc.getTicks();
        long rightTicks = rightEnc.getTicks(); //acquire encoder ticks
        long currentDis = (abs(leftTicks) + abs(rightTicks)) / 2; // calculate current distance traveled
        long remaining = target - currentDis; // calculate remaining distance
      
        if (currentDis >= target) break; // check if target distance is reached

        int speed;
        if (remaining < decelRange) {
          float ratio = (float)remaining / decelRange; // calculate ratio for deceleration
          ratio = constrain(ratio, 0.0, 1.0); // ensure ratio is between 0 and 1
          speed = minSpeed + (baseSpeed - minSpeed) * (ratio * ratio * ratio); // cubic deceleration curve for smoother stop, adjust exponent for more/less aggressive decel
          speed = constrain(speed, minSpeed, baseSpeed);
        } 
        else {
          speed = baseSpeed;
        }

        gyro.update(); // update gyro reading for straight correction
        float gyroError = gyro.getAngleZ(); // get current angle error from gyro
        long error = leftTicks - rightTicks; // calculate error between left and right ticks
        float combinedError = weightEnc * error + weightGyro * gyroError; // combine encoder and gyro errors for correction
        int correction = straightPID.compute(combinedError); // compute PID correction based on combined error

        int leftSpeed = constrain(speed - correction, minSpeed, 255);
        int rightSpeed = constrain(speed + correction, minSpeed, 255);// adjust speeds based on error
        
        leftMotor.setSpeed(leftSpeed);
        rightMotor.setSpeed(rightSpeed);
      }
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0);
      brake(); // apply brake for more immediate stop after movement
      delay(brakeTime);
      stop();
    }

    void turn(long targetAngle, bool right) {//TURN FUNCTION
      leftEnc.reset();
      rightEnc.reset();
      turnPID.reset();
      gyro.reset();

      if (right) {
        leftMotor.forward();
        rightMotor.backward();
      } else {
        leftMotor.backward();
        rightMotor.forward();
      }
      
      unsigned long startTime = millis();

      while (true) {
        if (millis() - startTime > timeout) break;
        
        gyro.update(); //update and acquire gyro angle

        float gyroRateZ = abs(gyro.getRawRateZ()); // get raw gyro Z rate for direct correction  
        float angle = abs(gyro.getAngleZ());
        if (angle >= targetAngle) break;

        float error = targetAngle - angle;

        //Stop Condition
        if (error < turnStopZone && gyroRateZ < remainingGyroRate) {
          break;// if within stop zone and gyro rate is low, consider turn complete to prevent overshooting due to inertia
        }

        //Main Control (gyro)
        int speed = turnPID.compute(error);

        if (error < turnSlowZone && error > 0){//Gradual Deceleration 
          float ratio = error / turnSlowZone; // calculate ratio for deceleration
          ratio = constrain(ratio, 0.0, 1.0); // ensure ratio is between 0 and 1
          speed = minSpeed + (turnSpeed - minSpeed) * (ratio * ratio * ratio); // cubic deceleration curve for smoother stop, adjust exponent for more/less aggressive decel
        }

        speed = constrain(speed, minSpeed, turnSpeed);

        //Balance control (encoder)
        long leftTicks = abs(leftEnc.getTicks());
        long rightTicks = abs(rightEnc.getTicks());

        long balanceError = leftTicks - rightTicks;
        int correction = balancePID.compute(balanceError);
        int leftSpeed;
        int rightSpeed;

        if (right) {
          leftSpeed  = speed - correction;
          rightSpeed = speed + correction;
        } else {
          leftSpeed  = speed + correction;
          rightSpeed = speed - correction;
        } // adjust speeds based on error for more accurate turns

        leftMotor.setSpeed(constrain(leftSpeed, 0, 255));
        rightMotor.setSpeed(constrain(rightSpeed, 0, 255));
      }
      
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0);
      brake(); // apply brake for more immediate stop after turn
      delay(brakeTime);
      stop();
    }

  public:
    Movement(Motor &lm, Motor &rm,
             Encoder &le, Encoder &re, Gyro &g)
      : leftMotor(lm), rightMotor(rm),
        leftEnc(le), rightEnc(re), gyro(g) {}

    void moveForward(float cm) {
      long target = cm * ticksPerCm;
      moveStraight(target, true);
    }

    void moveBackward(float cm) {
      long target = cm * ticksPerCm;
      moveStraight(target, false);
    }

    void turnRight(float degrees) {
      long target = degrees * ticksPerDegree;
      turn(target, true);
    }

    void turnLeft(float degrees) {
      long target = degrees * ticksPerDegree;
      turn(target, false);
    }   

    void brake() {
      leftMotor.brake();
      rightMotor.brake();
    }

    void stop() {
      leftMotor.stop();
      rightMotor.stop();
    }

    void setTicksPerCm(float v) { ticksPerCm = v; }
    void setTicksPerDegree(float v) { ticksPerDegree = v; }
    void setDecelRange(int v) { decelRange = v; }

    void setTurnSlowZone(float v) { turnSlowZone = v; }
    void setTurnStopZone(float v) { turnStopZone = v; }
    void setGyroRateLimit(float v) { remainingGyroRate = v; }

    void setMinSpeed(int v) { minSpeed = v; }
    void setBaseSpeed(int v) { baseSpeed = v; }
    void setTurnSpeed(int v) { turnSpeed = v; }

    void setWeightEnc(float v) { weightEnc = v; }
    void setWeightGyro(float v) { weightGyro = v; }

    void setStraightPID(float kp, float ki, float kd) {
      straightPID = PID(kp, ki, kd, 100.0);
    }

    void setTurnPID(float kp, float ki, float kd) {
      turnPID = PID(kp, ki, kd, 100.0);
    }

    void setBalancePID(float kp, float ki, float kd) {
      balancePID = PID(kp, ki, kd, 100.0);
    }
};

class CommandHandler{
  private:
    Movement &movement;
    String buffer;
    bool isMoving = false;
    /*
    T:TC= ticks per cm
    T:TD= ticks per degree
    T:DR= decel range
    T:TW= turn slow zone
    T:TP= turn stop zone
    T:GR= gyro rate limit for turn stop
    T:MS= min speed
    T:BS= base speed
    T:TS= turn speed
    T:WE= weight encoder
    T:WG= weight gyro
    T:SPID= straight PID (format: SPID=kp,ki,kd)
    T:TPID= turn PID (format: TPID=kp,ki,kd)
    T:BPID= balance PID (format: BPID=kp,ki,kd)
    M:FWD: Forward
    M:BWD: Backward
    M:TR: Turn Right
    M:TL: Turn Left
    M:STOP : Stop immediately
    M:BRK: Brake immediately
    */
    public:
    CommandHandler(Movement &m) : movement(m) {}

    void update(){
      while (Serial.available()){
        char c = Serial.read();

        if (c=='\r') continue; // ignore carriage return
  
        if (c == '\n') {
          handleCommand(buffer);
          buffer = "";
        } 
        else {
          buffer += c;
        }
      }
    }

    void handleCommand(String cmd){
      cmd.trim(); // remove any leading/trailing whitespace
      if (cmd.startsWith("T:")){
        handleTune(cmd.substring(2));
      }
      else if (cmd.startsWith("M:")){
        handleMove(cmd.substring(2));
      }
    }

    void handleTune(String cmd){//Tuning Commands
      //Extract Key
      int eq = cmd.indexOf('=');
      if (eq == -1) {
        Serial.println("Invalid command format");
        return;
      }

      String key = cmd.substring(0, eq);
      String value = cmd.substring(eq + 1);
      value.trim();

      //Distance Tuning
      if (key == "TC"){movement.setTicksPerCm(value.toFloat());} // Ticks per cm 
      else if (key == "TD"){movement.setTicksPerDegree(value.toFloat());} // Ticks per degree for turns
      else if (key == "DR"){movement.setDecelRange(value.toInt());} // Deceleration range for straight movements
      else if (key == "TW"){movement.setTurnSlowZone(value.toFloat());} // Degrees within target angle to start slowing down for turns
      else if (key == "TP"){movement.setTurnStopZone(value.toFloat());} // Degrees within target angle to stop for turns
      else if (key == "GR"){movement.setGyroRateLimit(value.toFloat());} // Gyro rate limit for turn stop condition to prevent overshooting due to inertia
      else if (key == "MS"){movement.setMinSpeed(value.toInt());} // Minimum speed to prevent stalling
      else if (key == "BS"){movement.setBaseSpeed(value.toInt());} // Base speed for straight movements
      else if (key == "TS"){movement.setTurnSpeed(value.toInt());} // Base speed for turns
      else if (key == "WE"){movement.setWeightEnc(value.toFloat());} // Weight for encoder correction in straight movement PID
      else if (key == "WG"){movement.setWeightGyro(value.toFloat());} // Weight for gyro correction in straight movement PID
      
      else if (key == "SPID"){
        float kp, ki, kd;
        sscanf(value.c_str(), "%f,%f,%f", &kp, &ki, &kd);
        movement.setStraightPID(kp, ki, kd);
      }
      else if (key == "TPID"){
        float kp, ki, kd;
        sscanf(value.c_str(), "%f,%f,%f", &kp, &ki, &kd);
        movement.setTurnPID(kp, ki, kd);
      }
      else if (key == "BPID"){
        float kp, ki, kd;
        sscanf(value.c_str(), "%f,%f,%f", &kp, &ki, &kd);
        movement.setBalancePID(kp, ki, kd);
      }
      else {
        Serial.println("Unknown tuning parameter");
        return;
      }
      Serial.println("Tuning" + key + " set to " + value);
    }

    void handleMove(String cmd){
      if (isMoving) {
        Serial.println("Busy");
        return;
      }

      isMoving = true;

      //Extract Command and value
      int sep = cmd.indexOf(':');
      if (sep == -1) {
        isMoving = false;
        Serial.println("Invalid command format");
        return;
      }

      String action = cmd.substring(0, sep);
      action.toUpperCase();
      String value = cmd.substring(sep + 1);
      value.trim();

      if (action == "FWD"){movement.moveForward(value.toFloat());} // Move forward by specified cm
      else if (action == "BWD"){movement.moveBackward(value.toFloat());} // Move backward by specified cm
      else if (action == "TR"){movement.turnRight(value.toFloat());} // Turn right by specified degrees
      else if (action == "TL"){movement.turnLeft(value.toFloat());} // Turn left by specified degrees
      else if (action == "STOP"){movement.stop();} // Stop immediately
      else if (action == "BRK"){movement.brake();} // Brake immediately
      else {
        Serial.println("Unknown movement command");
        isMoving = false;
        return;
      }
      isMoving = false;
      Serial.println("Movement " + action + " executed");
    }
};

void setup(){
  Serial.begin(115200);
  Wire.begin();

  leftMotor.begin();
  rightMotor.begin();

  leftEnc.begin();
  rightEnc.begin();

  leftEnc.attachInterruptHandler();
  rightEnc.attachInterruptHandler();
  gyro.begin();

  Serial.println("Ready");
}

void loop(){
  cmd.update();
}