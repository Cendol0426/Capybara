/*Required Pins:
Motor A: IN1, IN2, ENA
Motor B: IN3, IN4, ENB
Encoder A: pinA, pinB
Encoder B: pinA, pinB
Ultrasonic: trigPin, echoPin
Switch Button
*/

#include <Arduino.h>

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
#define button 12

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

class Movement{
  private:
    //Declare motors and encoders
    Motor &leftMotor;
    Motor &rightMotor;
    Encoder &leftEnc;
    Encoder &rightEnc;
    unsigned long lastPrint = 0;
    int printInterval = 50; // ms (20Hz)

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

    //Speed Constants
    int minSpeed = 80; // minimum speed to prevent stalling
    int baseSpeed = 150;
    int turnSpeed = 120;

    long prevError = 0;
    float integral = 0;

    void moveStraight(long target, bool forward){

      leftEnc.reset();
      rightEnc.reset();
      straightPID.reset();

      if(forward){
        leftMotor.forward();
        rightMotor.forward();
      }
      else{
        leftMotor.backward();
        rightMotor.backward();
      }

      leftMotor.setSpeed(baseSpeed);
      rightMotor.setSpeed(baseSpeed);

      unsigned long startTime = millis();

      while(true){

        if(millis() - startTime > timeout)
          break;

        long leftTicks  = leftEnc.getTicks();
        long rightTicks = rightEnc.getTicks();

        long currentDis =
          (abs(leftTicks) + abs(rightTicks)) / 2;

        if(currentDis >= target)
          break;

        long remaining = target - currentDis;

        int speed;

        // Deceleration
        if(remaining < decelRange){

          float ratio =
            (float)remaining / decelRange;

          ratio = constrain(ratio, 0.0, 1.0);

          speed =
            minSpeed +
            (baseSpeed - minSpeed) *
            (ratio * ratio * ratio);

          speed = constrain(speed,
                            minSpeed,
                            baseSpeed);
        }
        else{
          speed = baseSpeed;
        }

        // Encoder balancing only
        long error =
          leftTicks - rightTicks;

        int correction =
          straightPID.compute(error);

        int leftSpeed =
          constrain(speed - correction,
                    minSpeed,
                    255);

        int rightSpeed =
          constrain(speed + correction,
                    minSpeed,
                    255);

        int avgSpeed =
          (leftSpeed + rightSpeed) / 2;

        if(millis() - lastPrint > printInterval){

          Serial.print("ERR:");
          Serial.println(error);

          Serial.print("SPD:");
          Serial.println(avgSpeed);

          lastPrint = millis();
        }

        leftMotor.setSpeed(leftSpeed);
        rightMotor.setSpeed(rightSpeed);
      }

      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0);

      brake();

      delay(brakeTime);

      stop();
    }

    void turn(long targetTicks, bool right){

      leftEnc.reset();
      rightEnc.reset();

      turnPID.reset();
      balancePID.reset();

      if(right){
        leftMotor.forward();
        rightMotor.backward();
      }
      else{
        leftMotor.backward();
        rightMotor.forward();
      }

      unsigned long startTime = millis();

      while(true){

        if(millis() - startTime > timeout)
          break;

        long leftTicks =
          abs(leftEnc.getTicks());

        long rightTicks =
          abs(rightEnc.getTicks());

        long avgTicks =
          (leftTicks + rightTicks) / 2;

        if(avgTicks >= targetTicks)
          break;

        long remaining =
          targetTicks - avgTicks;

        int speed;

        // Deceleration
        if(remaining < decelRange){

          float ratio =
            (float)remaining / decelRange;

          ratio = constrain(ratio, 0.0, 1.0);

          speed =
            minSpeed +
            (turnSpeed - minSpeed) *
            (ratio * ratio * ratio);
        }
        else{
          speed = turnSpeed;
        }

        speed = constrain(speed,
                          minSpeed,
                          turnSpeed);

        // Encoder balancing
        long balanceError =
          leftTicks - rightTicks;

        int correction =
          balancePID.compute(balanceError);

        int leftSpeed;
        int rightSpeed;

        if(right){

          leftSpeed =
            speed - correction;

          rightSpeed =
            speed + correction;
        }
        else{

          leftSpeed =
            speed + correction;

          rightSpeed =
            speed - correction;
        }

        leftSpeed =
          constrain(leftSpeed,
                    minSpeed,
                    255);

        rightSpeed =
          constrain(rightSpeed,
                    minSpeed,
                    255);

        int avgSpeed =
          (leftSpeed + rightSpeed) / 2;

        if(millis() - lastPrint > printInterval){

          Serial.print("TURN:");
          Serial.print(avgTicks);

          Serial.print("/");

          Serial.print(targetTicks);

          Serial.print(" ERR:");
          Serial.print(balanceError);

          Serial.print(" SPD:");
          Serial.println(avgSpeed);

          lastPrint = millis();
        }

        leftMotor.setSpeed(leftSpeed);
        rightMotor.setSpeed(rightSpeed);
      }

      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0);

      brake();

      delay(brakeTime);

      stop();
    }

  public:
    Movement(Motor &lm, Motor &rm,
             Encoder &le, Encoder &re, Gyro &g)
      : leftMotor(lm), rightMotor(rm),
        leftEnc(le), rightEnc(re) {}

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

void setup(){
  Serial.begin(9600);

  Motor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_ENA);
  Motor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_ENB);
  Encoder leftEnc(LEFT_ENCA, LEFT_ENCB, LEFT_ENCA, RIGHT_ENCA);
  Encoder rightEnc(RIGHT_ENCA, RIGHT_ENCB, LEFT_ENCA, RIGHT_ENCA);

  leftMotor.begin();
  rightMotor.begin();
  leftEnc.begin();
  rightEnc.begin();

  leftEnc.attachInterruptHandler();
  rightEnc.attachInterruptHandler();

  Movement movement(leftMotor, rightMotor, leftEnc, rightEnc, gyro);
}

void loop(){
  //test
  movement.moveForward(50); // Move forward 50 cm
  delay(1000);
  movement.turnRight(90); // Turn right 90 degrees
  delay(1000);
  movement.moveBackward(50); // Move backward 50 cm
  delay(1000);
  movement.turnLeft(90); // Turn left 90 degrees
  delay(1000);

}