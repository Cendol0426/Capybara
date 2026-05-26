#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <PS2X_lib.h>

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
#define PS2CLK 13
#define PS2CMD 12
#define PS2SEL 11
#define PS2DAT 10

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

class Drive{
  private:
    Motor &leftMotor;
    Motor &rightMotor;

    PS2X ps2;
    int ps2clk, ps2cmd, ps2sel, ps2dat;

    String buffer;

    int constrainSpeed(int spd){
      return constrain(spd, -255, 255);
    }

    void setMotor(Motor &motor, int speed){
      speed = constrainSpeed(speed);
      if (speed > 0){motor.forward();}
      else if (speed < 0){motor.backward();}
      else{motor.stop();}

      motor.setSpeed(abs(speed));
    }

  public:
    Drive(Motor &lm, Motor &rm, int clk, int cmd, int sel, int dat): leftMotor(lm), rightMotor(rm), ps2clk(clk), ps2cmd(cmd), ps2sel(sel), ps2dat(dat) {}

    void begin(){
      leftMotor.begin(); 
      rightMotor.begin();
      int err = ps2.config_gamepad(ps2clk, ps2cmd, ps2sel, ps2dat, false, false);
      if(err == 0){
        Serial.println("Controller Found and Configured");
      }
      else if(err == 1){
        Serial.println("No controller found, check wiring");
      }
      else if(err == 2){
        Serial.println("Controller found but not accepting commands");
      }
      else if(err == 3){
        Serial.println("Controller refusing to enter config mode");
      }
    }

    void stop(){tank(0, 0);}

    void tank(int leftSpeed, int rightSpeed){
      setMotor(leftMotor, leftSpeed);
      setMotor(rightMotor, rightSpeed);
    }


    void update(){
      ps2.read_gamepad(false, 0);

      int leftY = ps2.Analog(PSS_LY);
      int rightY = ps2.Analog(PSS_RY);

      // Map 0-255 to -255 to 255 with deadzone
      int leftSpeed = map(leftY, 0, 255, 255, -255);
      int rightSpeed = map(rightY, 0, 255, 255, -255);

      // Apply deadzone
      if (abs(leftSpeed) < 20) leftSpeed = 0;
      if (abs(rightSpeed) < 20) rightSpeed = 0;

      tank(leftSpeed, rightSpeed);
    
    }
};

Motor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_ENA);
Motor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_ENB);
Drive drive(leftMotor, rightMotor, PS2CLK, PS2CMD, PS2SEL, PS2DAT);

void setup(){
  Serial.begin(115200);
  drive.begin();
}

void loop(){
  drive.update();
}