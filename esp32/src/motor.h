#include <Arduino.h>

#define MotorPWMfreq 30000
#define PWMA_pin 13
#define AIN1 33
#define AIN2 12
#define PWMB_pin 14
#define BIN1 26
#define BIN2 27
#define STBY 25

void motorsetup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY,1);
  ledcSetup(0, MotorPWMfreq, 8);
  ledcAttachPin(PWMA_pin, 0);
  ledcSetup(1, MotorPWMfreq, 8);
  ledcAttachPin(PWMB_pin, 1);
}
void motor(byte ch, int speed) {
  if (ch == 1) {
    if (speed > 0) {
      digitalWrite(AIN1, 1);
      digitalWrite(AIN2, 0);
      ledcWrite(0, speed);
    } else {
      digitalWrite(AIN1, 1);
      digitalWrite(AIN2, 0);
      ledcWrite(0, -speed);
    }
  } else if (ch == 2) {
    if (speed > 0) {
      digitalWrite(BIN1, 0);
      digitalWrite(BIN2, 1);
      ledcWrite(1, speed);
    } else {
      digitalWrite(BIN1, 1);
      digitalWrite(BIN2, 0);
      ledcWrite(1, -speed);
    }
  }
}

void motor2F(int speed1, int speed2) {
  motor(1, speed1);
  motor(2, speed2);
}

void motor2(int speed1, int speed2) {
  motor(1, speed1);
  motor(2, speed2);
}