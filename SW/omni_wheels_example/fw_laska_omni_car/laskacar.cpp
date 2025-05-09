#include "laskacar.hpp"


LaskaOmniCar::LaskaOmniCar(Adafruit_DCMotor *BackLeftMotor,
                   Adafruit_DCMotor *BackRightMotor,
                   Adafruit_DCMotor *FrontRightMotor,
                   Adafruit_DCMotor *FrontLeftMotor) {
  this->BackLeftMotor = BackLeftMotor;
  this->BackRightMotor = BackRightMotor;
  this->FrontRightMotor = FrontRightMotor;
  this->FrontLeftMotor = FrontLeftMotor;
}

void LaskaOmniCar::move(int frontback, int leftright, int spin) {
  // Serial.print("FB: "); Serial.println(frontback);
  // Serial.print("LR: "); Serial.println(leftright);
  // Serial.print("SP: "); Serial.println(spin);

  // calculate the motor speeds
  int lf = this->clamp(-frontback + leftright + spin, -255, 255);
  int lb = this->clamp(-frontback - leftright + spin, -255, 255);
  int rf = this->clamp(-frontback - leftright - spin, -255, 255);
  int rb = this->clamp(-frontback + leftright - spin, -255, 255);

  // Serial.print(lf); Serial.print(" ");
  // Serial.print(lb); Serial.print(" ");
  // Serial.print(rf); Serial.print(" ");
  // Serial.print(rb); Serial.println();

  // back motors have opposite polarity
  // than the front motors
  this->run_motor_speed(this->FrontLeftMotor, lf);
  this->run_motor_speed(this->BackLeftMotor, lb);
  this->run_motor_speed(this->FrontRightMotor, rf);
  this->run_motor_speed(this->BackRightMotor, rb);
}

void LaskaOmniCar::run_motor_speed(Adafruit_DCMotor *motor, int speed) {
  motor->setSpeed(abs(speed));
  if (speed == 0) {
    motor->run(RELEASE);
  } else if (speed > 0) {
    motor->run(FORWARD);
  } else {
    motor->run(BACKWARD);
  }
}

int LaskaOmniCar::clamp(int val, int min, int max) {
  if (val < min) {
    return min;
  }
  if (val > max) {
    return max;
  }
  return val;
}
