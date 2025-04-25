#include <Adafruit_MotorShield.h>

class LaskaOmniCar {
private:
  Adafruit_DCMotor *BackLeftMotor;
  Adafruit_DCMotor *BackRightMotor;
  Adafruit_DCMotor *FrontRightMotor;
  Adafruit_DCMotor *FrontLeftMotor;

public:
  LaskaOmniCar(Adafruit_DCMotor *BackLeftMotor,
      Adafruit_DCMotor *BackRightMotor,
      Adafruit_DCMotor *FrontRightMotor,
      Adafruit_DCMotor *FrontLeftMotor);

  /**
   * Move the car based on the values of gamepad
   * joysticks.
   *
   * @param frontback frontback axis input -255 to 255
   * @param leftright leftright axis input -255 to 255
   * @param spin spin axis input -255 to 255
   */
  void move(int frontback, int leftright, int spin);

private:
  /**
   *  Runs the motor with specified speed and direction.
   *  Direction is expressed as the sign of speed value.
   */
  void run_motor_speed(Adafruit_DCMotor *motor, int speed);
  int clamp(int val, int min, int max);
};
