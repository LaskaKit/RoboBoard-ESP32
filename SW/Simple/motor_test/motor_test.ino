// Motor test
#include <Adafruit_GFX.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x40);

Adafruit_DCMotor *M1 = AFMS.getMotor(3);
Adafruit_DCMotor *M2 = AFMS.getMotor(4);
Adafruit_DCMotor *M3 = AFMS.getMotor(1);
Adafruit_DCMotor *M4 = AFMS.getMotor(2);

void setup() {
  Serial.begin(115200);

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Driver. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  uint8_t i;

  Serial.println("forward");

  M4->run(FORWARD);
  for (i=0; i<255; i++) {
    M4->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    M4->setSpeed(i);
    delay(10);
  }

  Serial.println("backward");

  M4->run(BACKWARD);
  for (i=0; i<255; i++) {
    M4->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    M4->setSpeed(i);
    delay(10);
  }

  Serial.println("stop");
  M4->run(RELEASE);
}

void loop() {
  
}
