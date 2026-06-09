
#include <Servo.h>

#define SERVOS_PER_ARDUINO 12 // Max supported servo's for Servo.h
#define ARDUINO_ID = 2
#define SERVO_START_POS = 90 // Default resting position of the servo's

// Pins that control Servo's (lowest will be servo 0)

const uint8_t ServoOutputPins[SERVOS_PER_ARDUINO] = {
  2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
};

const uint8_t DropSensorPins[SERVOS_PER_ARDUINO] = {
  22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44
};

Servo servoList[SERVOS_PER_ARDUINO];

void setup() {

  //Atach Pins to servo's
  for (uint8_t i; i < SERVOS_PER_ARDUINO; i++) {
      servoList[i].attach(ServoOutputPins[i]);
      servoList[i].write(SERVO_START_POS);
  }
}

void loop() {

}