#include "io.h"
#include "servo_functions.h"

constexpr uint8_t HopperSensorPins[NUM_SERVOS] = {
    12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 22, 24, 25, 26, 27, 28, 29,
    30, 32, 32, 33, 34, 35, 37, 38, 44, 43, 42, 41, 40, 29, 53, 52,
    51, 50, 49, 48, 47, 46, 45
  };

bool sensorActive(uint8_t servo) {
  return digitalRead(HopperSensorPins[servo]) == HIGH; // 1 = detected
}

void ioSetup() {
  for (uint8_t i = 0; i < NUM_BOARDS; i++) {
    pinMode(HopperSensorPins[i], INPUT);
  }
}