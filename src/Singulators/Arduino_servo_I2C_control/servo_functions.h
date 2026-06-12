#ifndef SERVO_FUNCTIONS_H
#define SERVO_FUNCTIONS_H
  #include <Arduino.h>
  #include <Adafruit_PWMServoDriver.h>
  #include "comms.h"

  constexpr uint8_t PCA9685_I2C_ADDRESS_1  = PCA9685_I2C_ADDRESS;
  constexpr uint8_t PCA9685_I2C_ADDRESS_2 = PCA9685_I2C_ADDRESS + 1;
  constexpr uint8_t PCA9685_I2C_ADDRESS_3 = PCA9685_I2C_ADDRESS + 2;
  constexpr uint8_t NUM_BOARDS = 3;
  constexpr uint8_t CHANNELS_PER_BOARD = 16;
  constexpr uint8_t NUM_SERVOS = NUM_BOARDS * CHANNELS_PER_BOARD;

  constexpr uint8_t SERVO_FREQ_HZ = 50; //Standard frequency for analog servos
  constexpr uint8_t MAX_RETRIES = 3;

  // Pulse length counts (out of 4096) corresponding to servo endpoints.
  // These are typical for SG90 / MG90S; tune per servo if needed.
  // At 50 Hz, one period = 20 ms = 4096 counts -> 1 count ≈ 4.88 µs
  //   500 µs  ≈ 102 counts  (0°)
  //   1500 µs ≈ 307 counts  (90°)
  //   2500 µs ≈ 512 counts  (180°)
  constexpr uint16_t SERVO_MIN_COUNT = 102;
  constexpr uint16_t SERVO_MAX_COUNT = 512;
  constexpr uint8_t SERVO_ACTIVE_ANGLE = 90; //Need some fine tuneing
  constexpr uint8_t SERVO_REST_ANGLE = 0; //Need some finetuneing.

  constexpr uint16_t MOVE_SETTLE_MS = 250; // Wait after commanding move

  void setServoAngle(uint8_t index, uint8_t angleDeg);
  void releaseServo(uint8_t index);
  void setupServos();

#endif //SERVO_FUNCTIONS_H